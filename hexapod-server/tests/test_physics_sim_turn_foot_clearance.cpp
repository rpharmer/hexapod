#include "body_controller.hpp"
#include "control_config.hpp"
#include "config/toml_parser.hpp"
#include "geometry_config.hpp"
#include "leg_fk.hpp"
#include "motion_intent_utils.hpp"
#include "physics_sim_bridge.hpp"
#include "physics_sim_estimator.hpp"
#include "robot_runtime.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <limits>
#include <optional>
#include <string>
#include <thread>

#if defined(__linux__)
#include <csignal>
#include <sys/wait.h>
#include <unistd.h>
#endif

namespace {

bool expect(bool condition, const std::string& message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

class CapturingPhysicsSimBridge final : public IHardwareBridge {
public:
    CapturingPhysicsSimBridge(std::string host,
                              int port,
                              int bus_loop_period_us,
                              int physics_solver_iterations)
        : inner_(std::move(host), port, bus_loop_period_us, physics_solver_iterations, nullptr) {}

    bool init() override { return inner_.init(); }

    bool read(RobotState& out) override {
        const bool ok = inner_.read(out);
        if (ok) {
            last_state_ = out;
        }
        return ok;
    }

    bool write(const JointTargets& in) override { return inner_.write(in); }

    std::optional<BridgeCommandResultMetadata> last_bridge_result() const override {
        return inner_.last_bridge_result();
    }

    const std::optional<RobotState>& last_state() const { return last_state_; }

private:
    PhysicsSimBridge inner_;
    std::optional<RobotState> last_state_{};
};

void runControlLoopStep(RobotRuntime& runtime, const ScenarioMotionIntent& motion) {
    runtime.setMotionIntent(makeMotionIntent(motion));
    runtime.busStep();
    runtime.estimatorStep();
    runtime.safetyStep();
    runtime.controlStep();
}

double minFootTipWorldZ(const RobotState& state) {
    const HexapodGeometry geometry = defaultHexapodGeometry();
    LegFK fk{};
    BodyPose body_pose{};
    body_pose.position = state.body_twist_state.body_trans_m;
    body_pose.roll = AngleRad{state.body_twist_state.twist_pos_rad.x};
    body_pose.pitch = AngleRad{state.body_twist_state.twist_pos_rad.y};
    body_pose.yaw = AngleRad{state.body_twist_state.twist_pos_rad.z};

    double min_z = std::numeric_limits<double>::infinity();
    for (int leg = 0; leg < kNumLegs; ++leg) {
        const FootTarget foot_world =
            fk.footInWorldFrame(state.leg_states[static_cast<std::size_t>(leg)], body_pose, geometry.legGeometry[leg]);
        min_z = std::min(min_z, foot_world.pos_body_m.z);
    }
    return min_z;
}

} // namespace

int main(int argc, char** argv) {
#if !defined(__linux__)
    std::cout << "skip test_physics_sim_turn_foot_clearance (Linux-only)\n";
    return 0;
#else
    const char* sim_exe = nullptr;
    if (argc >= 2 && argv[1][0] != '\0') {
        sim_exe = argv[1];
    } else {
        sim_exe = std::getenv("HEXAPOD_PHYSICS_SIM_EXE");
    }
    if (sim_exe == nullptr || sim_exe[0] == '\0') {
        std::cout << "skip test_physics_sim_turn_foot_clearance (pass sim path or HEXAPOD_PHYSICS_SIM_EXE)\n";
        return 0;
    }

    const int port = 26000 + (static_cast<int>(::getpid()) % 4000);
    const int bus_loop_period_us = 20000;

    pid_t pid = ::fork();
    if (pid < 0) {
        std::cerr << "fork failed\n";
        return 2;
    }
    if (pid == 0) {
        const std::string port_str = std::to_string(port);
        ::execl(sim_exe, sim_exe, "--serve", "--serve-port", port_str.c_str(), nullptr);
        std::perror("execl");
        _exit(127);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds{250});

    auto bridge = std::make_unique<CapturingPhysicsSimBridge>("127.0.0.1", port, bus_loop_period_us, 24);
    CapturingPhysicsSimBridge* bridge_ptr = bridge.get();

    ParsedToml parsed{};
    const std::string config_path = "../config.physics-sim-wsl.txt";
    const TomlParser parser{};
    if (!expect(parser.parse(config_path, parsed), "physics-sim WSL config should parse for turn regression")) {
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return EXIT_FAILURE;
    }

    control_config::ControlConfig cfg = control_config::fromParsedToml(parsed);
    cfg.freshness.estimator.max_allowed_age_us = DurationUs{10'000'000};
    cfg.freshness.intent.max_allowed_age_us = DurationUs{10'000'000};

    RobotRuntime runtime(
        std::move(bridge), std::make_unique<PhysicsSimEstimator>(), nullptr, cfg, telemetry::makeNoopTelemetryPublisher());
    if (!expect(runtime.init(), "runtime init should succeed against the live physics sim")) {
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return EXIT_FAILURE;
    }

    const ScenarioMotionIntent stand_motion{true, RobotMode::STAND, GaitType::TRIPOD, 0.14, 0.0, 0.0, 0.0};
    ScenarioMotionIntent turn_motion{true, RobotMode::WALK, GaitType::TRIPOD, 0.14, 0.0, 0.0, 0.0};
    turn_motion.yaw_rate_radps = 0.45;

    constexpr int kStandWarmupSteps = 140;
    constexpr int kTurnObserveSteps = 1400;

    for (int i = 0; i < kStandWarmupSteps; ++i) {
        runControlLoopStep(runtime, stand_motion);
    }

    if (!expect(bridge_ptr->last_state().has_value(), "bridge should produce an initial state before turning")) {
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return EXIT_FAILURE;
    }

    double min_foot_tip_world_z_m = std::numeric_limits<double>::infinity();
    double min_body_height_m = std::numeric_limits<double>::infinity();

    for (int i = 0; i < kTurnObserveSteps; ++i) {
        runControlLoopStep(runtime, turn_motion);

        const ControlStatus status = runtime.getStatus();
        if (!expect(bridge_ptr->last_state().has_value(), "bridge should keep producing live state during turn")) {
            ::kill(pid, SIGTERM);
            ::waitpid(pid, nullptr, 0);
            return EXIT_FAILURE;
        }
        const RobotState& state = bridge_ptr->last_state().value();
        min_body_height_m = std::min(min_body_height_m, state.body_twist_state.body_trans_m.z);
        min_foot_tip_world_z_m = std::min(min_foot_tip_world_z_m, minFootTipWorldZ(state));

        if (status.active_fault != FaultCode::NONE) {
            std::cerr << "turn step=" << i
                      << " fault=" << static_cast<int>(status.active_fault)
                      << " body_height=" << state.body_twist_state.body_trans_m.z
                      << " min_foot_tip_z=" << min_foot_tip_world_z_m
                      << '\n';
            ::kill(pid, SIGTERM);
            ::waitpid(pid, nullptr, 0);
            return EXIT_FAILURE;
        }
    }

    ::kill(pid, SIGTERM);
    ::waitpid(pid, nullptr, 0);

    std::cout << "turn_min_body_height_m=" << min_body_height_m
              << " turn_min_foot_tip_world_z_m=" << min_foot_tip_world_z_m << '\n';

    return expect(min_foot_tip_world_z_m >= -1.0e-4,
                  "turning motion should keep the reported server foot tip at or above the ground plane")
               ? EXIT_SUCCESS
               : EXIT_FAILURE;
#endif
}
