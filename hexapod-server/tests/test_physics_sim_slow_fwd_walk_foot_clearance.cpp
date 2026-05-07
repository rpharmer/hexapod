#include "body_controller.hpp"
#include "control_config.hpp"
#include "geometry_config.hpp"
#include "leg_fk.hpp"
#include "motion_intent_utils.hpp"
#include "physics_sim_metrics_emit.hpp"
#include "physics_sim_test_argv.hpp"
#include "test_limits_manifest.hpp"
#include "physics_sim_test_utils.hpp"
#include "physics_sim_bridge.hpp"
#include "physics_sim_estimator.hpp"
#include "robot_runtime.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
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

int rawFootContactCount(const RobotState& state) {
    int count = 0;
    for (const bool contact : state.foot_contacts) {
        if (contact) {
            ++count;
        }
    }
    return count;
}

std::string slowFwdWalkFootClearanceLimitsJson(const double min_foot_tip_world_z_m,
                                                const bool require_saw_raw_contact_loss,
                                                const double max_body_undershoot_m) {
    std::ostringstream o;
    o << std::setprecision(17) << "{\"min_foot_tip_world_z_m\":" << min_foot_tip_world_z_m
      << ",\"require_saw_raw_contact_loss\":" << (require_saw_raw_contact_loss ? "true" : "false")
      << ",\"max_body_undershoot_m\":" << max_body_undershoot_m << '}';
    return o.str();
}

} // namespace

int main(int argc, char** argv) {
#if !defined(__linux__)
    std::cout << "skip test_physics_sim_slow_fwd_walk_foot_clearance (Linux-only)\n";
    return 0;
#else
    bool emit_metrics_json = false;
    const char* sim_exe = nullptr;
    physics_sim_test_argv::parse(argc, argv, emit_metrics_json, sim_exe);
    std::string manifest_err;
    if (!test_limits::init(argc, argv, manifest_err)) {
        std::cerr << manifest_err << '\n';
        return 2;
    }
    if (sim_exe == nullptr || sim_exe[0] == '\0') {
        std::cout << "skip test_physics_sim_slow_fwd_walk_foot_clearance (pass sim path or HEXAPOD_PHYSICS_SIM_EXE)\n";
        return 0;
    }

    constexpr const char* kSuite = "physics_sim_slow_fwd_walk_foot_clearance";
    constexpr const char* kCase = "slow_fwd_walk_foot_clearance";
    const double kMinFootTipWorldZ =
        test_limits::getDouble(kSuite, kCase, "", "min_foot_tip_world_z_m", -1.0e-4);
    const bool kRequireSawRawContactLoss =
        test_limits::getBool(kSuite, kCase, "", "require_saw_raw_contact_loss", true);
    const double kMaxBodyUndershootM =
        test_limits::getDouble(kSuite, kCase, "", "max_body_undershoot_m", 0.010);
    const double kCommandedBodyHeightM =
        test_limits::getDouble(kSuite, kCase, "", "commanded_body_height_m", 0.14);
    const auto limitsJson = [&]() {
        return slowFwdWalkFootClearanceLimitsJson(kMinFootTipWorldZ, kRequireSawRawContactLoss, kMaxBodyUndershootM);
    };

    const auto harness = physics_sim_test_utils::loadHarnessSettings();
    const int port = 26000 + (static_cast<int>(::getpid()) % 4000);
    const int bus_loop_period_us = harness.bus_loop_period_us;

    pid_t pid = ::fork();
    if (pid < 0) {
        std::cerr << "fork failed\n";
        return 2;
    }
    if (pid == 0) {
        physics_sim_test_utils::quietChildProcessStdIo();
        const std::string port_str = std::to_string(port);
        ::execl(sim_exe, sim_exe, "--serve", "--serve-port", port_str.c_str(), nullptr);
        std::perror("execl");
        _exit(127);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds{250});

    auto bridge = std::make_unique<CapturingPhysicsSimBridge>(
        "127.0.0.1", port, bus_loop_period_us, harness.physics_solver_iterations);
    CapturingPhysicsSimBridge* bridge_ptr = bridge.get();

    control_config::ControlConfig cfg = harness.control_cfg;
    cfg.freshness.estimator.max_allowed_age_us = DurationUs{10'000'000};
    cfg.freshness.intent.max_allowed_age_us = DurationUs{10'000'000};

    RobotRuntime runtime(
        std::move(bridge), std::make_unique<PhysicsSimEstimator>(), nullptr, cfg, telemetry::makeNoopTelemetryPublisher());
    if (!expect(runtime.init(), "runtime init should succeed against the live physics sim")) {
        if (emit_metrics_json) {
            physics_sim_metrics::emitLine("physics_sim_slow_fwd_walk_foot_clearance", "slow_fwd_walk_foot_clearance", false,
                                          limitsJson(), "{\"stage\":\"runtime_init_failed\"}");
        }
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return EXIT_FAILURE;
    }

    // Scenario 05, t=3-12 s: TRIPOD 0.06 m/s straight forward — smallest non-fast swing height
    // in the forward-walk phases.  This exercises the swing floor at low speed without yaw.
    const ScenarioMotionIntent stand_motion{true, RobotMode::STAND, GaitType::TRIPOD, 0.14, 0.0, 0.0, 0.0};
    const ScenarioMotionIntent walk_motion{true, RobotMode::WALK, GaitType::TRIPOD, 0.14, 0.06, 0.0, 0.0};

    const int kStandWarmupSteps = static_cast<int>(
        physics_sim_test_utils::scaledLegacyStepCount(140, bus_loop_period_us));
    const int kWalkObserveSteps = static_cast<int>(
        physics_sim_test_utils::scaledLegacyStepCount(1400, bus_loop_period_us));
    for (int i = 0; i < kStandWarmupSteps; ++i) {
        runControlLoopStep(runtime, stand_motion);
    }

    if (!expect(bridge_ptr->last_state().has_value(), "bridge should produce an initial state before walking")) {
        if (emit_metrics_json) {
            physics_sim_metrics::emitLine("physics_sim_slow_fwd_walk_foot_clearance", "slow_fwd_walk_foot_clearance", false,
                                          limitsJson(), "{\"stage\":\"no_initial_state\"}");
        }
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return EXIT_FAILURE;
    }

    double min_foot_tip_world_z_m = std::numeric_limits<double>::infinity();
    double min_body_height_m = std::numeric_limits<double>::infinity();
    bool saw_any_raw_contact_loss = false;

    for (int i = 0; i < kWalkObserveSteps; ++i) {
        runControlLoopStep(runtime, walk_motion);

        const ControlStatus status = runtime.getStatus();
        if (!expect(bridge_ptr->last_state().has_value(), "bridge should keep producing live state during walk")) {
            ::kill(pid, SIGTERM);
            ::waitpid(pid, nullptr, 0);
            if (emit_metrics_json) {
                std::ostringstream metrics;
                metrics << std::setprecision(17) << "{\"stage\":\"lost_state_during_walk\",\"walk_step\":" << i
                        << ",\"min_body_height_m\":" << min_body_height_m
                        << ",\"min_foot_tip_world_z_m\":" << min_foot_tip_world_z_m
                        << ",\"saw_any_raw_contact_loss\":" << (saw_any_raw_contact_loss ? "true" : "false") << '}';
                physics_sim_metrics::emitLine("physics_sim_slow_fwd_walk_foot_clearance", "slow_fwd_walk_foot_clearance", false,
                                              limitsJson(), metrics.str());
            }
            return EXIT_FAILURE;
        }
        const RobotState& state = bridge_ptr->last_state().value();
        min_body_height_m = std::min(min_body_height_m, state.body_twist_state.body_trans_m.z);
        min_foot_tip_world_z_m = std::min(min_foot_tip_world_z_m, minFootTipWorldZ(state));
        saw_any_raw_contact_loss = saw_any_raw_contact_loss || (rawFootContactCount(state) < kNumLegs);

        if (status.active_fault != FaultCode::NONE) {
            std::cerr << "walk step=" << i
                      << " fault=" << static_cast<int>(status.active_fault)
                      << " body_height=" << state.body_twist_state.body_trans_m.z
                      << " min_foot_tip_z=" << min_foot_tip_world_z_m
                      << '\n';
            ::kill(pid, SIGTERM);
            ::waitpid(pid, nullptr, 0);
            if (emit_metrics_json) {
                const double undershoot = kCommandedBodyHeightM - min_body_height_m;
                std::ostringstream metrics;
                metrics << std::setprecision(17) << "{\"stage\":\"fault_during_walk\",\"walk_step\":" << i
                        << ",\"min_body_height_m\":" << min_body_height_m
                        << ",\"max_body_undershoot_m\":" << undershoot
                        << ",\"min_foot_tip_world_z_m\":" << min_foot_tip_world_z_m
                        << ",\"saw_any_raw_contact_loss\":" << (saw_any_raw_contact_loss ? "true" : "false") << '}';
                physics_sim_metrics::emitLine("physics_sim_slow_fwd_walk_foot_clearance", "slow_fwd_walk_foot_clearance", false,
                                              limitsJson(), metrics.str());
            }
            return EXIT_FAILURE;
        }
    }

    ::kill(pid, SIGTERM);
    ::waitpid(pid, nullptr, 0);

    const double max_body_undershoot_m = kCommandedBodyHeightM - min_body_height_m;

    std::cout << "slow_fwd_walk_min_body_height_m=" << min_body_height_m
              << " max_body_undershoot_m=" << max_body_undershoot_m
              << " min_foot_tip_world_z_m=" << min_foot_tip_world_z_m
              << " saw_any_raw_contact_loss=" << (saw_any_raw_contact_loss ? 1 : 0) << '\n';

    bool ok = true;
    ok = expect(min_foot_tip_world_z_m >= kMinFootTipWorldZ,
                "slow forward TRIPOD walk should keep the reported server foot tip at or above the ground plane") &&
         ok;
    ok = expect(!kRequireSawRawContactLoss || saw_any_raw_contact_loss,
                "slow forward TRIPOD walk should see at least one swing-leg contact loss confirming feet lift") &&
         ok;
    ok = expect(max_body_undershoot_m < kMaxBodyUndershootM,
                "body height should track within 10 mm of commanded 0.14 m during slow forward walk") &&
         ok;
    if (emit_metrics_json) {
        std::ostringstream metrics;
        metrics << std::setprecision(17) << "{\"min_body_height_m\":" << min_body_height_m
                << ",\"max_body_undershoot_m\":" << max_body_undershoot_m
                << ",\"min_foot_tip_world_z_m\":" << min_foot_tip_world_z_m
                << ",\"saw_any_raw_contact_loss\":" << (saw_any_raw_contact_loss ? "true" : "false") << '}';
        physics_sim_metrics::emitLine("physics_sim_slow_fwd_walk_foot_clearance", "slow_fwd_walk_foot_clearance", ok,
                                      limitsJson(), metrics.str());
    }
    return ok ? EXIT_SUCCESS : EXIT_FAILURE;
#endif
}
