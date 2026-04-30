#include "control_config.hpp"
#include "motion_intent_utils.hpp"
#include "physics_sim_bridge.hpp"
#include "physics_sim_estimator.hpp"
#include "replay_logger.hpp"
#include "robot_runtime.hpp"
#include "scenario_driver.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <vector>

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

class CollectingReplayLogger final : public replay::IReplayLogger {
public:
    void write(const replay_json::ReplayTelemetryRecord& record) override {
        records.push_back(record);
    }

    std::vector<replay_json::ReplayTelemetryRecord> records{};
};

void runControlLoopStep(RobotRuntime& runtime, const ScenarioMotionIntent& motion) {
    runtime.setMotionIntent(makeMotionIntent(motion));
    runtime.busStep();
    runtime.estimatorStep();
    runtime.safetyStep();
    runtime.controlStep();
}

} // namespace

int main(int argc, char** argv) {
#if !defined(__linux__)
    std::cout << "skip test_physics_sim_walk_entry_tracking (Linux-only)\n";
    return 0;
#else
    const char* sim_exe = nullptr;
    if (argc >= 2 && argv[1][0] != '\0') {
        sim_exe = argv[1];
    } else {
        sim_exe = std::getenv("HEXAPOD_PHYSICS_SIM_EXE");
    }
    if (sim_exe == nullptr || sim_exe[0] == '\0') {
        std::cout << "skip test_physics_sim_walk_entry_tracking (pass sim path or HEXAPOD_PHYSICS_SIM_EXE)\n";
        return 0;
    }

    const int port = 25000 + (static_cast<int>(::getpid()) % 4000);
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

    auto replay_logger = std::make_unique<CollectingReplayLogger>();
    CollectingReplayLogger* replay_ptr = replay_logger.get();

    control_config::ControlConfig cfg{};
    cfg.freshness.estimator.max_allowed_age_us = DurationUs{10'000'000};
    cfg.freshness.intent.max_allowed_age_us = DurationUs{10'000'000};
    cfg.locomotion_cmd.enable_first_order_filter = false;
    cfg.locomotion_cmd.enable_chassis_accel_limit = false;

    RobotRuntime runtime(
        std::move(bridge), std::make_unique<PhysicsSimEstimator>(), nullptr, cfg, telemetry::makeNoopTelemetryPublisher(), std::move(replay_logger));
    if (!expect(runtime.init(), "runtime init should succeed against the live physics sim")) {
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return EXIT_FAILURE;
    }

    const ScenarioMotionIntent stand_motion{true, RobotMode::STAND, GaitType::TRIPOD, 0.14, 0.0, 0.0, 0.0};
    const ScenarioMotionIntent walk_motion{true, RobotMode::WALK, GaitType::TRIPOD, 0.14, 0.08, 0.0, 0.0};

    constexpr int kStandWarmupSteps = 140;
    constexpr int kWalkObserveSteps = 160;

    for (int i = 0; i < kStandWarmupSteps; ++i) {
        runControlLoopStep(runtime, stand_motion);
    }
    for (int i = 0; i < kWalkObserveSteps; ++i) {
        runControlLoopStep(runtime, walk_motion);
    }

    ::kill(pid, SIGTERM);
    ::waitpid(pid, nullptr, 0);

    if (!expect(bridge_ptr->last_state().has_value(), "bridge should produce live sim state during walk entry")) {
        return EXIT_FAILURE;
    }

    std::vector<replay_json::ReplayTelemetryRecord> walk_records{};
    for (const auto& record : replay_ptr->records) {
        if (record.status.active_mode == RobotMode::WALK) {
            walk_records.push_back(record);
        }
    }

    constexpr std::size_t kRequiredWalkRecords = 60;
    if (!expect(walk_records.size() >= kRequiredWalkRecords,
                "replay logger should capture at least 120 WALK records for entry analysis")) {
        return EXIT_FAILURE;
    }

    double min_body_height_m = 1e9;
    double min_margin_m = 1e9;
    int max_mismatch = 0;
    std::array<double, kNumLegs> max_tracking_error_by_leg{};
    int worst_leg = -1;
    double worst_error = -1.0;
    for (std::size_t i = 0; i < kRequiredWalkRecords; ++i) {
        const auto& record = walk_records[i];
        min_body_height_m =
            std::min(min_body_height_m, record.transition_diagnostics.body_height_m);
        min_margin_m =
            std::min(min_margin_m, record.gait_state.static_stability_margin_m);
        max_mismatch =
            std::max(max_mismatch, record.transition_diagnostics.stance_contact_mismatch_count);
        for (int leg = 0; leg < kNumLegs; ++leg) {
            const double err = record.transition_diagnostics.joint_tracking_max_abs_error_rad[leg];
            if (err > max_tracking_error_by_leg[static_cast<std::size_t>(leg)]) {
                max_tracking_error_by_leg[static_cast<std::size_t>(leg)] = err;
            }
            if (err > worst_error) {
                worst_error = err;
                worst_leg = leg;
            }
        }
    }

    std::cout << "walk_entry min_height_m=" << min_body_height_m
              << " min_margin_m=" << min_margin_m
              << " max_mismatch=" << max_mismatch
              << " worst_leg=" << worst_leg
              << " worst_peak_rad=" << worst_error
              << '\n';

    return expect(min_body_height_m > 0.08,
                  "walk entry smoke guard should keep body height above a collapse floor") &&
           expect(min_margin_m > -0.03,
                  "walk entry smoke guard should avoid a large static stability deficit") &&
           expect(max_mismatch <= 5,
                  "walk entry smoke guard should keep stance/contact mismatch bounded") &&
           expect(worst_error < 6.5,
                  "walk entry smoke guard should keep per-leg joint tracking below the coarse peak threshold")
               ? EXIT_SUCCESS
               : EXIT_FAILURE;
#endif
}
