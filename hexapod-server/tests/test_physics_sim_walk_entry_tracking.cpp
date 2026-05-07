#include "control_config.hpp"
#include "motion_intent_utils.hpp"
#include "physics_sim_metrics_emit.hpp"
#include "physics_sim_test_argv.hpp"
#include "physics_sim_test_utils.hpp"
#include "test_limits_manifest.hpp"
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
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
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

std::string walkEntryTrackingLimitsJson(const double min_body_height_m,
                                        const double min_static_stability_margin_m,
                                        const double max_stance_contact_mismatch,
                                        const double max_worst_leg_tracking_error_rad) {
    std::ostringstream o;
    o << std::setprecision(17) << "{\"min_body_height_m\":" << min_body_height_m
      << ",\"min_static_stability_margin_m\":" << min_static_stability_margin_m
      << ",\"max_stance_contact_mismatch\":" << max_stance_contact_mismatch
      << ",\"max_worst_leg_tracking_error_rad\":" << max_worst_leg_tracking_error_rad << '}';
    return o.str();
}

} // namespace

int main(int argc, char** argv) {
#if !defined(__linux__)
    std::cout << "skip test_physics_sim_walk_entry_tracking (Linux-only)\n";
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
        std::cout << "skip test_physics_sim_walk_entry_tracking (pass sim path or HEXAPOD_PHYSICS_SIM_EXE)\n";
        return 0;
    }

    constexpr const char* kSuite = "physics_sim_walk_entry_tracking";
    constexpr const char* kCase = "walk_entry_tracking";
    const double kMinBodyHeightM = test_limits::getDouble(kSuite, kCase, "", "min_body_height_m", 0.08);
    const double kMinStaticStabilityMarginM =
        test_limits::getDouble(kSuite, kCase, "", "min_static_stability_margin_m", -0.03);
    const int kMaxStanceContactMismatch = static_cast<int>(test_limits::getDouble(
        kSuite, kCase, "", "max_stance_contact_mismatch", 5.0));
    const double kMaxWorstLegTrackingErrorRad =
        test_limits::getDouble(kSuite, kCase, "", "max_worst_leg_tracking_error_rad", 6.5);
    const auto limitsJson = [&]() {
        return walkEntryTrackingLimitsJson(kMinBodyHeightM,
                                           kMinStaticStabilityMarginM,
                                           static_cast<double>(kMaxStanceContactMismatch),
                                           kMaxWorstLegTrackingErrorRad);
    };

    const auto harness = physics_sim_test_utils::loadHarnessSettings();
    const int port = 25000 + (static_cast<int>(::getpid()) % 4000);
    const int bus_loop_period_us = harness.bus_loop_period_us;

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

    auto bridge = std::make_unique<CapturingPhysicsSimBridge>(
        "127.0.0.1", port, bus_loop_period_us, harness.physics_solver_iterations);
    CapturingPhysicsSimBridge* bridge_ptr = bridge.get();

    auto replay_logger = std::make_unique<CollectingReplayLogger>();
    CollectingReplayLogger* replay_ptr = replay_logger.get();

    control_config::ControlConfig cfg = harness.control_cfg;
    cfg.freshness.estimator.max_allowed_age_us = DurationUs{10'000'000};
    cfg.freshness.intent.max_allowed_age_us = DurationUs{10'000'000};
    cfg.locomotion_cmd.enable_first_order_filter = false;
    cfg.locomotion_cmd.enable_chassis_accel_limit = false;

    RobotRuntime runtime(
        std::move(bridge), std::make_unique<PhysicsSimEstimator>(), nullptr, cfg, telemetry::makeNoopTelemetryPublisher(), std::move(replay_logger));
    if (!expect(runtime.init(), "runtime init should succeed against the live physics sim")) {
        if (emit_metrics_json) {
            physics_sim_metrics::emitLine("physics_sim_walk_entry_tracking", "walk_entry_tracking", false,
                                          limitsJson(), "{\"stage\":\"runtime_init_failed\"}");
        }
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return EXIT_FAILURE;
    }

    const ScenarioMotionIntent stand_motion{true, RobotMode::STAND, GaitType::TRIPOD, 0.14, 0.0, 0.0, 0.0};
    const ScenarioMotionIntent walk_motion{true, RobotMode::WALK, GaitType::TRIPOD, 0.14, 0.08, 0.0, 0.0};

    const int kStandWarmupSteps = static_cast<int>(
        physics_sim_test_utils::scaledLegacyStepCount(140, bus_loop_period_us));
    const int kWalkObserveSteps = static_cast<int>(
        physics_sim_test_utils::scaledLegacyStepCount(160, bus_loop_period_us));

    for (int i = 0; i < kStandWarmupSteps; ++i) {
        runControlLoopStep(runtime, stand_motion);
    }
    for (int i = 0; i < kWalkObserveSteps; ++i) {
        runControlLoopStep(runtime, walk_motion);
    }

    ::kill(pid, SIGTERM);
    ::waitpid(pid, nullptr, 0);

    if (!expect(bridge_ptr->last_state().has_value(), "bridge should produce live sim state during walk entry")) {
        if (emit_metrics_json) {
            physics_sim_metrics::emitLine("physics_sim_walk_entry_tracking", "walk_entry_tracking", false,
                                          limitsJson(), "{\"stage\":\"no_bridge_state\"}");
        }
        return EXIT_FAILURE;
    }

    std::vector<replay_json::ReplayTelemetryRecord> walk_records{};
    for (const auto& record : replay_ptr->records) {
        if (record.status.active_mode == RobotMode::WALK) {
            walk_records.push_back(record);
        }
    }

    const std::size_t kRequiredWalkRecords =
        physics_sim_test_utils::scaledLegacyStepCount(60, bus_loop_period_us);
    if (!expect(walk_records.size() >= kRequiredWalkRecords,
                "replay logger should capture a full early WALK analysis window")) {
        if (emit_metrics_json) {
            std::ostringstream metrics;
            metrics << "{\"stage\":\"insufficient_walk_records\",\"walk_record_count\":" << walk_records.size() << '}';
            physics_sim_metrics::emitLine("physics_sim_walk_entry_tracking", "walk_entry_tracking", false,
                                          limitsJson(), metrics.str());
        }
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

    const bool ok = expect(min_body_height_m > kMinBodyHeightM,
                           "walk entry smoke guard should keep body height above a collapse floor") &&
                    expect(min_margin_m > kMinStaticStabilityMarginM,
                           "walk entry smoke guard should avoid a large static stability deficit") &&
                    expect(max_mismatch <= kMaxStanceContactMismatch,
                           "walk entry smoke guard should keep stance/contact mismatch bounded") &&
                    expect(worst_error < kMaxWorstLegTrackingErrorRad,
                           "walk entry smoke guard should keep per-leg joint tracking below the coarse peak threshold");
    if (emit_metrics_json) {
        std::ostringstream leg_err;
        leg_err << std::setprecision(17) << '[';
        for (int leg = 0; leg < kNumLegs; ++leg) {
            if (leg > 0) {
                leg_err << ',';
            }
            leg_err << max_tracking_error_by_leg[static_cast<std::size_t>(leg)];
        }
        leg_err << ']';
        std::ostringstream metrics;
        metrics << std::setprecision(17) << "{\"min_body_height_m\":" << min_body_height_m
                << ",\"min_static_stability_margin_m\":" << min_margin_m
                << ",\"max_stance_contact_mismatch\":" << max_mismatch
                << ",\"worst_leg_index\":" << worst_leg
                << ",\"worst_peak_tracking_error_rad\":" << worst_error
                << ",\"max_tracking_error_by_leg_rad\":" << leg_err.str() << '}';
        physics_sim_metrics::emitLine("physics_sim_walk_entry_tracking", "walk_entry_tracking", ok,
                                      limitsJson(), metrics.str());
    }
    return ok ? EXIT_SUCCESS : EXIT_FAILURE;
#endif
}
