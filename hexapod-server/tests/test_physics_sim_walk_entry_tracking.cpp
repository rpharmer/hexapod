#include "control_config.hpp"
#include "motion_intent_utils.hpp"
#include "motion_trace.hpp"
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
#include <fstream>
#include <filesystem>
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
                              PhysicsSimSolverSettings solver_settings)
        : inner_(std::move(host), port, bus_loop_period_us, solver_settings, nullptr) {}

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

void runControlLoopStep(RobotRuntime& runtime, const ScenarioMotionIntent& motion,
                        TimePointUs& command_time, int period_us) {
    auto intent = makeMotionIntent(motion);
    command_time.value += static_cast<std::uint64_t>(period_us);
    intent.timestamp_us = command_time;
    runtime.setMotionIntent(intent);
    runtime.busStep();
    runtime.estimatorStep();
    runtime.safetyStep();
    runtime.controlStep();
}

std::string walkEntryTrackingLimitsJson(const double min_body_height_m,
                                        const double min_static_stability_margin_m,
                                        const double max_stance_contact_mismatch,
                                        const double max_worst_leg_tracking_error_rad,
                                        const double max_measured_joint_speed_radps) {
    std::ostringstream o;
    o << std::setprecision(17) << "{\"min_body_height_m\":" << min_body_height_m
      << ",\"min_static_stability_margin_m\":" << min_static_stability_margin_m
      << ",\"max_stance_contact_mismatch\":" << max_stance_contact_mismatch
      << ",\"max_worst_leg_tracking_error_rad\":" << max_worst_leg_tracking_error_rad
      << ",\"max_measured_joint_speed_radps\":" << max_measured_joint_speed_radps << '}';
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
    const double kMaxMeasuredJointSpeedRadps =
        // The simulated servo cap is 6 rad/s. Leave solver tolerance so a velocity-clamped walk entry
        // fails without rejecting normal transient tracking below that physical limit.
        test_limits::getDouble(kSuite, kCase, "", "max_measured_joint_speed_radps", 6.1);
    const auto limitsJson = [&]() {
        return walkEntryTrackingLimitsJson(kMinBodyHeightM,
                                           kMinStaticStabilityMarginM,
                                           static_cast<double>(kMaxStanceContactMismatch),
                                           kMaxWorstLegTrackingErrorRad,
                                           kMaxMeasuredJointSpeedRadps);
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
        "127.0.0.1", port, bus_loop_period_us,
        physics_sim_test_utils::productionProximalSolverSettings());
    CapturingPhysicsSimBridge* bridge_ptr = bridge.get();

    auto replay_logger = std::make_unique<CollectingReplayLogger>();
    CollectingReplayLogger* replay_ptr = replay_logger.get();

    control_config::ControlConfig cfg = harness.control_cfg;
    cfg.freshness.estimator.max_allowed_age_us = DurationUs{10'000'000};
    cfg.freshness.intent.max_allowed_age_us = DurationUs{10'000'000};
    // Exercise the production command shaping.  Disabling it masks precisely the walk-entry
    // discontinuity this test is intended to catch.

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
    const ScenarioMotionIntent walk_motion{true, RobotMode::WALK, GaitType::TRIPOD, 0.14, 0.04, 0.0, 0.0};

    // Match the configured two-second stand settling delay before judging WALK.  A shorter
    // warmup folds startup servo settling into this transition metric.
    const int kStandWarmupSteps = static_cast<int>(
        physics_sim_test_utils::scaledLegacyStepCount(550, bus_loop_period_us));
    const int kWalkObserveSteps = static_cast<int>(
        physics_sim_test_utils::scaledLegacyStepCount(160, bus_loop_period_us));

    // Advance command shaping on the same simulated clock as the bus. Host
    // runtime varies with CPU/logging load and is not simulated elapsed time.
    TimePointUs command_time{now_us().value + 3'600'000'000ULL};
    for (int i = 0; i < kStandWarmupSteps; ++i) {
        runControlLoopStep(runtime, stand_motion, command_time, bus_loop_period_us);
    }
    for (int i = 0; i < kWalkObserveSteps; ++i) {
        runControlLoopStep(runtime, walk_motion, command_time, bus_loop_period_us);
    }

    ::kill(pid, SIGTERM);
    ::waitpid(pid, nullptr, 0);

    if (const char* directory = std::getenv("HEXAPOD_MOTION_TRACE_DIR")) {
        const auto path = std::filesystem::path(directory) / "walk_entry.ndjson";
        if (std::filesystem::exists(path)) throw std::runtime_error("refusing to overwrite motion trace");
        std::ofstream out(path);
        if (!out) throw std::runtime_error("cannot open motion trace");
        for (const auto& record : replay_ptr->records)
            writeMotionTrace(out, record);
    }

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
    const replay_json::ReplayTelemetryRecord* min_margin_record = nullptr;
    int max_mismatch = 0;
    std::array<double, kNumLegs> max_tracking_error_by_leg{};
    int worst_leg = -1;
    double worst_error = -1.0;
    double max_measured_joint_speed_radps = 0.0;
    for (std::size_t i = 0; i < kRequiredWalkRecords; ++i) {
        const auto& record = walk_records[i];
        min_body_height_m =
            std::min(min_body_height_m, record.transition_diagnostics.body_height_m);
        if (record.gait_state.static_stability_margin_m < min_margin_m) {
            min_margin_m = record.gait_state.static_stability_margin_m;
            min_margin_record = &record;
        }
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
        for (int leg = 0; leg < kNumLegs; ++leg) {
            for (int joint = 0; joint < kJointsPerLeg; ++joint) {
                // PhysicsSimBridge reports this velocity from the articulated servo model.
                // Do not derive it from host-loop timestamps: this test intentionally drives
                // the sim faster than wall time, which would manufacture a meaningless spike.
                const double measured =
                    record.estimated_state.leg_states[leg].joint_state[joint].vel_radps.value;
                max_measured_joint_speed_radps = std::max(max_measured_joint_speed_radps, std::abs(measured));
            }
        }
    }

    std::cout << "walk_entry min_height_m=" << min_body_height_m
              << " min_margin_m=" << min_margin_m
              << " max_mismatch=" << max_mismatch
              << " worst_leg=" << worst_leg
              << " worst_peak_rad=" << worst_error
              << " max_measured_joint_speed_radps=" << max_measured_joint_speed_radps
              << '\n';
    if (min_margin_record != nullptr) {
        std::cout << "walk_entry min_margin_support=";
        for (int leg = 0; leg < kNumLegs; ++leg) {
            std::cout << (min_margin_record->locomotion_feasibility.support.effective_support[leg] ? '1' : '0');
        }
        std::cout << " planned_stance=";
        for (int leg = 0; leg < kNumLegs; ++leg) {
            std::cout << (min_margin_record->gait_state.in_stance[leg] ? '1' : '0');
        }
        std::cout << " nominal_margin_m=" << min_margin_record->locomotion_feasibility.nominal_margin_m
                  << " actual_margin_m=" << min_margin_record->locomotion_feasibility.actual_margin_m
                  << '\n';
    }

    const bool ok = expect(min_body_height_m > kMinBodyHeightM,
                           "walk entry smoke guard should keep body height above a collapse floor") &&
                    expect(min_margin_m > kMinStaticStabilityMarginM,
                           "walk entry smoke guard should avoid a large static stability deficit") &&
                    expect(max_mismatch <= kMaxStanceContactMismatch,
                           "walk entry smoke guard should keep stance/contact mismatch bounded") &&
                    expect(worst_error < kMaxWorstLegTrackingErrorRad,
                           "walk entry smoke guard should keep per-leg joint tracking below the coarse peak threshold") &&
                    expect(max_measured_joint_speed_radps < kMaxMeasuredJointSpeedRadps,
                           "walk entry should stay below the simulated servo velocity limit");
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
                << ",\"max_measured_joint_speed_radps\":" << max_measured_joint_speed_radps
                << ",\"max_tracking_error_by_leg_rad\":" << leg_err.str() << '}';
        physics_sim_metrics::emitLine("physics_sim_walk_entry_tracking", "walk_entry_tracking", ok,
                                      limitsJson(), metrics.str());
    }
    return ok ? EXIT_SUCCESS : EXIT_FAILURE;
#endif
}
