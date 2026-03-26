#include "estimator.hpp"
#include "imu_unit.hpp"
#include "robot_runtime.hpp"
#include "sim_hardware_bridge.hpp"

#include <array>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace {

bool expect(bool condition, const std::string& message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

class ScriptedEstimator final : public IEstimator {
public:
    void enqueue(const RobotState& sample) {
        queue_.push_back(sample);
    }

    RobotState update(const RobotState& raw) override {
        if (!queue_.empty()) {
            last_ = queue_.front();
            queue_.erase(queue_.begin());
            return last_;
        }

        if (last_.timestamp_us.isZero()) {
            last_.sample_id = raw.sample_id;
            last_.timestamp_us = raw.timestamp_us;
            last_.foot_contacts = raw.foot_contacts;
            for (int leg = 0; leg < kNumLegs; ++leg) {
                for (int joint = 0; joint < kJointsPerLeg; ++joint) {
                    last_.leg_states[leg].joint_state[joint].pos_rad =
                        raw.leg_states[leg].joint_state[joint].pos_rad;
                }
            }
        }
        return last_;
    }

private:
    std::vector<RobotState> queue_{};
    RobotState last_{};
};

class CapturingEstimator final : public IEstimator {
public:
    RobotState update(const RobotState& raw) override {
        last_raw = raw;
        return raw;
    }

    RobotState last_raw{};
};

class FixedImuUnit final : public hardware::IImuUnit {
public:
    bool init() override {
        return true;
    }

    bool read(hardware::ImuSample& out) override {
        ++read_count;
        out = sample;
        return true;
    }

    hardware::ImuSample sample{};
    int read_count{0};
};

class CountingTelemetryPublisher final : public telemetry::ITelemetryPublisher {
public:
    void publishGeometry(const HexapodGeometry&) override {
        ++geometry_count;
    }

    void publishControlStep(const telemetry::ControlStepTelemetry&) override {
        ++control_step_count;
    }

    telemetry::TelemetryPublishCounters counters() const override {
        telemetry::TelemetryPublishCounters value{};
        value.packets_sent = static_cast<uint64_t>(geometry_count + control_step_count);
        return value;
    }

    size_t geometry_count{0};
    size_t control_step_count{0};
};

struct FlagScenario {
    const char* name;
    bool require_timestamp;
    bool require_nonzero_sample_id;
    bool require_monotonic_sample_id;
};

control_config::ControlConfig makeConfigFor(const FlagScenario& scenario) {
    control_config::ControlConfig cfg{};
    cfg.freshness.estimator.max_allowed_age_us = DurationUs{10'000'000};
    cfg.freshness.intent.max_allowed_age_us = DurationUs{10'000'000};

    cfg.freshness.estimator.require_timestamp = scenario.require_timestamp;
    cfg.freshness.estimator.require_nonzero_sample_id = scenario.require_nonzero_sample_id;
    cfg.freshness.estimator.require_monotonic_sample_id = scenario.require_monotonic_sample_id;

    cfg.freshness.intent.require_timestamp = scenario.require_timestamp;
    cfg.freshness.intent.require_nonzero_sample_id = scenario.require_nonzero_sample_id;
    cfg.freshness.intent.require_monotonic_sample_id = scenario.require_monotonic_sample_id;
    return cfg;
}

bool runControlStep(RobotRuntime& runtime) {
    runtime.busStep();
    runtime.estimatorStep();
    runtime.safetyStep();
    runtime.controlStep();
    return true;
}

RobotState makeEstimatorSample(uint64_t sample_id, TimePointUs timestamp_us) {
    RobotState est{};
    est.sample_id = sample_id;
    est.timestamp_us = timestamp_us;
    return est;
}

MotionIntent makeIntentSample(uint64_t sample_id, TimePointUs timestamp_us) {
    MotionIntent intent{};
    intent.requested_mode = RobotMode::WALK;
    intent.sample_id = sample_id;
    intent.timestamp_us = timestamp_us;
    return intent;
}

bool runEstimatorInvalidCase(const FlagScenario& scenario,
                             const RobotState& invalid_estimator,
                             const std::string& reason_label) {
    auto bridge = std::make_unique<SimHardwareBridge>();
    auto estimator = std::make_unique<ScriptedEstimator>();
    auto* scripted = estimator.get();
    RobotRuntime runtime(std::move(bridge), std::move(estimator), nullptr, makeConfigFor(scenario));

    if (!expect(runtime.init(), std::string{"init should succeed (estimator "} + scenario.name + ")")) {
        return false;
    }

    scripted->enqueue(makeEstimatorSample(100, now_us()));
    runtime.setMotionIntent(makeIntentSample(100, now_us()));
    runControlStep(runtime);

    const ControlStatus baseline = runtime.getStatus();
    if (!expect(baseline.active_mode == RobotMode::WALK,
                std::string{"baseline should allow WALK (estimator "} + scenario.name + ")")) {
        return false;
    }

    scripted->enqueue(invalid_estimator);
    runtime.setMotionIntent(makeIntentSample(101, now_us()));
    runControlStep(runtime);

    const ControlStatus rejected = runtime.getStatus();
    return expect(rejected.active_mode == RobotMode::SAFE_IDLE,
                  std::string{"invalid estimator should force SAFE_IDLE for "} + reason_label) &&
           expect(rejected.active_fault == FaultCode::ESTIMATOR_INVALID,
                  std::string{"invalid estimator should report ESTIMATOR_INVALID for "} + reason_label);
}

bool runIntentInvalidCase(const FlagScenario& scenario,
                          const MotionIntent& baseline_intent,
                          const MotionIntent& invalid_intent,
                          const std::string& reason_label) {
    auto bridge = std::make_unique<SimHardwareBridge>();
    auto estimator = std::make_unique<ScriptedEstimator>();
    auto* scripted = estimator.get();
    RobotRuntime runtime(std::move(bridge), std::move(estimator), nullptr, makeConfigFor(scenario));

    if (!expect(runtime.init(), std::string{"init should succeed (intent "} + scenario.name + ")")) {
        return false;
    }

    scripted->enqueue(makeEstimatorSample(200, now_us()));
    runtime.setMotionIntentForTest(baseline_intent);
    runControlStep(runtime);

    const ControlStatus baseline = runtime.getStatus();
    if (!expect(baseline.active_mode == RobotMode::WALK,
                std::string{"baseline should allow WALK (intent "} + scenario.name + ")")) {
        return false;
    }

    scripted->enqueue(makeEstimatorSample(201, now_us()));
    runtime.setMotionIntentForTest(invalid_intent);
    runControlStep(runtime);

    const ControlStatus rejected = runtime.getStatus();
    return expect(rejected.active_mode == RobotMode::SAFE_IDLE,
                  std::string{"invalid intent should force SAFE_IDLE for "} + reason_label) &&
           expect(rejected.active_fault == FaultCode::COMMAND_TIMEOUT,
                  std::string{"invalid intent should report COMMAND_TIMEOUT for "} + reason_label);
}

bool runTelemetryCadenceSuccessCase() {
    auto bridge = std::make_unique<SimHardwareBridge>();
    auto estimator = std::make_unique<ScriptedEstimator>();
    auto* scripted = estimator.get();
    auto telemetry = std::make_unique<CountingTelemetryPublisher>();
    auto* telemetry_raw = telemetry.get();

    control_config::ControlConfig cfg{};
    cfg.telemetry.enabled = true;
    cfg.telemetry.publish_period = std::chrono::milliseconds{200};
    cfg.telemetry.geometry_refresh_period = std::chrono::milliseconds{500};
    cfg.freshness.estimator.max_allowed_age_us = DurationUs{10'000'000};
    cfg.freshness.intent.max_allowed_age_us = DurationUs{10'000'000};
    RobotRuntime runtime(std::move(bridge), std::move(estimator), nullptr, cfg, std::move(telemetry));

    if (!expect(runtime.init(), "init should succeed (telemetry success cadence)")) {
        return false;
    }

    runtime.startTelemetry();
    if (!expect(telemetry_raw->geometry_count == 1, "startTelemetry should publish geometry once")) {
        return false;
    }

    scripted->enqueue(makeEstimatorSample(500, now_us()));
    runtime.setMotionIntent(makeIntentSample(500, now_us()));
    runControlStep(runtime);

    if (!expect(telemetry_raw->control_step_count == 1,
                "first successful control step should publish telemetry")) {
        return false;
    }

    scripted->enqueue(makeEstimatorSample(501, now_us()));
    runtime.setMotionIntent(makeIntentSample(501, now_us()));
    runControlStep(runtime);

    return expect(telemetry_raw->control_step_count == 1,
                  "second successful control step inside publish period should not publish telemetry") &&
           expect(telemetry_raw->geometry_count == 1,
                  "geometry should not refresh before geometry refresh period elapses");
}

bool runTelemetryCadenceRejectCase() {
    auto bridge = std::make_unique<SimHardwareBridge>();
    auto estimator = std::make_unique<ScriptedEstimator>();
    auto* scripted = estimator.get();
    auto telemetry = std::make_unique<CountingTelemetryPublisher>();
    auto* telemetry_raw = telemetry.get();

    control_config::ControlConfig cfg{};
    cfg.telemetry.enabled = true;
    cfg.telemetry.publish_period = std::chrono::milliseconds{200};
    cfg.telemetry.geometry_refresh_period = std::chrono::milliseconds{500};
    cfg.freshness.estimator.max_allowed_age_us = DurationUs{10'000'000};
    cfg.freshness.intent.max_allowed_age_us = DurationUs{10'000'000};
    cfg.freshness.intent.require_timestamp = true;
    RobotRuntime runtime(std::move(bridge), std::move(estimator), nullptr, cfg, std::move(telemetry));

    if (!expect(runtime.init(), "init should succeed (telemetry reject cadence)")) {
        return false;
    }

    runtime.startTelemetry();
    if (!expect(telemetry_raw->geometry_count == 1, "startTelemetry should publish geometry once")) {
        return false;
    }

    scripted->enqueue(makeEstimatorSample(600, now_us()));
    runtime.setMotionIntentForTest(makeIntentSample(600, TimePointUs{}));
    runControlStep(runtime);

    if (!expect(telemetry_raw->control_step_count == 1,
                "first rejected control step should publish telemetry")) {
        return false;
    }

    scripted->enqueue(makeEstimatorSample(601, now_us()));
    runtime.setMotionIntentForTest(makeIntentSample(601, TimePointUs{}));
    runControlStep(runtime);

    return expect(telemetry_raw->control_step_count == 1,
                  "second rejected control step inside publish period should not publish telemetry") &&
           expect(telemetry_raw->geometry_count == 1,
                  "rejected control steps should keep geometry refresh cadence");
}

bool runImuReadGateCase(bool imu_reads_enabled, bool expect_body_pose_state) {
    auto bridge = std::make_unique<SimHardwareBridge>();
    auto estimator = std::make_unique<CapturingEstimator>();
    auto* estimator_raw = estimator.get();
    auto imu = std::make_unique<FixedImuUnit>();
    auto* imu_raw = imu.get();
    imu_raw->sample.valid = true;
    imu_raw->sample.orientation_rad = EulerAnglesRad3{0.1, -0.2, 0.3};
    imu_raw->sample.angular_velocity_radps = AngularVelocityRadPerSec3{0.4, 0.5, -0.6};

    control_config::ControlConfig cfg{};
    cfg.runtime_imu.enable_reads = imu_reads_enabled;
    cfg.freshness.estimator.max_allowed_age_us = DurationUs{10'000'000};
    cfg.freshness.intent.max_allowed_age_us = DurationUs{10'000'000};
    RobotRuntime runtime(std::move(bridge), std::move(estimator), nullptr, cfg, nullptr, std::move(imu));

    if (!expect(runtime.init(), "init should succeed (imu read gate case)")) {
        return false;
    }

    runtime.busStep();
    runtime.estimatorStep();

    if (!expect(estimator_raw->last_raw.has_body_pose_state == expect_body_pose_state,
                "IMU read gate should control raw body_pose_state propagation")) {
        return false;
    }

    if (expect_body_pose_state) {
        return expect(imu_raw->read_count == 1, "enabled IMU reads should poll the IMU once per bus step") &&
               expect(estimator_raw->last_raw.body_pose_state.orientation_rad.x == 0.1,
                      "enabled IMU reads should propagate orientation into raw state");
    }
    return expect(imu_raw->read_count == 0, "disabled IMU reads should not poll IMU");
}

} // namespace

int main() {
    const std::array<FlagScenario, 3> scenarios{{
        {"require_timestamp", true, false, false},
        {"require_nonzero_sample_id", false, true, false},
        {"require_monotonic_sample_id", false, false, true},
    }};

    for (const FlagScenario& scenario : scenarios) {
        if (scenario.require_timestamp) {
            if (!runEstimatorInvalidCase(
                    scenario,
                    makeEstimatorSample(101, TimePointUs{}),
                    "zero estimator timestamp")) {
                return EXIT_FAILURE;
            }

            if (!runIntentInvalidCase(
                    scenario,
                    makeIntentSample(100, now_us()),
                    makeIntentSample(101, TimePointUs{}),
                    "zero intent timestamp")) {
                return EXIT_FAILURE;
            }
        }

        if (scenario.require_nonzero_sample_id) {
            if (!runEstimatorInvalidCase(
                    scenario,
                    makeEstimatorSample(0, now_us()),
                    "zero estimator sample_id")) {
                return EXIT_FAILURE;
            }

            if (!runIntentInvalidCase(
                    scenario,
                    makeIntentSample(100, now_us()),
                    makeIntentSample(0, now_us()),
                    "zero intent sample_id")) {
                return EXIT_FAILURE;
            }
        }

        if (scenario.require_monotonic_sample_id) {
            if (!runEstimatorInvalidCase(
                    scenario,
                    makeEstimatorSample(99, now_us()),
                    "decreasing estimator sample_id")) {
                return EXIT_FAILURE;
            }

            if (!runIntentInvalidCase(
                    scenario,
                    makeIntentSample(100, now_us()),
                    makeIntentSample(99, now_us()),
                    "decreasing intent sample_id")) {
                return EXIT_FAILURE;
            }
        }
    }

    if (!runTelemetryCadenceSuccessCase()) {
        return EXIT_FAILURE;
    }

    if (!runTelemetryCadenceRejectCase()) {
        return EXIT_FAILURE;
    }

    if (!runImuReadGateCase(false, false)) {
        return EXIT_FAILURE;
    }

    if (!runImuReadGateCase(true, true)) {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
