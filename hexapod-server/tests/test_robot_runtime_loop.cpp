#include "estimator.hpp"
#include "imu_unit.hpp"
#include "robot_runtime.hpp"
#include "sim_hardware_bridge.hpp"

#include <array>
#include <chrono>
#include <cmath>
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

bool jointTargetsEqual(const JointTargets& lhs, const JointTargets& rhs, double tolerance = 1e-12) {
    for (int leg = 0; leg < kNumLegs; ++leg) {
        for (int joint = 0; joint < kJointsPerLeg; ++joint) {
            const double lhs_pos = lhs.leg_states[leg].joint_state[joint].pos_rad.value;
            const double rhs_pos = rhs.leg_states[leg].joint_state[joint].pos_rad.value;
            if (std::abs(lhs_pos - rhs_pos) > tolerance) {
                return false;
            }
        }
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

    void publishControlStep(const telemetry::ControlStepTelemetry& telemetry_sample) override {
        ++control_step_count;
        last_control_step = telemetry_sample;
    }

    telemetry::TelemetryPublishCounters counters() const override {
        telemetry::TelemetryPublishCounters value{};
        value.packets_sent = static_cast<uint64_t>(geometry_count + control_step_count);
        return value;
    }

    size_t geometry_count{0};
    size_t control_step_count{0};
    std::optional<telemetry::ControlStepTelemetry> last_control_step{};
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

MotionIntent makeWalkingIntentSample(uint64_t sample_id, TimePointUs timestamp_us) {
    MotionIntent intent = makeIntentSample(sample_id, timestamp_us);
    intent.gait = GaitType::TRIPOD;
    intent.speed_mps = LinearRateMps{0.08};
    intent.heading_rad = AngleRad{0.0};
    intent.body_pose_setpoint.body_trans_m = Vec3{0.0, 0.0, 0.20};
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

bool runRuntimeFreshnessTelemetryCountersCase() {
    auto bridge = std::make_unique<SimHardwareBridge>();
    auto estimator = std::make_unique<ScriptedEstimator>();
    auto* scripted = estimator.get();
    auto telemetry = std::make_unique<CountingTelemetryPublisher>();
    auto* telemetry_raw = telemetry.get();

    control_config::ControlConfig cfg{};
    cfg.telemetry.enabled = true;
    cfg.telemetry.publish_period = std::chrono::milliseconds{0};
    cfg.freshness.estimator.max_allowed_age_us = DurationUs{10'000'000};
    cfg.freshness.intent.max_allowed_age_us = DurationUs{500};
    RobotRuntime runtime(std::move(bridge), std::move(estimator), nullptr, cfg, std::move(telemetry));

    if (!expect(runtime.init(), "init should succeed (freshness telemetry counters case)")) {
        return false;
    }
    runtime.startTelemetry();

    const TimePointUs baseline_now = now_us();
    scripted->enqueue(makeEstimatorSample(700, baseline_now));
    runtime.setMotionIntent(makeIntentSample(700, baseline_now));
    runControlStep(runtime);

    if (!expect(telemetry_raw->last_control_step.has_value(),
                "freshness telemetry case should capture baseline telemetry")) {
        return false;
    }

    scripted->enqueue(makeEstimatorSample(701, now_us()));
    runtime.setMotionIntentForTest(makeIntentSample(701, TimePointUs{1}));
    runControlStep(runtime);

    if (!expect(telemetry_raw->last_control_step.has_value(),
                "freshness telemetry case should capture reject telemetry")) {
        return false;
    }
    const auto rejected = telemetry_raw->last_control_step.value();
    if (!expect(rejected.runtime_freshness.total_rejects == 1,
                "reject telemetry should increment total freshness rejects") ||
        !expect(rejected.runtime_freshness.consecutive_rejects == 1,
                "reject telemetry should increment consecutive freshness rejects") ||
        !expect(rejected.runtime_freshness.stale_age_rejects == 1,
                "reject telemetry should attribute stale-age reject reason")) {
        return false;
    }

    scripted->enqueue(makeEstimatorSample(702, now_us()));
    runtime.setMotionIntent(makeIntentSample(702, now_us()));
    runControlStep(runtime);

    if (!expect(telemetry_raw->last_control_step.has_value(),
                "freshness telemetry case should capture recovery telemetry")) {
        return false;
    }
    const auto recovered = telemetry_raw->last_control_step.value();
    return expect(recovered.runtime_freshness.total_rejects == 1,
                  "recovery telemetry should keep total freshness rejects cumulative") &&
           expect(recovered.runtime_freshness.consecutive_rejects == 0,
                  "recovery telemetry should reset consecutive freshness rejects");
}

bool runMixedIntentPublisherSampleIdCase() {
    auto bridge = std::make_unique<SimHardwareBridge>();
    auto estimator = std::make_unique<ScriptedEstimator>();
    auto* scripted = estimator.get();

    control_config::ControlConfig cfg{};
    cfg.freshness.estimator.max_allowed_age_us = DurationUs{10'000'000};
    cfg.freshness.intent.max_allowed_age_us = DurationUs{10'000'000};
    cfg.freshness.intent.require_monotonic_sample_id = true;
    RobotRuntime runtime(std::move(bridge), std::move(estimator), nullptr, cfg);

    if (!expect(runtime.init(), "init should succeed (mixed intent publisher sample-id case)")) {
        return false;
    }

    scripted->enqueue(makeEstimatorSample(800, now_us()));
    runtime.setMotionIntent(makeIntentSample(800, now_us()));
    runControlStep(runtime);
    if (!expect(runtime.getStatus().active_mode == RobotMode::WALK,
                "baseline explicit sample-id intent should allow WALK")) {
        return false;
    }

    MotionIntent auto_stamped{};
    auto_stamped.requested_mode = RobotMode::WALK;
    auto_stamped.gait = GaitType::TRIPOD;
    auto_stamped.speed_mps = LinearRateMps{0.08};
    auto_stamped.heading_rad = AngleRad{0.0};
    auto_stamped.body_pose_setpoint.body_trans_m = Vec3{0.0, 0.0, 0.20};
    auto_stamped.sample_id = 0;
    auto_stamped.timestamp_us = now_us();

    scripted->enqueue(makeEstimatorSample(801, now_us()));
    runtime.setMotionIntent(auto_stamped);
    runControlStep(runtime);

    const ControlStatus post_mix = runtime.getStatus();
    return expect(post_mix.active_mode == RobotMode::WALK,
                  "auto-stamped intent after high explicit sample-id should remain monotonic and keep WALK") &&
           expect(post_mix.active_fault == FaultCode::NONE,
                  "mixed intent publishers should not trigger freshness COMMAND_TIMEOUT");
}

bool runWalkingFreshnessRejectRecoveryCase() {
    auto bridge = std::make_unique<SimHardwareBridge>();
    auto estimator = std::make_unique<ScriptedEstimator>();
    auto* scripted = estimator.get();
    auto telemetry = std::make_unique<CountingTelemetryPublisher>();
    auto* telemetry_raw = telemetry.get();

    control_config::ControlConfig cfg{};
    cfg.telemetry.enabled = true;
    cfg.telemetry.publish_period = std::chrono::milliseconds{0};
    cfg.freshness.estimator.max_allowed_age_us = DurationUs{10'000'000};
    cfg.freshness.intent.max_allowed_age_us = DurationUs{500};

    RobotRuntime runtime(std::move(bridge), std::move(estimator), nullptr, cfg, std::move(telemetry));
    if (!expect(runtime.init(), "init should succeed (walking freshness reject/recovery case)")) {
        return false;
    }
    runtime.startTelemetry();

    const TimePointUs t0 = now_us();
    scripted->enqueue(makeEstimatorSample(900, t0));
    runtime.setMotionIntent(makeWalkingIntentSample(900, t0));
    runControlStep(runtime);
    if (!expect(telemetry_raw->last_control_step.has_value(),
                "walking freshness case should capture initial walking telemetry")) {
        return false;
    }
    const auto baseline_first = telemetry_raw->last_control_step.value();
    if (!expect(baseline_first.status.active_mode == RobotMode::WALK,
                "initial walking step should run in WALK mode")) {
        return false;
    }

    scripted->enqueue(makeEstimatorSample(901, now_us()));
    runtime.setMotionIntent(makeWalkingIntentSample(901, now_us()));
    runControlStep(runtime);
    if (!expect(telemetry_raw->last_control_step.has_value(),
                "walking freshness case should capture second walking telemetry")) {
        return false;
    }
    const auto baseline_second = telemetry_raw->last_control_step.value();
    if (!expect(baseline_second.status.active_mode == RobotMode::WALK,
                "second walking step should remain in WALK mode")) {
        return false;
    }

    scripted->enqueue(makeEstimatorSample(902, now_us()));
    runtime.setMotionIntentForTest(makeWalkingIntentSample(902, TimePointUs{1}));
    runControlStep(runtime);
    if (!expect(telemetry_raw->last_control_step.has_value(),
                "walking freshness case should capture first reject telemetry")) {
        return false;
    }
    const auto reject_first = telemetry_raw->last_control_step.value();
    if (!expect(reject_first.status.active_mode == RobotMode::SAFE_IDLE,
                "strict-control reject should force SAFE_IDLE in walking context") ||
        !expect(reject_first.status.active_fault == FaultCode::COMMAND_TIMEOUT,
                "strict-control stale-intent reject should report COMMAND_TIMEOUT") ||
        !expect(jointTargetsEqual(reject_first.joint_targets, baseline_second.joint_targets),
                "first strict-control reject should hold previous walking-safe joint targets")) {
        return false;
    }

    scripted->enqueue(makeEstimatorSample(903, now_us()));
    runtime.setMotionIntentForTest(makeWalkingIntentSample(903, TimePointUs{1}));
    runControlStep(runtime);
    if (!expect(telemetry_raw->last_control_step.has_value(),
                "walking freshness case should capture second reject telemetry")) {
        return false;
    }
    const auto reject_second = telemetry_raw->last_control_step.value();
    if (!expect(reject_second.status.active_mode == RobotMode::SAFE_IDLE,
                "reject window should remain in SAFE_IDLE") ||
        !expect(jointTargetsEqual(reject_second.joint_targets, reject_first.joint_targets),
                "reject window should suppress locomotion progression by holding joint targets")) {
        return false;
    }

    scripted->enqueue(makeEstimatorSample(904, now_us()));
    runtime.setMotionIntent(makeWalkingIntentSample(904, now_us()));
    runControlStep(runtime);
    if (!expect(telemetry_raw->last_control_step.has_value(),
                "walking freshness case should capture recovery telemetry")) {
        return false;
    }
    const auto recovered_first = telemetry_raw->last_control_step.value();
    if (!expect(recovered_first.status.active_mode == RobotMode::WALK,
                "freshness recovery should resume WALK mode cleanly") ||
        !expect(recovered_first.status.active_fault == FaultCode::NONE,
                "freshness recovery should clear COMMAND_TIMEOUT fault")) {
        return false;
    }

    scripted->enqueue(makeEstimatorSample(905, now_us()));
    runtime.setMotionIntent(makeWalkingIntentSample(905, now_us()));
    runControlStep(runtime);
    if (!expect(telemetry_raw->last_control_step.has_value(),
                "walking freshness case should capture post-recovery walking telemetry")) {
        return false;
    }
    const auto recovered_second = telemetry_raw->last_control_step.value();
    return expect(recovered_second.status.active_mode == RobotMode::WALK,
                  "post-recovery walking step should stay in WALK mode") &&
           expect(recovered_second.status.active_fault == FaultCode::NONE,
                  "post-recovery walking step should keep fault cleared") &&
           expect(recovered_second.runtime_freshness.consecutive_rejects == 0,
                  "post-recovery walking step should clear consecutive freshness reject window");
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

    if (!runRuntimeFreshnessTelemetryCountersCase()) {
        return EXIT_FAILURE;
    }

    if (!runMixedIntentPublisherSampleIdCase()) {
        return EXIT_FAILURE;
    }

    if (!runWalkingFreshnessRejectRecoveryCase()) {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
