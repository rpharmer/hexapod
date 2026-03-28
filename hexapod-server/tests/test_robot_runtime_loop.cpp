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
#include <thread>
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

class StableCapturingEstimator final : public IEstimator {
public:
    RobotState update(const RobotState& raw) override {
        last_raw = raw;
        RobotState stable = stable_estimate;
        stable.sample_id = raw.sample_id;
        stable.timestamp_us = raw.timestamp_us;
        stable.valid = true;
        return stable;
    }

    RobotState stable_estimate{};
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

bool runBusFaultDuringMixedStanceSwingCase() {
    auto bridge = std::make_unique<SimHardwareBridge>();
    auto estimator = std::make_unique<StableCapturingEstimator>();
    auto* estimator_raw = estimator.get();
    estimator_raw->stable_estimate.foot_contacts = {true, true, true, true, true, true};
    for (auto& leg : estimator_raw->stable_estimate.leg_states) {
        leg.joint_state[0].pos_rad = AngleRad{0.0};
        leg.joint_state[1].pos_rad = AngleRad{-0.6};
        leg.joint_state[2].pos_rad = AngleRad{-0.8};
    }

    control_config::ControlConfig cfg{};
    cfg.freshness.estimator.max_allowed_age_us = DurationUs{10'000'000};
    cfg.freshness.intent.max_allowed_age_us = DurationUs{10'000'000};
    RobotRuntime runtime(std::move(bridge), std::move(estimator), nullptr, cfg);

    if (!expect(runtime.init(), "init should succeed (bus fault during mixed stance/swing case)")) {
        return false;
    }

    MotionIntent walk{};
    walk.requested_mode = RobotMode::WALK;
    walk.gait = GaitType::TRIPOD;
    walk.speed_mps = LinearRateMps{0.60};
    walk.heading_rad = AngleRad{0.0};
    walk.body_pose_setpoint.body_trans_m = Vec3{0.0, 0.0, 0.20};

    bool reached_mixed_stance_swing = false;
    ControlStatus mixed_status{};
    RobotState pre_fault_raw{};
    for (uint64_t sample_id = 1; sample_id <= 150; ++sample_id) {
        walk.sample_id = sample_id;
        walk.timestamp_us = now_us();
        runtime.setMotionIntentForTest(walk);
        runControlStep(runtime);
        std::this_thread::sleep_for(std::chrono::milliseconds{5});

        const ControlStatus status = runtime.getStatus();
        bool any_stance = false;
        bool any_swing = false;
        for (const bool in_stance : status.dynamic_gait.leg_in_stance) {
            any_stance = any_stance || in_stance;
            any_swing = any_swing || !in_stance;
        }

        if (any_stance && any_swing) {
            reached_mixed_stance_swing = true;
            mixed_status = status;
            pre_fault_raw = estimator_raw->last_raw;
            break;
        }
    }

    if (!expect(reached_mixed_stance_swing,
                "walking runtime should reach a mixed stance/swing gait state before fault injection")) {
        return false;
    }
    if (!expect(mixed_status.active_mode == RobotMode::WALK,
                "pre-fault mixed stance/swing step should still run in WALK mode")) {
        return false;
    }

    SimHardwareFaultToggles fault_toggles{};
    fault_toggles.drop_bus = true;
    if (!expect(runtime.setSimFaultToggles(fault_toggles),
                "runtime should support toggling simulated bus faults")) {
        return false;
    }

    walk.sample_id += 1;
    walk.timestamp_us = now_us();
    runtime.setMotionIntentForTest(walk);
    runControlStep(runtime);

    const ControlStatus post_fault = runtime.getStatus();
    if (!expect(post_fault.active_fault == FaultCode::BUS_TIMEOUT,
                "bus drop in mixed stance/swing should trigger immediate BUS_TIMEOUT safety fault") ||
        !expect(post_fault.active_mode == RobotMode::FAULT,
                "bus drop in mixed stance/swing should transition runtime mode to FAULT immediately")) {
        return false;
    }

    const bool gait_motion_inhibited =
        post_fault.dynamic_gait.suppress_stride_progression ||
        post_fault.dynamic_gait.fallback_stage != 0;
    if (!expect(gait_motion_inhibited,
                "BUS_TIMEOUT torque-cut/inhibit policy should gate gait progression immediately")) {
        return false;
    }

    const RobotState post_fault_raw = estimator_raw->last_raw;
    double max_joint_step_rad = 0.0;
    for (int leg = 0; leg < kNumLegs; ++leg) {
        for (int joint = 0; joint < kJointsPerLeg; ++joint) {
            const double before = pre_fault_raw.leg_states[leg].joint_state[joint].pos_rad.value;
            const double after = post_fault_raw.leg_states[leg].joint_state[joint].pos_rad.value;
            max_joint_step_rad = std::max(max_joint_step_rad, std::abs(after - before));
        }
    }

    return expect(max_joint_step_rad < 0.35,
                  "bus fault transition should avoid large per-step joint output transients");
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

    if (!runBusFaultDuringMixedStanceSwingCase()) {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
