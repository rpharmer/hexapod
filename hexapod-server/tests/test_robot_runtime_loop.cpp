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
#include <thread>
#include <random>
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
        geometry_publish_timestamps_us.push_back(now_us().value);
    }

    void publishControlStep(const telemetry::ControlStepTelemetry& telemetry_sample) override {
        ++control_step_count;
        control_publish_timestamps_us.push_back(telemetry_sample.timestamp_us.value);
        last_control_step = telemetry_sample;
    }

    telemetry::TelemetryPublishCounters counters() const override {
        telemetry::TelemetryPublishCounters value{};
        value.packets_sent = static_cast<uint64_t>(geometry_count + control_step_count);
        return value;
    }

    size_t geometry_count{0};
    size_t control_step_count{0};
    std::vector<uint64_t> geometry_publish_timestamps_us{};
    std::vector<uint64_t> control_publish_timestamps_us{};
    std::optional<telemetry::ControlStepTelemetry> last_control_step{};
};

class CapturingHardwareBridge final : public IHardwareBridge {
public:
    struct WriteSample {
        JointTargets targets{};
        TimePointUs timestamp_us{};
    };

    bool init() override {
        initialized_ = true;
        return true;
    }

    bool read(RobotState& out) override {
        if (!initialized_) {
            return false;
        }
        out = state_;
        out.bus_ok = true;
        out.timestamp_us = now_us();
        return true;
    }

    bool write(const JointTargets& in) override {
        if (!initialized_) {
            return false;
        }
        writes.push_back(WriteSample{in, now_us()});
        return true;
    }

    std::vector<WriteSample> writes{};

private:
    bool initialized_{false};
    RobotState state_{};
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

bool runTelemetryRapidLoopCadenceCase() {
    auto bridge = std::make_unique<SimHardwareBridge>();
    auto estimator = std::make_unique<ScriptedEstimator>();
    auto* scripted = estimator.get();
    auto telemetry = std::make_unique<CountingTelemetryPublisher>();
    auto* telemetry_raw = telemetry.get();

    control_config::ControlConfig cfg{};
    cfg.telemetry.enabled = true;
    cfg.telemetry.publish_period = std::chrono::milliseconds{25};
    cfg.telemetry.geometry_refresh_period = std::chrono::milliseconds{80};
    cfg.freshness.estimator.max_allowed_age_us = DurationUs{10'000'000};
    cfg.freshness.intent.max_allowed_age_us = DurationUs{10'000'000};
    RobotRuntime runtime(std::move(bridge), std::move(estimator), nullptr, cfg, std::move(telemetry));

    if (!expect(runtime.init(), "init should succeed (telemetry rapid loop cadence)")) {
        return false;
    }

    runtime.startTelemetry();
    if (!expect(telemetry_raw->geometry_count == 1,
                "rapid loop cadence should publish geometry on startTelemetry")) {
        return false;
    }

    const uint64_t publish_period_us = static_cast<uint64_t>(cfg.telemetry.publish_period.count()) * 1000ULL;
    const uint64_t geometry_period_us =
        static_cast<uint64_t>(cfg.telemetry.geometry_refresh_period.count()) * 1000ULL;

    const auto rapid_window = std::chrono::milliseconds{240};
    const auto start = std::chrono::steady_clock::now();
    uint64_t sample_id = 900;
    while (std::chrono::steady_clock::now() - start < rapid_window) {
        scripted->enqueue(makeEstimatorSample(sample_id, now_us()));
        runtime.setMotionIntent(makeIntentSample(sample_id, now_us()));
        runControlStep(runtime);
        ++sample_id;
        std::this_thread::sleep_for(std::chrono::milliseconds{1});
    }

    if (!expect(telemetry_raw->control_step_count > 0,
                "rapid loop cadence should publish control telemetry during loop window")) {
        return false;
    }

    const auto& control_timestamps = telemetry_raw->control_publish_timestamps_us;
    if (!expect(!control_timestamps.empty(),
                "rapid loop cadence should capture control publish timestamps")) {
        return false;
    }
    for (size_t i = 1; i < control_timestamps.size(); ++i) {
        const uint64_t delta_us = control_timestamps[i] - control_timestamps[i - 1];
        if (!expect(delta_us >= publish_period_us,
                    "publish period throttling should prevent sub-period control telemetry bursts")) {
            return false;
        }
    }

    const uint64_t elapsed_control_window_us =
        control_timestamps.back() - control_timestamps.front() + publish_period_us;
    const uint64_t theoretical_max_publishes =
        (elapsed_control_window_us / publish_period_us) + 1;
    if (!expect(telemetry_raw->control_step_count <= theoretical_max_publishes,
                "rapid loop cadence should never over-publish beyond period budget")) {
        return false;
    }

    if (!expect(telemetry_raw->geometry_count >= 3,
                "geometry refresh should publish periodically during sustained rapid loops")) {
        return false;
    }

    const auto& geometry_timestamps = telemetry_raw->geometry_publish_timestamps_us;
    if (!expect(geometry_timestamps.size() == telemetry_raw->geometry_count,
                "rapid loop cadence should capture all geometry publish timestamps")) {
        return false;
    }
    for (size_t i = 1; i < geometry_timestamps.size(); ++i) {
        const uint64_t delta_us = geometry_timestamps[i] - geometry_timestamps[i - 1];
        if (!expect(delta_us >= geometry_period_us,
                    "geometry refresh interval should be honored")) {
            return false;
        }
    }

    return true;
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

bool runSimFaultToggleLifecycleCase(RobotMode injection_mode) {
    const std::string mode_label = injection_mode == RobotMode::WALK ? "WALK" : "STAND";

    struct FaultToggleCase {
        const char* name;
        SimHardwareFaultToggles toggles;
        FaultCode expected_fault;
    };

    const std::array<FaultToggleCase, 3> cases{{
        {"drop_bus", SimHardwareFaultToggles{.drop_bus = true}, FaultCode::BUS_TIMEOUT},
        {"low_voltage", SimHardwareFaultToggles{.low_voltage = true}, FaultCode::MOTOR_FAULT},
        {"forced_unstable_contacts",
         SimHardwareFaultToggles{.forced_contacts = std::array<bool, kNumLegs>{true, false, true, false, false, false}},
         FaultCode::TIP_OVER},
    }};

    for (const auto& fault_case : cases) {
        auto bridge = std::make_unique<SimHardwareBridge>();
        auto estimator = std::make_unique<CapturingEstimator>();
        auto* estimator_raw = estimator.get();

        control_config::ControlConfig cfg{};
        cfg.freshness.estimator.max_allowed_age_us = DurationUs{10'000'000};
        cfg.freshness.intent.max_allowed_age_us = DurationUs{10'000'000};
        cfg.freshness.estimator.require_nonzero_sample_id = false;
        cfg.freshness.estimator.require_monotonic_sample_id = false;
        cfg.freshness.intent.require_nonzero_sample_id = false;
        cfg.freshness.intent.require_monotonic_sample_id = false;
        RobotRuntime runtime(std::move(bridge), std::move(estimator), nullptr, cfg);

        if (!expect(runtime.init(), std::string{"init should succeed (sim fault toggles "} + mode_label + "/" +
                                     fault_case.name + ")")) {
            return false;
        }

        const auto evaluate_with = [&](const SimHardwareFaultToggles& toggles, RobotMode mode) {
            runtime.setSimFaultToggles(toggles);
            MotionIntent intent{};
            intent.requested_mode = mode;
            intent.timestamp_us = now_us();
            runtime.setMotionIntentForTest(intent);
            runControlStep(runtime);
            const ControlStatus status = runtime.getStatus();
            return std::pair{status, estimator_raw->last_raw};
        };

        const auto [baseline_status, baseline_raw] = evaluate_with(SimHardwareFaultToggles{}, injection_mode);
        if (!expect(baseline_status.active_fault == FaultCode::NONE,
                    std::string{"baseline should be fault-free before "} + fault_case.name + " in " + mode_label)) {
            return false;
        }
        if (!expect(baseline_status.active_mode == injection_mode,
                    std::string{"baseline should honor "} + mode_label + " before " + fault_case.name)) {
            return false;
        }

        const auto [fault_status, fault_raw] = evaluate_with(fault_case.toggles, injection_mode);
        if (!expect(fault_status.active_fault == fault_case.expected_fault,
                    std::string{"fault toggle should trip expected fault for "} + fault_case.name + " in " +
                        mode_label)) {
            return false;
        }
        if (!expect(fault_status.active_mode == RobotMode::FAULT,
                    std::string{"active mode should transition to FAULT for "} + fault_case.name + " in " +
                        mode_label)) {
            return false;
        }

        const auto [latched_status, latched_raw] = evaluate_with(SimHardwareFaultToggles{}, injection_mode);
        if (!expect(latched_status.active_fault == fault_case.expected_fault,
                    std::string{"fault should remain latched after clearing toggle for "} + fault_case.name +
                        " in " + mode_label)) {
            return false;
        }
        if (!expect(latched_status.active_mode == RobotMode::FAULT,
                    std::string{"latched fault should keep active mode in FAULT for "} + fault_case.name + " in " +
                        mode_label)) {
            return false;
        }

        SafetySupervisor local_supervisor{};
        MotionIntent active_mode_intent{};
        active_mode_intent.requested_mode = injection_mode;
        active_mode_intent.timestamp_us = now_us();
        MotionIntent safe_idle_intent{};
        safe_idle_intent.requested_mode = RobotMode::SAFE_IDLE;
        safe_idle_intent.timestamp_us = now_us();

        SafetyState latched_safety = local_supervisor.evaluate(
            fault_raw, fault_raw, active_mode_intent, SafetySupervisor::FreshnessInputs{true, true});
        if (!expect(latched_safety.active_fault == fault_case.expected_fault,
                    std::string{"safety should report expected active fault for "} + fault_case.name + " in " +
                        mode_label) ||
            !expect(latched_safety.fault_lifecycle == FaultLifecycle::LATCHED,
                    std::string{"safety lifecycle should be LATCHED for "} + fault_case.name + " in " + mode_label) ||
            !expect(latched_safety.inhibit_motion,
                    std::string{"fault should inhibit motion for "} + fault_case.name + " in " + mode_label) ||
            !expect(latched_safety.torque_cut,
                    std::string{"fault should request torque cut for "} + fault_case.name + " in " + mode_label)) {
            return false;
        }

        SafetyState held_latched = local_supervisor.evaluate(
            latched_raw, latched_raw, active_mode_intent, SafetySupervisor::FreshnessInputs{true, true});
        if (!expect(held_latched.active_fault == fault_case.expected_fault,
                    std::string{"safety should hold latched fault until SAFE_IDLE for "} + fault_case.name + " in " +
                        mode_label) ||
            !expect(held_latched.fault_lifecycle == FaultLifecycle::LATCHED,
                    std::string{"safety lifecycle should remain LATCHED while not in SAFE_IDLE for "} +
                        fault_case.name + " in " + mode_label) ||
            !expect(held_latched.inhibit_motion,
                    std::string{"latched safety should continue inhibiting motion for "} + fault_case.name + " in " +
                        mode_label) ||
            !expect(held_latched.torque_cut,
                    std::string{"latched safety should keep torque-cut asserted for "} + fault_case.name + " in " +
                        mode_label)) {
            return false;
        }

        SafetyState recovering = local_supervisor.evaluate(
            latched_raw, latched_raw, safe_idle_intent, SafetySupervisor::FreshnessInputs{true, true});
        if (!expect(recovering.active_fault == fault_case.expected_fault,
                    std::string{"fault should remain active while recovering for "} + fault_case.name + " in " +
                        mode_label) ||
            !expect(recovering.fault_lifecycle == FaultLifecycle::RECOVERING,
                    std::string{"safety should enter RECOVERING after SAFE_IDLE for "} + fault_case.name + " in " +
                        mode_label)) {
            return false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds{550});
        const SafetyState cleared = local_supervisor.evaluate(
            latched_raw, latched_raw, safe_idle_intent, SafetySupervisor::FreshnessInputs{true, true});
        if (!expect(cleared.active_fault == FaultCode::NONE,
                    std::string{"fault should clear after recovery hold for "} + fault_case.name + " in " +
                        mode_label) ||
            !expect(cleared.fault_lifecycle == FaultLifecycle::ACTIVE,
                    std::string{"fault lifecycle should return ACTIVE after recovery for "} + fault_case.name +
                        " in " + mode_label) ||
            !expect(!cleared.torque_cut,
                    std::string{"torque-cut should clear after recovery for "} + fault_case.name + " in " +
                        mode_label)) {
            return false;
        }
    }

    return true;
}

bool jointTargetsEqual(const JointTargets& lhs, const JointTargets& rhs) {
    for (int leg = 0; leg < kNumLegs; ++leg) {
        for (int joint = 0; joint < kJointsPerLeg; ++joint) {
            if (lhs.leg_states[leg].joint_state[joint].pos_rad.value !=
                rhs.leg_states[leg].joint_state[joint].pos_rad.value) {
bool runLoopInterleavingJitterStressCase() {
    auto bridge = std::make_unique<SimHardwareBridge>();
    auto estimator = std::make_unique<ScriptedEstimator>();
    auto* scripted = estimator.get();

    control_config::ControlConfig cfg{};
    cfg.freshness.estimator.max_allowed_age_us = DurationUs{2'000};
    cfg.freshness.intent.max_allowed_age_us = DurationUs{2'000};
    cfg.freshness.estimator.require_timestamp = true;
    cfg.freshness.intent.require_timestamp = true;
    RobotRuntime runtime(std::move(bridge), std::move(estimator), nullptr, cfg);

    if (!expect(runtime.init(), "init should succeed (loop interleaving jitter stress case)")) {
        return false;
    }

    std::mt19937 rng(0xC0FFEEu);
    std::uniform_int_distribution<int> jitter_us_dist(0, 150);
    std::uniform_int_distribution<int> interleave_dist(0, 5);

    uint64_t est_sample_id = 1'000;
    uint64_t intent_sample_id = 1'000;
    bool observed_reject = false;
    bool observed_walk = false;

    for (int cycle = 0; cycle < 180; ++cycle) {
        const bool stale_estimator = (cycle % 11) == 0;
        const bool stale_intent = (cycle % 13) == 0;

        RobotState estimator_sample = makeEstimatorSample(++est_sample_id, now_us());
        if (stale_estimator) {
            estimator_sample.timestamp_us = TimePointUs{1};
        }
        scripted->enqueue(estimator_sample);

        MotionIntent intent = makeIntentSample(++intent_sample_id, now_us());
        if (stale_intent) {
            intent.timestamp_us = TimePointUs{1};
        }
        runtime.setMotionIntentForTest(intent);

        switch (interleave_dist(rng)) {
        case 0:
            runtime.busStep();
            std::this_thread::sleep_for(std::chrono::microseconds(jitter_us_dist(rng)));
            runtime.estimatorStep();
            std::this_thread::sleep_for(std::chrono::microseconds(jitter_us_dist(rng)));
            runtime.safetyStep();
            runtime.controlStep();
            break;
        case 1:
            runtime.busStep();
            runtime.safetyStep();
            std::this_thread::sleep_for(std::chrono::microseconds(jitter_us_dist(rng)));
            runtime.estimatorStep();
            runtime.controlStep();
            break;
        case 2:
            runtime.estimatorStep();
            runtime.busStep();
            runtime.safetyStep();
            runtime.controlStep();
            break;
        case 3:
            runtime.busStep();
            runtime.estimatorStep();
            runtime.controlStep();
            runtime.safetyStep();
            break;
        case 4:
            runtime.busStep();
            runtime.estimatorStep();
            runtime.safetyStep();
            runtime.busStep();
            runtime.controlStep();
            break;
        default:
            runtime.safetyStep();
            runtime.busStep();
            runtime.estimatorStep();
            runtime.safetyStep();
            runtime.controlStep();
            break;
        }

        const ControlStatus status = runtime.getStatus();
        if (status.active_mode == RobotMode::WALK) {
            observed_walk = true;
        }
        if (status.active_fault == FaultCode::COMMAND_TIMEOUT ||
            status.active_fault == FaultCode::ESTIMATOR_INVALID) {
            observed_reject = true;
        }

        if (!expect(status.active_fault == FaultCode::NONE || status.active_mode == RobotMode::SAFE_IDLE,
                    "faulted status should always force SAFE_IDLE under interleaving jitter")) {
            return false;
        }

        if (status.active_fault == FaultCode::ESTIMATOR_INVALID) {
            if (!expect(!status.estimator_valid,
                        "ESTIMATOR_INVALID should always set estimator_valid=false")) {
                return false;
            }
        }

        if (status.active_fault == FaultCode::COMMAND_TIMEOUT) {
            if (!expect(status.estimator_valid,
                        "COMMAND_TIMEOUT should preserve estimator_valid=true when only intent is stale")) {
                return false;
            }
        }

        if (status.active_mode == RobotMode::WALK) {
            if (!expect(status.active_fault == FaultCode::NONE,
                        "WALK mode should never report a fault under interleaving jitter")) {
                return false;
            }
            if (!expect(status.estimator_valid,
                        "WALK mode should require estimator_valid=true")) {
                return false;
            }
        }
    }
    return true;
}

bool runFreshnessRejectFallbackAndCountersCase() {
    auto bridge = std::make_unique<SimHardwareBridge>();
    auto estimator = std::make_unique<ScriptedEstimator>();
    auto* scripted = estimator.get();
    auto telemetry = std::make_unique<CountingTelemetryPublisher>();
    auto* telemetry_raw = telemetry.get();

    control_config::ControlConfig cfg{};
    cfg.telemetry.enabled = true;
    cfg.telemetry.publish_period = std::chrono::milliseconds{0};
    cfg.freshness.estimator.max_allowed_age_us = DurationUs{10'000'000};
    cfg.freshness.intent.max_allowed_age_us = DurationUs{10'000'000};
    cfg.freshness.estimator.require_monotonic_sample_id = true;
    cfg.freshness.intent.require_monotonic_sample_id = true;
    RobotRuntime runtime(std::move(bridge), std::move(estimator), nullptr, cfg, std::move(telemetry));

    if (!expect(runtime.init(), "init should succeed (freshness reject fallback counters case)")) {
        return false;
    }
    runtime.startTelemetry();

    MotionIntent nominal{};
    nominal.requested_mode = RobotMode::WALK;
    nominal.gait = GaitType::TRIPOD;
    nominal.speed_mps = LinearRateMps{0.10};
    nominal.heading_rad = AngleRad{0.0};
    nominal.body_pose_setpoint.body_trans_m = Vec3{0.0, 0.0, 0.20};
    nominal.sample_id = 900;
    nominal.timestamp_us = now_us();

    scripted->enqueue(makeEstimatorSample(900, now_us()));
    runtime.setMotionIntentForTest(nominal);
    runControlStep(runtime);

    if (!expect(telemetry_raw->last_control_step.has_value(),
                "freshness fallback case should capture baseline telemetry")) {
        return false;
    }
    const auto baseline = telemetry_raw->last_control_step.value();
    if (!expect(baseline.status.active_mode == RobotMode::WALK,
                "baseline freshness fallback case should allow WALK")) {
        return false;
    }

    MotionIntent non_monotonic_intent = nominal;
    non_monotonic_intent.sample_id = 899;
    non_monotonic_intent.timestamp_us = now_us();
    non_monotonic_intent.speed_mps = LinearRateMps{0.35};
    non_monotonic_intent.heading_rad = AngleRad{0.5};

    scripted->enqueue(makeEstimatorSample(899, now_us()));
    runtime.setMotionIntentForTest(non_monotonic_intent);
    runControlStep(runtime);

    if (!expect(telemetry_raw->last_control_step.has_value(),
                "freshness fallback case should capture rejected telemetry")) {
        return false;
    }
    const auto first_reject = telemetry_raw->last_control_step.value();
    if (!expect(first_reject.status.active_mode == RobotMode::SAFE_IDLE,
                "non-monotonic samples should force SAFE_IDLE") ||
        !expect(first_reject.status.active_fault == FaultCode::ESTIMATOR_INVALID,
                "non-monotonic estimator sample should set ESTIMATOR_INVALID fault") ||
        !expect(first_reject.status.estimator_valid == false,
                "non-monotonic estimator sample should mark estimator invalid") ||
        !expect(jointTargetsEqual(first_reject.joint_targets, baseline.joint_targets),
                "reject path should keep safe fallback joint targets and skip pipeline output overwrite") ||
        !expect(first_reject.runtime_freshness.total_rejects == 1,
                "first reject should increment total freshness reject counter") ||
        !expect(first_reject.runtime_freshness.consecutive_rejects == 1,
                "first reject should increment consecutive freshness reject counter") ||
        !expect(first_reject.runtime_freshness.non_monotonic_id_rejects == 1,
                "first reject should increment non-monotonic freshness reject counter") ||
        !expect(first_reject.runtime_freshness.invalid_sample_id_rejects == 0,
                "non-monotonic case should not increment invalid-sample freshness reject counter")) {
        return false;
    }

    MotionIntent recovered = nominal;
    recovered.sample_id = 901;
    recovered.timestamp_us = now_us();
    scripted->enqueue(makeEstimatorSample(901, now_us()));
    runtime.setMotionIntentForTest(recovered);
    runControlStep(runtime);

    if (!expect(telemetry_raw->last_control_step.has_value(),
                "freshness fallback case should capture recovery telemetry")) {
        return false;
    }
    const auto recovered_step = telemetry_raw->last_control_step.value();
    return expect(recovered_step.status.loop_counter > first_reject.status.loop_counter,
                  "recovery step should continue incrementing runtime status loop counter") &&
           expect(recovered_step.runtime_freshness.total_rejects == 1,
                  "recovery should keep freshness total rejects cumulative") &&
           expect(recovered_step.runtime_freshness.consecutive_rejects == 0,
                  "recovery should reset consecutive freshness rejects");
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
    const auto baseline_second = telemetry_raw->last_control_step.value();

    scripted->enqueue(makeEstimatorSample(902, now_us()));
    runtime.setMotionIntentForTest(makeWalkingIntentSample(902, TimePointUs{1}));
    runControlStep(runtime);
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
    const auto recovered_second = telemetry_raw->last_control_step.value();
    return expect(recovered_second.status.active_mode == RobotMode::WALK,
                  "post-recovery walking step should stay in WALK mode") &&
           expect(recovered_second.status.active_fault == FaultCode::NONE,
                  "post-recovery walking step should keep fault cleared") &&
           expect(recovered_second.runtime_freshness.consecutive_rejects == 0,
                  "post-recovery walking step should clear consecutive freshness reject window");
}

bool runJointTargetWriteSlewLimitCase() {
    auto bridge = std::make_unique<CapturingHardwareBridge>();
    auto* bridge_raw = bridge.get();
    auto estimator = std::make_unique<ScriptedEstimator>();
    RobotRuntime runtime(std::move(bridge), std::move(estimator), nullptr, control_config::ControlConfig{});

    if (!expect(runtime.init(), "init should succeed (joint target write slew case)")) {
        return false;
    }

    JointTargets step_positive{};
    JointTargets step_negative{};
    for (int leg = 0; leg < kNumLegs; ++leg) {
        for (int joint = 0; joint < kJointsPerLeg; ++joint) {
            step_positive.leg_states[leg].joint_state[joint].pos_rad = AngleRad{3.0};
            step_negative.leg_states[leg].joint_state[joint].pos_rad = AngleRad{-3.0};
        }
    }

    runtime.setJointTargetsForTest(step_positive);
    runtime.busStep();
    std::this_thread::sleep_for(std::chrono::milliseconds{20});

    runtime.setJointTargetsForTest(step_negative);
    runtime.busStep();
    std::this_thread::sleep_for(std::chrono::milliseconds{20});

    runtime.setJointTargetsForTest(step_positive);
    runtime.busStep();

    if (!expect(bridge_raw->writes.size() >= 3,
                "slew case should record at least three hardware writes")) {
        return false;
    }

    constexpr double kJointTargetSlewLimitRadPerSec = 18.0;
    constexpr double kSlackRad = 1e-6;
    for (size_t idx = 1; idx < bridge_raw->writes.size(); ++idx) {
        const auto& prev = bridge_raw->writes[idx - 1];
        const auto& current = bridge_raw->writes[idx];
        if (current.timestamp_us.value <= prev.timestamp_us.value) {
            return expect(false, "hardware write timestamps should be monotonic");
        }
        const double dt_s =
            static_cast<double>(current.timestamp_us.value - prev.timestamp_us.value) / 1'000'000.0;
        const double max_delta = (kJointTargetSlewLimitRadPerSec * dt_s) + kSlackRad;
        for (int leg = 0; leg < kNumLegs; ++leg) {
            for (int joint = 0; joint < kJointsPerLeg; ++joint) {
                const double previous = prev.targets.leg_states[leg].joint_state[joint].pos_rad.value;
                const double observed = current.targets.leg_states[leg].joint_state[joint].pos_rad.value;
                const double observed_delta = std::abs(observed - previous);
                if (!expect(observed_delta <= max_delta,
                            "per-cycle hw write delta should be bounded by joint slew limit")) {
                    return false;
                }
            }
        }
    }

    return true;

    return expect(observed_walk, "jitter stress should include accepted WALK outputs") &&
           expect(observed_reject, "jitter stress should include freshness rejects");
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

    if (!runTelemetryRapidLoopCadenceCase()) {
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

    if (!runSimFaultToggleLifecycleCase(RobotMode::WALK)) {
        return EXIT_FAILURE;
    }

    if (!runSimFaultToggleLifecycleCase(RobotMode::STAND)) {
        return EXIT_FAILURE;
    }

    if (!runFreshnessRejectFallbackAndCountersCase()) {
        return EXIT_FAILURE;
    }
    if (!runWalkingFreshnessRejectRecoveryCase()) {
        return EXIT_FAILURE;
    }

    if (!runJointTargetWriteSlewLimitCase()) {
    if (!runLoopInterleavingJitterStressCase()) {
    if (!runBusFaultDuringMixedStanceSwingCase()) {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
