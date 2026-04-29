#include "safety_supervisor.hpp"

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <thread>

namespace {

bool expect(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

RobotState nominalRaw() {
    RobotState raw{};
    raw.timestamp_us = now_us();
    raw.bus_ok = true;
    raw.has_power_state = true;
    raw.voltage = 12.0f;
    raw.current = 1.0f;
    raw.foot_contacts = {true, true, true, true, true, true};
    return raw;
}

RobotState nominalEstimated() {
    RobotState est{};
    est.timestamp_us = now_us();
    return est;
}

MotionIntent intentNow(RobotMode mode) {
    MotionIntent intent{};
    intent.requested_mode = mode;
    intent.timestamp_us = now_us();
    intent.twist.body_trans_m.z = 0.14;
    return intent;
}

MotionIntent staleIntent(RobotMode mode) {
    MotionIntent intent{};
    intent.requested_mode = mode;
    intent.timestamp_us = TimePointUs{};
    intent.twist.body_trans_m.z = 0.14;
    return intent;
}

control_config::SafetyConfig collapseSafetyConfig() {
    control_config::SafetyConfig cfg{};
    cfg.max_tilt_rad = AngleRad{0.55};
    cfg.body_height_collapse_margin_m = 0.0;
    cfg.body_height_collapse_min_safe_m = 0.10;
    cfg.body_height_collapse_max_contacts = 3;
    return cfg;
}

control_config::SafetyConfig rapidMotionSafetyConfig() {
    control_config::SafetyConfig cfg{};
    cfg.max_tilt_rad = AngleRad{0.55};
    cfg.rapid_body_rate_radps = 1.0;
    return cfg;
}

bool testHigherPriorityFaultWins() {
    SafetySupervisor supervisor;
    RobotState raw = nominalRaw();
    RobotState est = nominalEstimated();

    raw.bus_ok = false;
    est.body_twist_state.twist_pos_rad.x = 2.0; // clearly above default tilt limit

    const SafetyState state = supervisor.evaluate(
        raw, est, intentNow(RobotMode::WALK), SafetySupervisor::FreshnessInputs{true, true});
    return expect(state.active_fault == FaultCode::TIP_OVER,
                  "simultaneous bus + tip-over should keep higher-priority TIP_OVER") &&
           expect(state.torque_cut, "tip-over should request torque cut");
}

bool testRulePrecedenceAcrossAllFaultTriggers() {
    SafetySupervisor supervisor;
    RobotState raw = nominalRaw();
    RobotState est = nominalEstimated();

    raw.bus_ok = false;
    raw.voltage = 1.0f;
    raw.current = 999.0f;
    raw.foot_contacts = {false, false, false, false, false, false};
    est.body_twist_state.twist_pos_rad.x = 2.0;

    const SafetyState state = supervisor.evaluate(
        raw, est, intentNow(RobotMode::WALK), SafetySupervisor::FreshnessInputs{false, false});

    return expect(state.active_fault == FaultCode::TIP_OVER,
                  "when every trigger is active, highest-precedence TIP_OVER should win") &&
           expect(state.torque_cut,
                  "highest-precedence TIP_OVER should keep torque cut enabled");
}

bool testEstimatorBeatsCommandTimeoutWithoutTorqueCut() {
    SafetySupervisor supervisor;
    RobotState raw = nominalRaw();
    RobotState est = nominalEstimated();

    raw.foot_contacts = {false, false, false, false, false, false};

    const SafetyState state = supervisor.evaluate(
        raw, est, intentNow(RobotMode::WALK), SafetySupervisor::FreshnessInputs{false, false});

    return expect(state.active_fault == FaultCode::ESTIMATOR_INVALID,
                  "ESTIMATOR_INVALID should beat lower-precedence COMMAND_TIMEOUT") &&
           expect(!state.torque_cut,
                  "estimator and command timeout faults should not request torque cut");
}

bool testBusTimeoutBeatsFreshnessFaults() {
    SafetySupervisor supervisor;
    RobotState raw = nominalRaw();
    RobotState est = nominalEstimated();

    raw.bus_ok = false;
    raw.foot_contacts = {false, false, false, false, false, false};

    const SafetyState state = supervisor.evaluate(
        raw, est, staleIntent(RobotMode::WALK), SafetySupervisor::FreshnessInputs{false, false});

    return expect(state.active_fault == FaultCode::BUS_TIMEOUT,
                  "BUS_TIMEOUT should beat estimator/intent freshness faults") &&
           expect(state.torque_cut, "BUS_TIMEOUT should keep torque_cut enabled");
}

bool testMotorFaultTorqueCut() {
    SafetySupervisor supervisor;
    RobotState raw = nominalRaw();
    RobotState est = nominalEstimated();

    raw.current = 999.0f;

    const SafetyState state = supervisor.evaluate(
        raw, est, intentNow(RobotMode::WALK), SafetySupervisor::FreshnessInputs{true, true});

    return expect(state.active_fault == FaultCode::MOTOR_FAULT,
                  "over-current should produce MOTOR_FAULT") &&
           expect(state.torque_cut, "MOTOR_FAULT should request torque cut");
}

bool testBodyHeightCollapseTriggersTipOverEarly() {
    SafetySupervisor supervisor(collapseSafetyConfig());
    RobotState raw = nominalRaw();
    RobotState est = nominalEstimated();

    raw.foot_contacts = {true, true, false, false, false, false};
    est.has_body_twist_state = true;
    est.body_twist_state.body_trans_m.z = 0.08;
    est.body_twist_state.twist_pos_rad.x = 0.10;
    est.body_twist_state.twist_pos_rad.y = 0.12;

    const SafetyState state =
        supervisor.evaluate(raw, est, intentNow(RobotMode::WALK), SafetySupervisor::FreshnessInputs{true, true});

    return expect(state.active_fault == FaultCode::BODY_COLLAPSE,
                  "body-height collapse with sparse support should trip BODY_COLLAPSE early") &&
           expect(state.torque_cut, "collapse-triggered BODY_COLLAPSE should request torque cut");
}

bool testBodyHeightCollapseStaysQuietWithHealthySupport() {
    SafetySupervisor supervisor(collapseSafetyConfig());
    RobotState raw = nominalRaw();
    RobotState est = nominalEstimated();

    raw.foot_contacts = {true, true, true, true, true, true};
    est.has_body_twist_state = true;
    est.body_twist_state.body_trans_m.z = 0.08;
    est.body_twist_state.twist_pos_rad.x = 0.10;
    est.body_twist_state.twist_pos_rad.y = 0.12;

    const SafetyState state =
        supervisor.evaluate(raw, est, intentNow(RobotMode::WALK), SafetySupervisor::FreshnessInputs{true, true});

    return expect(state.active_fault == FaultCode::NONE,
                  "healthy support should not trip the body-height collapse detector");
}

bool testRapidBodyRateTriggersTipOverEarly() {
    SafetySupervisor supervisor(rapidMotionSafetyConfig());
    RobotState raw = nominalRaw();
    RobotState est = nominalEstimated();
    MotionIntent intent = intentNow(RobotMode::WALK);
    intent.cmd_vx_mps = LinearRateMps{0.30};
    intent.speed_mps = LinearRateMps{0.30};

    raw.foot_contacts = {true, true, false, false, false, false};
    est.has_body_twist_state = true;
    est.body_twist_state.body_trans_mps.x = 0.30;
    est.body_twist_state.body_trans_mps.y = 0.0;
    est.has_imu = true;
    est.imu.valid = true;
    est.imu.gyro_radps.x = 0.95;
    est.imu.gyro_radps.y = 0.35;
    est.body_twist_state.body_trans_m.z = 0.12;
    est.body_twist_state.twist_pos_rad.x = 0.16;
    est.body_twist_state.twist_pos_rad.y = 0.18;

    const SafetyState state =
        supervisor.evaluate(raw, est, intent, SafetySupervisor::FreshnessInputs{true, true});

    return expect(state.active_fault == FaultCode::TIP_OVER,
                  "sparse support with high body-rate should trip TIP_OVER early") &&
           expect(state.torque_cut, "rate-triggered TIP_OVER should request torque cut");
}

bool testRapidBodyRateStaysQuietWithHealthySupport() {
    SafetySupervisor supervisor(rapidMotionSafetyConfig());
    RobotState raw = nominalRaw();
    RobotState est = nominalEstimated();
    MotionIntent intent = intentNow(RobotMode::WALK);
    intent.cmd_vx_mps = LinearRateMps{0.06};
    intent.speed_mps = LinearRateMps{0.06};

    raw.foot_contacts = {true, true, true, true, true, true};
    est.has_body_twist_state = true;
    est.body_twist_state.body_trans_mps.x = 0.06;
    est.body_twist_state.body_trans_mps.y = 0.0;
    est.has_imu = true;
    est.imu.valid = true;
    est.imu.gyro_radps.x = 0.95;
    est.imu.gyro_radps.y = 0.35;
    est.body_twist_state.body_trans_m.z = 0.12;
    est.body_twist_state.twist_pos_rad.x = 0.16;
    est.body_twist_state.twist_pos_rad.y = 0.18;

    const SafetyState state =
        supervisor.evaluate(raw, est, intent, SafetySupervisor::FreshnessInputs{true, true});

    return expect(state.active_fault == FaultCode::NONE,
                  "healthy support should not trip the rapid body-rate detector");
}

bool testLatchedRemainsWhenIntentNotSafeIdle() {
    SafetySupervisor supervisor;
    RobotState raw = nominalRaw();
    RobotState est = nominalEstimated();

    raw.bus_ok = false;
    const SafetyState latched = supervisor.evaluate(
        raw, est, intentNow(RobotMode::WALK), SafetySupervisor::FreshnessInputs{true, true});
    if (!expect(latched.active_fault == FaultCode::BUS_TIMEOUT, "bus fault should latch BUS_TIMEOUT")) {
        return false;
    }

    raw.bus_ok = true;
    const SafetyState held = supervisor.evaluate(
        raw, est, intentNow(RobotMode::WALK), SafetySupervisor::FreshnessInputs{true, true});
    return expect(held.active_fault == FaultCode::BUS_TIMEOUT,
                  "fault should remain BUS_TIMEOUT after hardware clears") &&
           expect(held.fault_lifecycle == FaultLifecycle::LATCHED,
                  "lifecycle should stay LATCHED when intent mode is not SAFE_IDLE");
}

bool testLatchedRemainsWhenIntentStale() {
    SafetySupervisor supervisor;
    RobotState raw = nominalRaw();
    RobotState est = nominalEstimated();

    raw.bus_ok = false;
    const SafetyState latched = supervisor.evaluate(
        raw, est, intentNow(RobotMode::WALK), SafetySupervisor::FreshnessInputs{true, true});
    if (!expect(latched.active_fault == FaultCode::BUS_TIMEOUT, "bus fault should latch before stale-intent check")) {
        return false;
    }

    raw.bus_ok = true;
    const SafetyState held = supervisor.evaluate(
        raw, est, staleIntent(RobotMode::SAFE_IDLE), SafetySupervisor::FreshnessInputs{true, false});
    return expect(held.active_fault == FaultCode::BUS_TIMEOUT,
                  "fault should remain BUS_TIMEOUT with stale SAFE_IDLE intent") &&
           expect(held.fault_lifecycle == FaultLifecycle::LATCHED,
                  "lifecycle should stay LATCHED when intent timestamp is stale");
}

bool testRecoveryRequiresBothConditionsAndHoldTime() {
    SafetySupervisor supervisor;
    RobotState raw = nominalRaw();
    RobotState est = nominalEstimated();

    raw.bus_ok = false;
    const SafetyState latched = supervisor.evaluate(
        raw, est, intentNow(RobotMode::WALK), SafetySupervisor::FreshnessInputs{true, true});
    if (!expect(latched.active_fault == FaultCode::BUS_TIMEOUT, "bus fault should latch for recovery test")) {
        return false;
    }

    raw.bus_ok = true;

    const SafetyState not_safe_idle = supervisor.evaluate(
        raw, est, intentNow(RobotMode::WALK), SafetySupervisor::FreshnessInputs{true, true});
    if (!expect(not_safe_idle.fault_lifecycle == FaultLifecycle::LATCHED,
                "recovery must not start until mode is SAFE_IDLE")) {
        return false;
    }

    const SafetyState stale_safe_idle = supervisor.evaluate(
        raw, est, staleIntent(RobotMode::SAFE_IDLE), SafetySupervisor::FreshnessInputs{true, false});
    if (!expect(stale_safe_idle.fault_lifecycle == FaultLifecycle::LATCHED,
                "recovery must not start with stale SAFE_IDLE intent")) {
        return false;
    }

    const SafetyState recovering = supervisor.evaluate(
        raw, est, intentNow(RobotMode::SAFE_IDLE), SafetySupervisor::FreshnessInputs{true, true});
    if (!expect(recovering.fault_lifecycle == FaultLifecycle::RECOVERING,
                "recovery should start only with SAFE_IDLE + fresh intent")) {
        return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(550));
    const SafetyState cleared = supervisor.evaluate(
        raw, est, intentNow(RobotMode::SAFE_IDLE), SafetySupervisor::FreshnessInputs{true, true});
    return expect(cleared.active_fault == FaultCode::NONE,
                  "fault should clear after recovery hold time") &&
           expect(cleared.fault_lifecycle == FaultLifecycle::ACTIVE,
                  "lifecycle should return ACTIVE after clear");
}

} // namespace

int main() {
    if (!testHigherPriorityFaultWins() ||
        !testRulePrecedenceAcrossAllFaultTriggers() ||
        !testEstimatorBeatsCommandTimeoutWithoutTorqueCut() ||
        !testBusTimeoutBeatsFreshnessFaults() ||
        !testMotorFaultTorqueCut() ||
        !testBodyHeightCollapseTriggersTipOverEarly() ||
        !testBodyHeightCollapseStaysQuietWithHealthySupport() ||
        !testLatchedRemainsWhenIntentNotSafeIdle() ||
        !testLatchedRemainsWhenIntentStale() ||
        !testRecoveryRequiresBothConditionsAndHoldTime()) {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
