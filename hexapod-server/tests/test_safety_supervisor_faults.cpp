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
    return intent;
}

MotionIntent staleIntent(RobotMode mode) {
    MotionIntent intent{};
    intent.requested_mode = mode;
    intent.timestamp_us = TimePointUs{};
    return intent;
}

bool testHigherPriorityFaultWins() {
    SafetySupervisor supervisor;
    RobotState raw = nominalRaw();
    RobotState est = nominalEstimated();

    raw.bus_ok = false;
    est.body_pose_state.orientation_rad.x = 2.0; // clearly above default tilt limit
    est.has_measured_body_pose_state = true;
    est.has_body_pose_state = true;

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
    est.body_pose_state.orientation_rad.x = 2.0;
    est.has_measured_body_pose_state = true;
    est.has_body_pose_state = true;

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




bool testInferredTiltDoesNotTripMeasuredTiltPolicy() {
    SafetySupervisor supervisor;
    RobotState raw = nominalRaw();
    RobotState est = nominalEstimated();

    est.body_pose_state.orientation_rad.x = 2.0;
    est.has_inferred_body_pose_state = true;
    est.has_body_pose_state = true;

    const SafetyState state = supervisor.evaluate(
        raw, est, intentNow(RobotMode::WALK), SafetySupervisor::FreshnessInputs{true, true});

    return expect(state.active_fault != FaultCode::TIP_OVER,
                  "inferred-only tilt should not trigger measured-only tilt policy") &&
           expect(state.active_fault == FaultCode::NONE,
                  "without other triggers, inferred-only tilt should keep safety clear");
}

bool testUnstableSupportTriggersTipOver() {
    SafetySupervisor supervisor;
    RobotState raw = nominalRaw();
    RobotState est = nominalEstimated();

    raw.foot_contacts = {true, false, true, false, false, false};

    const SafetyState state = supervisor.evaluate(
        raw, est, intentNow(RobotMode::WALK), SafetySupervisor::FreshnessInputs{true, true});

    return expect(!state.stable, "stability tracking should report unstable when support polygon is invalid") &&
           expect(state.support_contact_count == 2, "safety state should track active support contacts") &&
           expect(state.active_fault == FaultCode::TIP_OVER,
                  "unstable support should trigger TIP_OVER safety fault") &&
           expect(state.torque_cut, "TIP_OVER from instability should request torque cut");
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

bool testJointDiscontinuityTripsSafety() {
    SafetySupervisor supervisor;
    RobotState raw = nominalRaw();
    RobotState est = nominalEstimated();

    est.sample_id = 1;
    est.leg_states[0].joint_state[0].pos_rad = AngleRad{0.0};
    const SafetyState baseline = supervisor.evaluate(
        raw, est, intentNow(RobotMode::WALK), SafetySupervisor::FreshnessInputs{true, true});
    if (!expect(baseline.active_fault == FaultCode::NONE, "baseline sample should not trigger discontinuity fault")) {
        return false;
    }

    est.sample_id = 2;
    est.leg_states[0].joint_state[0].pos_rad = AngleRad{2.0};
    const SafetyState jumped = supervisor.evaluate(
        raw, est, intentNow(RobotMode::WALK), SafetySupervisor::FreshnessInputs{true, true});

    return expect(jumped.active_fault == FaultCode::JOINT_LIMIT,
                  "large joint discontinuity should trigger JOINT_LIMIT fault") &&
           expect(jumped.torque_cut, "joint discontinuity fault should request torque cut");
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

    // kRecoveryHoldTimeUs is 500ms. Verify the fault remains RECOVERING
    // immediately before the threshold and only clears once we are past it.
    std::this_thread::sleep_for(std::chrono::milliseconds(490));
    const SafetyState before_threshold = supervisor.evaluate(
        raw, est, intentNow(RobotMode::SAFE_IDLE), SafetySupervisor::FreshnessInputs{true, true});
    if (!expect(before_threshold.active_fault == FaultCode::BUS_TIMEOUT,
                "fault should remain latched just below recovery hold threshold") ||
        !expect(before_threshold.fault_lifecycle == FaultLifecycle::RECOVERING,
                "lifecycle should remain RECOVERING just below recovery hold threshold")) {
        return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    const SafetyState after_threshold = supervisor.evaluate(
        raw, est, intentNow(RobotMode::SAFE_IDLE), SafetySupervisor::FreshnessInputs{true, true});
    return expect(after_threshold.active_fault == FaultCode::NONE,
                  "fault should clear after recovery hold time") &&
           expect(after_threshold.fault_lifecycle == FaultLifecycle::ACTIVE,
                  "lifecycle should return ACTIVE after clear");
}

} // namespace

int main() {
    if (!testHigherPriorityFaultWins() ||
        !testRulePrecedenceAcrossAllFaultTriggers() ||
        !testEstimatorBeatsCommandTimeoutWithoutTorqueCut() ||
        !testBusTimeoutBeatsFreshnessFaults() ||
        !testInferredTiltDoesNotTripMeasuredTiltPolicy() ||
        !testUnstableSupportTriggersTipOver() ||
        !testMotorFaultTorqueCut() ||
        !testJointDiscontinuityTripsSafety() ||
        !testLatchedRemainsWhenIntentNotSafeIdle() ||
        !testLatchedRemainsWhenIntentStale() ||
        !testRecoveryRequiresBothConditionsAndHoldTime()) {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
