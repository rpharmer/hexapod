#include "safety_supervisor.hpp"

#include <array>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string_view>
#include <thread>
#include <vector>

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

bool testTiltViolationBeatsBusTimeoutInHardFaultContext() {
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
                  "bus fault + measured tilt violation should immediately select TIP_OVER by priority") &&
           expect(state.torque_cut,
                  "TIP_OVER selected by priority in bus-fault context should keep torque cut enabled");
}

bool testTiltViolationBeatsMotorFaultWhenVoltageLow() {
    SafetySupervisor supervisor;
    RobotState raw = nominalRaw();
    RobotState est = nominalEstimated();

    raw.voltage = 1.0f; // below min_bus_voltage_v
    est.body_pose_state.orientation_rad.x = 2.0; // clearly above default tilt limit
    est.has_measured_body_pose_state = true;
    est.has_body_pose_state = true;

    const SafetyState state = supervisor.evaluate(
        raw, est, intentNow(RobotMode::WALK), SafetySupervisor::FreshnessInputs{true, true});
    return expect(state.active_fault == FaultCode::TIP_OVER,
                  "low voltage + measured tilt violation should immediately select TIP_OVER by priority") &&
           expect(state.torque_cut,
                  "TIP_OVER selected by priority in low-voltage context should keep torque cut enabled");
}

bool testTiltViolationBeatsMotorFaultWhenCurrentHigh() {
    SafetySupervisor supervisor;
    RobotState raw = nominalRaw();
    RobotState est = nominalEstimated();

    raw.current = 999.0f; // above max_bus_current_a
    est.body_pose_state.orientation_rad.x = 2.0; // clearly above default tilt limit
    est.has_measured_body_pose_state = true;
    est.has_body_pose_state = true;

    const SafetyState state = supervisor.evaluate(
        raw, est, intentNow(RobotMode::WALK), SafetySupervisor::FreshnessInputs{true, true});
    return expect(state.active_fault == FaultCode::TIP_OVER,
                  "high current + measured tilt violation should immediately select TIP_OVER by priority") &&
           expect(state.torque_cut,
                  "TIP_OVER selected by priority in high-current context should keep torque cut enabled");
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

bool testCombinationPriorityOrderIsDeterministic() {
    struct FaultTriggerCase {
        bool trigger_timeout{false};
        bool trigger_estimator_invalid{false};
        bool trigger_bus_timeout{false};
        bool trigger_motor_fault{false};
        bool trigger_tip_over{false};
        FaultCode expected_fault{FaultCode::NONE};
        const char* label{""};
    };

    const std::array<FaultTriggerCase, 6> cases{{
        FaultTriggerCase{
            .trigger_timeout = true,
            .trigger_motor_fault = true,
            .expected_fault = FaultCode::MOTOR_FAULT,
            .label = "timeout + low voltage should select MOTOR_FAULT",
        },
        FaultTriggerCase{
            .trigger_timeout = true,
            .trigger_tip_over = true,
            .expected_fault = FaultCode::TIP_OVER,
            .label = "timeout + tilt should select TIP_OVER",
        },
        FaultTriggerCase{
            .trigger_timeout = true,
            .trigger_estimator_invalid = true,
            .trigger_motor_fault = true,
            .expected_fault = FaultCode::TIP_OVER,
            .label = "timeout + estimator invalid + low voltage should select TIP_OVER",
        },
        FaultTriggerCase{
            .trigger_timeout = true,
            .trigger_estimator_invalid = true,
            .trigger_bus_timeout = true,
            .expected_fault = FaultCode::TIP_OVER,
            .label = "timeout + estimator invalid + bus timeout should select TIP_OVER",
        },
        FaultTriggerCase{
            .trigger_timeout = true,
            .trigger_motor_fault = true,
            .trigger_tip_over = true,
            .expected_fault = FaultCode::TIP_OVER,
            .label = "timeout + low voltage + tilt should select TIP_OVER",
        },
        FaultTriggerCase{
            .trigger_timeout = true,
            .trigger_estimator_invalid = true,
            .trigger_bus_timeout = true,
            .trigger_motor_fault = true,
            .trigger_tip_over = true,
            .expected_fault = FaultCode::TIP_OVER,
            .label = "all simultaneous triggers should select TIP_OVER",
        },
    }};

    for (int run = 0; run < 12; ++run) {
        for (const FaultTriggerCase& test_case : cases) {
            SafetySupervisor supervisor;
            RobotState raw = nominalRaw();
            RobotState est = nominalEstimated();
            MotionIntent intent = intentNow(RobotMode::WALK);
            SafetySupervisor::FreshnessInputs freshness{true, true};

            if (test_case.trigger_timeout) {
                freshness.intent_valid = false;
                intent = staleIntent(RobotMode::WALK);
            }
            if (test_case.trigger_estimator_invalid) {
                raw.foot_contacts = {false, false, false, false, false, false};
            }
            if (test_case.trigger_bus_timeout) {
                raw.bus_ok = false;
            }
            if (test_case.trigger_motor_fault) {
                raw.voltage = 1.0f;
            }
            if (test_case.trigger_tip_over) {
                est.body_pose_state.orientation_rad.x = 2.0;
                est.has_measured_body_pose_state = true;
                est.has_body_pose_state = true;
            }

            SafetyState state = supervisor.evaluate(raw, est, intent, freshness);
            if (test_case.trigger_tip_over &&
                !test_case.trigger_bus_timeout &&
                !test_case.trigger_motor_fault &&
                !test_case.trigger_estimator_invalid) {
                // Tilt-only detections in non-hard-fault contexts are intentionally debounced in
                // SafetySupervisor::evaluate(). Re-run a few samples so precedence assertions
                // evaluate the latched fault after debounce confirmation.
                for (int i = 0; i < 32 && state.active_fault != FaultCode::TIP_OVER; ++i) {
                    state = supervisor.evaluate(raw, est, intent, freshness);
                }
            }
            if (!expect(state.active_fault == test_case.expected_fault, test_case.label)) {
                return false;
            }
        }
    }

    return true;
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

bool testSupportInstabilityBypassesTiltDebounce() {
    SafetySupervisor supervisor;
    RobotState raw = nominalRaw();
    RobotState est = nominalEstimated();

    // Keep measured tilt below threshold so only support-polygon instability can trip TIP_OVER.
    est.body_pose_state.orientation_rad = Vec3{0.05, 0.05, 0.0};
    est.has_measured_body_pose_state = true;
    est.has_body_pose_state = true;
    raw.foot_contacts = {true, false, true, false, false, false};

    const SafetyState immediate = supervisor.evaluate(
        raw, est, intentNow(RobotMode::WALK), SafetySupervisor::FreshnessInputs{true, true});
    if (!expect(immediate.active_fault == FaultCode::TIP_OVER,
                "support-polygon TIP_OVER should trigger immediately without debounce samples")) {
        return false;
    }

    SafetySupervisor fresh_supervisor;
    const SafetyState first_tilt_sample = [&]() {
        RobotState tilt_raw = nominalRaw();
        RobotState tilt_est = nominalEstimated();
        tilt_est.body_pose_state.orientation_rad = Vec3{2.0, 0.0, 0.0};
        tilt_est.has_measured_body_pose_state = true;
        tilt_est.has_body_pose_state = true;
        return fresh_supervisor.evaluate(
            tilt_raw, tilt_est, intentNow(RobotMode::WALK), SafetySupervisor::FreshnessInputs{true, true});
    }();

    return expect(first_tilt_sample.active_fault == FaultCode::NONE,
                  "tilt-only TIP_OVER should still be debounced on first sample");
}

bool testTiltDebounceRequiresConsecutiveBreaches() {
    SafetySupervisor supervisor;
    RobotState raw = nominalRaw();
    RobotState est = nominalEstimated();
    est.has_measured_body_pose_state = true;
    est.has_body_pose_state = true;

    est.body_pose_state.orientation_rad.x = 2.0;
    const SafetyState first_breach = supervisor.evaluate(
        raw, est, intentNow(RobotMode::WALK), SafetySupervisor::FreshnessInputs{true, true});
    if (!expect(first_breach.active_fault == FaultCode::NONE,
                "first isolated tilt breach should be debounced")) {
        return false;
    }

    est.body_pose_state.orientation_rad.x = 0.0;
    const SafetyState cleared_sample = supervisor.evaluate(
        raw, est, intentNow(RobotMode::WALK), SafetySupervisor::FreshnessInputs{true, true});
    if (!expect(cleared_sample.active_fault == FaultCode::NONE,
                "non-breach sample should keep TIP_OVER clear")) {
        return false;
    }

    for (int sample = 0; sample < 24; ++sample) {
        est.body_pose_state.orientation_rad.x = 2.0;
        const SafetyState pending = supervisor.evaluate(
            raw, est, intentNow(RobotMode::WALK), SafetySupervisor::FreshnessInputs{true, true});
        if (!expect(pending.active_fault == FaultCode::NONE,
                    "TIP_OVER should not trip before required consecutive tilt breaches")) {
            return false;
        }
    }

    est.body_pose_state.orientation_rad.x = 2.0;
    const SafetyState tripped = supervisor.evaluate(
        raw, est, intentNow(RobotMode::WALK), SafetySupervisor::FreshnessInputs{true, true});
    return expect(tripped.active_fault == FaultCode::TIP_OVER,
                  "TIP_OVER should trip once required consecutive tilt breaches are reached") &&
           expect(tripped.active_fault_trip_count == 1,
                  "trip counter should increment once after debounce threshold is met");
}

bool testTiltOnlyTipOverRequiresConfirmationSamplesAndResetsOnClearSample() {
    constexpr uint32_t kTipOverConfirmSamples = 25; // Mirrors SafetySupervisor policy constant.

    {
        SafetySupervisor supervisor;
        RobotState raw = nominalRaw();
        RobotState est = nominalEstimated();
        est.body_pose_state.orientation_rad.x = 2.0;
        est.has_measured_body_pose_state = true;
        est.has_body_pose_state = true;

        for (uint32_t sample = 1; sample < kTipOverConfirmSamples; ++sample) {
            const SafetyState state = supervisor.evaluate(
                raw, est, intentNow(RobotMode::WALK), SafetySupervisor::FreshnessInputs{true, true});
            if (!expect(state.stable, "tilt-only confirmation path requires stable support polygon")) {
                return false;
            }
            if (!expect(state.active_fault != FaultCode::TIP_OVER,
                        "tilt-only violations should not trip before kTipOverConfirmSamples")) {
                return false;
            }
        }

        const SafetyState tripped = supervisor.evaluate(
            raw, est, intentNow(RobotMode::WALK), SafetySupervisor::FreshnessInputs{true, true});
        if (!expect(tripped.active_fault == FaultCode::TIP_OVER,
                    "tilt-only violations should trip exactly at kTipOverConfirmSamples")) {
            return false;
        }
    }

    {
        SafetySupervisor supervisor;
        RobotState raw = nominalRaw();
        RobotState violating_est = nominalEstimated();
        violating_est.body_pose_state.orientation_rad.x = 2.0;
        violating_est.has_measured_body_pose_state = true;
        violating_est.has_body_pose_state = true;

        for (uint32_t sample = 1; sample < kTipOverConfirmSamples; ++sample) {
            const SafetyState state = supervisor.evaluate(
                raw, violating_est, intentNow(RobotMode::WALK), SafetySupervisor::FreshnessInputs{true, true});
            if (!expect(state.active_fault != FaultCode::TIP_OVER,
                        "pre-reset violating samples should still be below trip threshold")) {
                return false;
            }
        }

        RobotState non_violating_est = violating_est;
        non_violating_est.body_pose_state.orientation_rad.x = 0.0;
        const SafetyState reset_sample = supervisor.evaluate(
            raw, non_violating_est, intentNow(RobotMode::WALK), SafetySupervisor::FreshnessInputs{true, true});
        if (!expect(reset_sample.active_fault != FaultCode::TIP_OVER,
                    "single non-violating sample should prevent immediate tip-over trip")) {
            return false;
        }

        const SafetyState after_reset = supervisor.evaluate(
            raw, violating_est, intentNow(RobotMode::WALK), SafetySupervisor::FreshnessInputs{true, true});
        return expect(after_reset.active_fault != FaultCode::TIP_OVER,
                      "candidate counter should reset after one non-violating sample");
    }
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

struct LifecycleFaultCase {
    std::string_view name;
    FaultCode expected_fault;
    RobotMode trip_mode;
    SafetySupervisor::FreshnessInputs trip_freshness;
    void (*mutate)(RobotState& raw, RobotState& est);
};

void setBusTimeoutFault(RobotState& raw, RobotState&) {
    raw.bus_ok = false;
}

void setMotorFault(RobotState& raw, RobotState&) {
    raw.current = 999.0f;
}

void setCommandTimeoutFault(RobotState&, RobotState&) {}

void setEstimatorInvalidFault(RobotState& raw, RobotState&) {
    raw.foot_contacts = {false, false, false, false, false, false};
}

void setJointLimitFault(RobotState&, RobotState& est) {
    est.sample_id = 1;
    est.leg_states[0].joint_state[0].pos_rad = AngleRad{0.0};
}

bool testFaultLifecycleTransitionsAndTripMetadataTableDriven() {
    const std::vector<LifecycleFaultCase> cases{
        LifecycleFaultCase{
            .name = "BUS_TIMEOUT",
            .expected_fault = FaultCode::BUS_TIMEOUT,
            .trip_mode = RobotMode::WALK,
            .trip_freshness = SafetySupervisor::FreshnessInputs{true, true},
            .mutate = setBusTimeoutFault,
        },
        LifecycleFaultCase{
            .name = "MOTOR_FAULT",
            .expected_fault = FaultCode::MOTOR_FAULT,
            .trip_mode = RobotMode::WALK,
            .trip_freshness = SafetySupervisor::FreshnessInputs{true, true},
            .mutate = setMotorFault,
        },
        LifecycleFaultCase{
            .name = "COMMAND_TIMEOUT",
            .expected_fault = FaultCode::COMMAND_TIMEOUT,
            .trip_mode = RobotMode::WALK,
            .trip_freshness = SafetySupervisor::FreshnessInputs{true, false},
            .mutate = setCommandTimeoutFault,
        },
        LifecycleFaultCase{
            .name = "ESTIMATOR_INVALID",
            .expected_fault = FaultCode::ESTIMATOR_INVALID,
            .trip_mode = RobotMode::WALK,
            .trip_freshness = SafetySupervisor::FreshnessInputs{false, true},
            .mutate = setEstimatorInvalidFault,
        },
        LifecycleFaultCase{
            .name = "JOINT_LIMIT",
            .expected_fault = FaultCode::JOINT_LIMIT,
            .trip_mode = RobotMode::WALK,
            .trip_freshness = SafetySupervisor::FreshnessInputs{true, true},
            .mutate = setJointLimitFault,
        },
    };

    for (const LifecycleFaultCase& test_case : cases) {
        SafetySupervisor supervisor;
        RobotState raw = nominalRaw();
        RobotState est = nominalEstimated();

        if (test_case.expected_fault == FaultCode::JOINT_LIMIT) {
            test_case.mutate(raw, est);
            const SafetyState baseline = supervisor.evaluate(
                raw, est, intentNow(RobotMode::WALK), SafetySupervisor::FreshnessInputs{true, true});
            if (!expect(baseline.active_fault == FaultCode::NONE, "joint-limit baseline sample should be clear")) {
                return false;
            }
            est.sample_id = 2;
            est.leg_states[0].joint_state[0].pos_rad = AngleRad{2.0};
        } else {
            test_case.mutate(raw, est);
        }

        const MotionIntent trip_intent =
            test_case.trip_freshness.intent_valid ? intentNow(test_case.trip_mode) : staleIntent(test_case.trip_mode);

        const SafetyState latched = supervisor.evaluate(raw, est, trip_intent, test_case.trip_freshness);
        if (!expect(latched.active_fault == test_case.expected_fault,
                    "table-driven trip should latch expected fault code")) {
            return false;
        }
        if (!expect(latched.fault_lifecycle == FaultLifecycle::LATCHED,
                    "table-driven trip should enter LATCHED lifecycle")) {
            return false;
        }
        if (!expect(latched.active_fault_trip_count == 1,
                    "first trip should set active_fault_trip_count to 1")) {
            return false;
        }
        if (!expect(!latched.active_fault_last_trip_us.isZero(),
                    "first trip should stamp active_fault_last_trip_us")) {
            return false;
        }

        raw = nominalRaw();
        est = nominalEstimated();
        const SafetyState recovering = supervisor.evaluate(
            raw, est, intentNow(RobotMode::SAFE_IDLE), SafetySupervisor::FreshnessInputs{true, true});
        if (!expect(recovering.fault_lifecycle == FaultLifecycle::RECOVERING,
                    "clear path should transition LATCHED -> RECOVERING")) {
            return false;
        }
        if (!expect(recovering.active_fault == test_case.expected_fault,
                    "fault should remain set while RECOVERING hold is active")) {
            return false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(450));
        const SafetyState pre_hold = supervisor.evaluate(
            raw, est, intentNow(RobotMode::SAFE_IDLE), SafetySupervisor::FreshnessInputs{true, true});
        if (!expect(pre_hold.fault_lifecycle == FaultLifecycle::RECOVERING,
                    "fault should remain RECOVERING before hold window elapses")) {
            return false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        const SafetyState near_boundary = supervisor.evaluate(
            raw, est, intentNow(RobotMode::SAFE_IDLE), SafetySupervisor::FreshnessInputs{true, true});
        if (!expect(near_boundary.fault_lifecycle == FaultLifecycle::RECOVERING,
                    "fault should remain RECOVERING at edge timing boundary before 500ms hold")) {
            return false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(70));
        const SafetyState cleared = supervisor.evaluate(
            raw, est, intentNow(RobotMode::SAFE_IDLE), SafetySupervisor::FreshnessInputs{true, true});
        if (!expect(cleared.active_fault == FaultCode::NONE,
                    "fault should clear after hold window is exceeded")) {
            return false;
        }
        if (!expect(cleared.fault_lifecycle == FaultLifecycle::ACTIVE,
                    "lifecycle should return to ACTIVE after clear")) {
            return false;
        }
        if (!expect(cleared.active_fault_trip_count == 0,
                    "clear should zero active fault trip counter for NONE state")) {
            return false;
        }
        if (!expect(cleared.active_fault_last_trip_us.isZero(),
                    "clear should zero active fault last-trip timestamp for NONE state")) {
            return false;
        }

        raw = nominalRaw();
        est = nominalEstimated();
        if (test_case.expected_fault == FaultCode::JOINT_LIMIT) {
            est.sample_id = 100;
            est.leg_states[0].joint_state[0].pos_rad = AngleRad{0.0};
            const SafetyState rebaseline = supervisor.evaluate(
                raw, est, intentNow(RobotMode::WALK), SafetySupervisor::FreshnessInputs{true, true});
            if (!expect(rebaseline.active_fault == FaultCode::NONE,
                        "joint-limit rebaseline should be clear before second trip")) {
                return false;
            }
            est.sample_id = 101;
            est.leg_states[0].joint_state[0].pos_rad = AngleRad{2.0};
        } else {
            test_case.mutate(raw, est);
        }

        const MotionIntent retrip_intent =
            test_case.trip_freshness.intent_valid ? intentNow(test_case.trip_mode) : staleIntent(test_case.trip_mode);
        const SafetyState retripped = supervisor.evaluate(raw, est, retrip_intent, test_case.trip_freshness);
        if (!expect(retripped.active_fault == test_case.expected_fault,
                    "second trip should latch same fault class")) {
            return false;
        }
        if (!expect(retripped.active_fault_trip_count == 2,
                    "fault-class trip counter should increment across clears")) {
            return false;
        }
        if (!expect(retripped.active_fault_last_trip_us.value >= latched.active_fault_last_trip_us.value,
                    "fault-class timestamp should update on second trip")) {
            return false;
        }
    }

    return true;
}

} // namespace

int main() {
    if (!testHigherPriorityFaultWins() ||
        !testTiltViolationBeatsBusTimeoutInHardFaultContext() ||
        !testTiltViolationBeatsMotorFaultWhenVoltageLow() ||
        !testTiltViolationBeatsMotorFaultWhenCurrentHigh() ||
        !testRulePrecedenceAcrossAllFaultTriggers() ||
        !testEstimatorBeatsCommandTimeoutWithoutTorqueCut() ||
        !testBusTimeoutBeatsFreshnessFaults() ||
        !testCombinationPriorityOrderIsDeterministic() ||
        !testInferredTiltDoesNotTripMeasuredTiltPolicy() ||
        !testUnstableSupportTriggersTipOver() ||
        !testSupportInstabilityBypassesTiltDebounce() ||
        !testTiltDebounceRequiresConsecutiveBreaches() ||
        !testTiltOnlyTipOverRequiresConfirmationSamplesAndResetsOnClearSample() ||
        !testMotorFaultTorqueCut() ||
        !testJointDiscontinuityTripsSafety() ||
        !testLatchedRemainsWhenIntentNotSafeIdle() ||
        !testLatchedRemainsWhenIntentStale() ||
        !testRecoveryRequiresBothConditionsAndHoldTime() ||
        !testFaultLifecycleTransitionsAndTripMetadataTableDriven()) {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
