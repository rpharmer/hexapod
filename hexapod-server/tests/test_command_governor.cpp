#include "command_governor.hpp"
#include "support_assessment.hpp"

#include "motion_intent_utils.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {

bool expect(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

MotionIntent makeWalkIntent(const double vx_mps, const double yaw_rate_radps, const uint64_t timestamp_us) {
    MotionIntent intent = makeMotionIntent(RobotMode::WALK, GaitType::TRIPOD, 0.14);
    intent.cmd_vx_mps = LinearRateMps{vx_mps};
    intent.cmd_vy_mps = LinearRateMps{0.0};
    intent.cmd_yaw_radps = AngularRateRadPerSec{yaw_rate_radps};
    intent.speed_mps = LinearRateMps{vx_mps};
    intent.heading_rad = AngleRad{0.0};
    intent.twist.body_trans_m.z = 0.14;
    intent.timestamp_us = TimePointUs{timestamp_us};
    intent.sample_id = timestamp_us;
    return intent;
}

RobotState makeState(double tilt_rad,
                     double body_rate_radps,
                     double trust,
                     double mismatch_ratio,
                     double body_height_m = 0.14) {
    RobotState est{};
    est.valid = true;
    est.timestamp_us = TimePointUs{1'000'000};
    est.has_body_twist_state = true;
    est.body_twist_state.twist_pos_rad.x = tilt_rad * 0.7;
    est.body_twist_state.twist_pos_rad.y = tilt_rad * 0.7;
    est.body_twist_state.body_trans_m.z = body_height_m;
    est.has_imu = true;
    est.imu.valid = true;
    est.imu.gyro_radps.x = body_rate_radps * 0.7;
    est.imu.gyro_radps.y = body_rate_radps * 0.7;
    est.has_fusion_diagnostics = true;
    est.fusion.model_trust = trust;
    est.fusion.residuals.contact_mismatch_ratio = mismatch_ratio;
    return est;
}

GaitState makePreviousGait(double static_margin_m, double clearance_m) {
    GaitState gait{};
    gait.timestamp_us = TimePointUs{1'000'000};
    gait.static_stability_margin_m = static_margin_m;
    gait.in_stance.fill(true);
    gait.support_liftoff_clearance_m.fill(clearance_m);
    gait.support_liftoff_safe_to_lift.fill(clearance_m > 0.0);
    gait.stride_phase_rate_hz = FrequencyHz{1.0};
    return gait;
}

SupportAssessment makeSupport(double static_margin_m,
                              int support_count,
                              int confirmed_support_count = -1,
                              int search_leg_count = 0,
                              int lost_candidate_count = 0,
                              int expected_touchdown_count = 0) {
    SupportAssessment support{};
    support.static_margin_m = static_margin_m;
    support.support_count = support_count;
    support.confirmed_support_count =
        confirmed_support_count >= 0 ? std::min(confirmed_support_count, support_count) : support_count;
    support.uncertain_support_count = support.support_count - support.confirmed_support_count;
    support.search_leg_count = search_leg_count;
    support.lost_candidate_count = lost_candidate_count;
    support.expected_touchdown_count = expected_touchdown_count;
    support.all_support_confirmed =
        support.support_count > 0 && support.uncertain_support_count == 0 &&
        support.search_leg_count == 0 && support.lost_candidate_count == 0 &&
        support.expected_touchdown_count == 0;
    support.sparse_support = support_count <= 3;
    support.degraded_support = support.sparse_support || static_margin_m < 0.0;
    for (int leg = 0; leg < kNumLegs; ++leg) {
        support.effective_support[static_cast<std::size_t>(leg)] = leg < support_count;
    }
    return support;
}

} // namespace

int main() {
    control_config::SafetyConfig safety_config{};
    safety_config.rapid_body_rate_radps = 2.5;
    safety_config.rapid_body_rate_max_contacts = 2;
    safety_config.body_height_collapse_margin_m = 0.05;
    CommandGovernor governor{{}, safety_config};
    SafetyState healthy_safety{};
    healthy_safety.inhibit_motion = false;
    const GaitState steady_gait = makePreviousGait(0.024, 0.018);
    const RobotState healthy_state = makeState(0.03, 0.12, 0.92, 0.04);

    MotionIntent low_speed = makeWalkIntent(0.06, 0.0, 1'000'000);
    const CommandGovernorState low_out =
        governor.apply(healthy_state, low_speed, healthy_safety, steady_gait);
    if (!expect(low_out.command_scale > 0.95, "low-speed command should pass through almost unchanged") ||
        !expect(low_out.cadence_scale > 0.92, "low-speed command should not be cadence-throttled") ||
        !expect(low_out.swing_height_floor_m >= gaitPresetSwingHeightFloor(GaitType::TRIPOD),
                "low-speed command should preserve the preset swing floor")) {
        return EXIT_FAILURE;
    }

    MotionIntent aggressive = makeWalkIntent(0.34, 0.68, 1'004'000);
    const RobotState stressed_state = makeState(0.24, 1.25, 0.52, 0.34);
    const GaitState stressed_gait = makePreviousGait(-0.006, -0.010);
    const CommandGovernorState stressed_out =
        governor.apply(stressed_state, aggressive, healthy_safety, stressed_gait);

    if (!expect(stressed_out.command_scale < low_out.command_scale,
                "worse stability should attenuate the commanded speed") ||
        !expect(stressed_out.cadence_scale <= low_out.cadence_scale,
                "worse stability should not increase cadence") ||
        !expect(stressed_out.swing_height_floor_m >= low_out.swing_height_floor_m,
                "worse stability should not reduce swing floor") ||
        !expect(stressed_out.body_height_delta_m <= low_out.body_height_delta_m,
                "worse stability should squat the body more, not less") ||
        !expect(stressed_out.command_scale > 0.25,
                "aggressive motion should be reduced progressively, not collapsed to zero")) {
        return EXIT_FAILURE;
    }

    MotionIntent boundary_low = makeWalkIntent(0.119, 0.24, 2'000'000);
    MotionIntent boundary_high = makeWalkIntent(0.121, 0.26, 2'004'000);
    CommandGovernor boundary_low_governor{{}, safety_config};
    CommandGovernor boundary_high_governor{{}, safety_config};
    const CommandGovernorState boundary_low_out =
        boundary_low_governor.apply(healthy_state, boundary_low, healthy_safety, steady_gait);
    const CommandGovernorState boundary_high_out =
        boundary_high_governor.apply(healthy_state, boundary_high, healthy_safety, steady_gait);

    if (!expect(std::abs(boundary_low_out.command_scale - boundary_high_out.command_scale) < 0.05,
                "command governor should remain continuous near the low-speed boundary") ||
        !expect(std::abs(boundary_low_out.cadence_scale - boundary_high_out.cadence_scale) < 0.05,
                "cadence shaping should remain continuous near the low-speed boundary")) {
        return EXIT_FAILURE;
    }

    MotionIntent recovery_intent = makeWalkIntent(0.24, 0.52, 3'000'000);
    const RobotState recovery_state = makeState(0.22, 3.20, 0.88, 0.02);
    const SupportAssessment sparse_support = makeSupport(-0.018, 1);
    const CommandGovernorState recovery_out =
        governor.apply(recovery_state, recovery_intent, healthy_safety, stressed_gait, &sparse_support);
    if (!expect(recovery_out.recovery_stage == RecoveryStage::ActiveHold,
                "sparse unstable walk should enter active recovery hold") ||
        !expect(recovery_out.recovery_hold_active, "active recovery hold should set hold-active flag") ||
        !expect(recovery_out.freeze_phase, "active recovery hold should freeze gait phase") ||
        !expect(recovery_out.command_scale == 0.0, "recovery hold should zero command scale") ||
        !expect(recovery_out.cadence_scale == 0.0, "recovery hold should zero cadence scale") ||
        !expect(recovery_out.body_height_delta_m == 0.0, "recovery hold should suppress body squat") ||
        !expect(recovery_out.current_support_count == 1, "recovery hold should report current support count") ||
        !expect(hasCommandGovernorReason(recovery_out.reasons, CommandGovernorReason::RecoveryHold),
                "recovery hold should set its reason bit")) {
        return EXIT_FAILURE;
    }

    MotionIntent held_preview = makeWalkIntent(0.24, 0.52, 3'004'000);
    const CommandGovernorState held_preview_out =
        governor.preview(recovery_state, held_preview, healthy_safety, stressed_gait);
    if (!expect(held_preview_out.recovery_hold_active,
                "recovery hold should persist across the preview pass before support refresh") ||
        !expect(held_preview_out.recovery_stage == RecoveryStage::ActiveHold,
                "preview pass should preserve the active-hold stage") ||
        !expect(held_preview_out.freeze_phase, "preview pass should preserve frozen phase while held")) {
        return EXIT_FAILURE;
    }

    MotionIntent post_preview_hold = makeWalkIntent(0.24, 0.52, 3'008'000);
    const CommandGovernorState post_preview_hold_out =
        governor.apply(recovery_state, post_preview_hold, healthy_safety, stressed_gait, &sparse_support);
    if (!expect(post_preview_hold_out.recovery_hold_active,
                "preview pass should not clear recovery hold before the real evaluation")) {
        return EXIT_FAILURE;
    }

    CommandGovernor zero_margin_governor{{}, safety_config};
    MotionIntent zero_margin_intent = makeWalkIntent(0.24, 0.52, 3'020'000);
    GaitState blocked_previous_gait = makePreviousGait(0.0, -0.004);
    blocked_previous_gait.support_liftoff_safe_to_lift.fill(false);
    const SupportAssessment zero_margin_support = makeSupport(0.0, kNumLegs);
    const CommandGovernorState zero_margin_out = zero_margin_governor.apply(
        recovery_state, zero_margin_intent, healthy_safety, blocked_previous_gait, &zero_margin_support);
    if (!expect(zero_margin_out.recovery_stage == RecoveryStage::ActiveHold,
                "zero-margin deadlock should enter active recovery hold") ||
        !expect(zero_margin_out.recovery_hold_active,
                "zero-margin deadlock should still latch recovery hold before the support polygon goes negative") ||
        !expect(zero_margin_out.freeze_phase, "zero-margin deadlock should freeze gait phase")) {
        return EXIT_FAILURE;
    }

    const SupportAssessment uncertain_support = makeSupport(0.028, 6, 4, 1, 1, 0);
    MotionIntent recovery_release_a = makeWalkIntent(0.05, 0.0, 3'104'000);
    const RobotState calmer_state = makeState(0.05, 1.2, 0.90, 0.02);
    CommandGovernorState recovery_release_a_out =
        governor.apply(calmer_state, recovery_release_a, healthy_safety, steady_gait, &uncertain_support);
    recovery_release_a_out = governor.finalizeRecovery(
        calmer_state, recovery_release_a, healthy_safety, steady_gait, uncertain_support, recovery_release_a_out);
    MotionIntent recovery_release_b = makeWalkIntent(0.05, 0.0, 3'604'000);
    CommandGovernorState recovery_release_b_out =
        governor.apply(calmer_state, recovery_release_b, healthy_safety, steady_gait, &uncertain_support);
    recovery_release_b_out = governor.finalizeRecovery(
        calmer_state, recovery_release_b, healthy_safety, steady_gait, uncertain_support, recovery_release_b_out);
    if (!expect(recovery_release_a_out.recovery_stage == RecoveryStage::ActiveHold,
                "uncertain support should keep recovery in active hold") ||
        !expect(recovery_release_b_out.recovery_stage == RecoveryStage::ActiveHold,
                "uncertain support should block release even after a long healthy-looking window")) {
        return EXIT_FAILURE;
    }

    const SupportAssessment fully_confirmed_support = makeSupport(0.028, 6, 6);
    MotionIntent confirmed_release_a = makeWalkIntent(0.05, 0.0, 4'000'000);
    CommandGovernorState confirmed_release_a_out =
        governor.apply(calmer_state, confirmed_release_a, healthy_safety, steady_gait, &fully_confirmed_support);
    confirmed_release_a_out = governor.finalizeRecovery(
        calmer_state, confirmed_release_a, healthy_safety, steady_gait, fully_confirmed_support, confirmed_release_a_out);
    MotionIntent confirmed_release_b = makeWalkIntent(0.05, 0.0, 4'350'000);
    CommandGovernorState confirmed_release_b_out =
        governor.apply(calmer_state, confirmed_release_b, healthy_safety, steady_gait, &fully_confirmed_support);
    confirmed_release_b_out = governor.finalizeRecovery(
        calmer_state, confirmed_release_b, healthy_safety, steady_gait, fully_confirmed_support, confirmed_release_b_out);
    if (!expect(confirmed_release_a_out.recovery_stage == RecoveryStage::ActiveHold,
                "recovery should wait for the full healthy-confirmation window") ||
        !expect(confirmed_release_b_out.recovery_stage == RecoveryStage::Settling,
                "recovery should move into settling after the healthy-confirmation window") ||
        !expect(confirmed_release_b_out.freeze_phase,
                "settling should keep gait phase frozen") ||
        !expect(confirmed_release_b_out.command_scale == 0.0,
                "settling should still fully brake locomotion") ||
        !expect(confirmed_release_b_out.body_height_delta_m == 0.0,
                "settling should suppress body squat")) {
        return EXIT_FAILURE;
    }

    MotionIntent settling_intent = makeWalkIntent(0.05, 0.0, 4'650'000);
    CommandGovernorState settling_out =
        governor.apply(calmer_state, settling_intent, healthy_safety, steady_gait, &fully_confirmed_support);
    settling_out = governor.finalizeRecovery(
        calmer_state, settling_intent, healthy_safety, steady_gait, fully_confirmed_support, settling_out);
    if (!expect(settling_out.recovery_stage == RecoveryStage::RampOut,
                "settling should transition into ramp-out after its dwell time") ||
        !expect(!settling_out.freeze_phase, "ramp-out should release gait phase") ||
        !expect(settling_out.command_scale < 0.2,
                "ramp-out should start near zero command scale") ||
        !expect(settling_out.body_height_delta_m == 0.0,
                "ramp-out should suppress body squat")) {
        return EXIT_FAILURE;
    }

    const SupportAssessment nominal_walk_support = makeSupport(0.022, 5, 5);
    MotionIntent ramp_out_continue_intent = makeWalkIntent(0.14, 0.0, 4'950'000);
    const RobotState ramp_out_continue_state = makeState(0.04, 0.45, 0.92, 0.03);
    CommandGovernorState ramp_out_continue_out =
        governor.apply(ramp_out_continue_state,
                       ramp_out_continue_intent,
                       healthy_safety,
                       steady_gait,
                       &nominal_walk_support);
    ramp_out_continue_out = governor.finalizeRecovery(ramp_out_continue_state,
                                                      ramp_out_continue_intent,
                                                      healthy_safety,
                                                      steady_gait,
                                                      nominal_walk_support,
                                                      ramp_out_continue_out);
    if (!expect(ramp_out_continue_out.recovery_stage == RecoveryStage::RampOut,
                "ramp-out should tolerate healthy non-six-leg walking support") ||
        !expect(!ramp_out_continue_out.recovery_hold_active,
                "ramp-out should stay out of hold during healthy walking support") ||
        !expect(ramp_out_continue_out.body_height_delta_m == 0.0,
                "ramp-out should keep body-height squat suppressed while walking resumes")) {
        return EXIT_FAILURE;
    }

    MotionIntent trust_recovery_intent = makeWalkIntent(0.14, 0.0, 5'000'000);
    const RobotState trust_recovery_state = makeState(0.03, 0.55, 0.62, 0.28);
    CommandGovernorState trust_recovery_out =
        governor.apply(trust_recovery_state, trust_recovery_intent, healthy_safety, steady_gait, &nominal_walk_support);
    trust_recovery_out = governor.finalizeRecovery(
        trust_recovery_state, trust_recovery_intent, healthy_safety, steady_gait, nominal_walk_support, trust_recovery_out);
    if (!expect(trust_recovery_out.recovery_stage == RecoveryStage::RampOut,
                "ramp-out should stay active while unhealthy trust or contact quality blocks completion") ||
        !expect(!trust_recovery_out.recovery_hold_active,
                "trust or contact quality alone should not force recovery hold without dynamic collapse")) {
        return EXIT_FAILURE;
    }

    MotionIntent burst_recovery_intent_a = makeWalkIntent(0.24, 0.52, 5'050'000);
    const CommandGovernorState burst_recovery_out_a =
        governor.apply(recovery_state, burst_recovery_intent_a, healthy_safety, stressed_gait, &sparse_support);
    if (!expect(burst_recovery_out_a.recovery_stage == RecoveryStage::ActiveHold,
                "support degradation during ramp-out should re-enter active hold") ||
        !expect(burst_recovery_out_a.recovery_hold_active,
                "support degradation during ramp-out should re-enter recovery hold")) {
        return EXIT_FAILURE;
    }

    MotionIntent burst_release_attempt_a = makeWalkIntent(0.05, 0.0, 5'150'000);
    CommandGovernorState burst_release_attempt_a_out =
        governor.apply(calmer_state, burst_release_attempt_a, healthy_safety, steady_gait, &fully_confirmed_support);
    burst_release_attempt_a_out = governor.finalizeRecovery(
        calmer_state, burst_release_attempt_a, healthy_safety, steady_gait, fully_confirmed_support, burst_release_attempt_a_out);
    if (!expect(burst_release_attempt_a_out.recovery_stage == RecoveryStage::ActiveHold,
                "freshly re-entered recovery should ignore brief healthy windows")) {
        return EXIT_FAILURE;
    }

    MotionIntent burst_release_attempt_b = makeWalkIntent(0.05, 0.0, 5'500'000);
    CommandGovernorState burst_release_attempt_b_out =
        governor.apply(calmer_state, burst_release_attempt_b, healthy_safety, steady_gait, &fully_confirmed_support);
    burst_release_attempt_b_out = governor.finalizeRecovery(
        calmer_state, burst_release_attempt_b, healthy_safety, steady_gait, fully_confirmed_support, burst_release_attempt_b_out);
    if (!expect(burst_release_attempt_b_out.recovery_stage == RecoveryStage::Settling,
                "recovery should re-enter settling after the healthy-confirmation window")) {
        return EXIT_FAILURE;
    }

    MotionIntent degrade_during_settle = makeWalkIntent(0.05, 0.0, 5'560'000);
    CommandGovernorState degrade_during_settle_out =
        governor.apply(calmer_state, degrade_during_settle, healthy_safety, steady_gait, &fully_confirmed_support);
    degrade_during_settle_out = governor.finalizeRecovery(
        calmer_state, degrade_during_settle, healthy_safety, steady_gait, sparse_support, degrade_during_settle_out);
    if (!expect(degrade_during_settle_out.recovery_stage == RecoveryStage::ActiveHold,
                "support degradation during settling should re-enter active hold")) {
        return EXIT_FAILURE;
    }

    // --- New: filtered RampOut abort gate rejects single-cycle dynamic spikes ---
    // Drive a fresh governor cleanly through ActiveHold → Settling → RampOut, then inject
    // a single-tick body-rate spike. The filter+persist+min-dwell gate should NOT abort on
    // the spike (the breach has not persisted long enough). The same scenario in the
    // pre-change code would have aborted instantly on the spike.
    {
        CommandGovernor spike_governor{{}, safety_config};
        const SupportAssessment six_legs_clean = makeSupport(0.030, 6, 6);
        const RobotState calm_six = makeState(0.04, 0.40, 0.92, 0.03);

        // Latch into ActiveHold by simulating a degraded prior frame.
        MotionIntent enter = makeWalkIntent(0.20, 0.30, 6'000'000);
        const RobotState bad = makeState(0.22, 3.20, 0.85, 0.03);
        const SupportAssessment sparse = makeSupport(-0.020, 1);
        spike_governor.apply(bad, enter, healthy_safety, steady_gait, &sparse);

        // Wait the healthy-confirmation window with clean state to advance to Settling.
        MotionIntent confirm_a = makeWalkIntent(0.05, 0.0, 6'050'000);
        CommandGovernorState s = spike_governor.apply(calm_six, confirm_a, healthy_safety, steady_gait, &six_legs_clean);
        s = spike_governor.finalizeRecovery(calm_six, confirm_a, healthy_safety, steady_gait, six_legs_clean, s);
        MotionIntent confirm_b = makeWalkIntent(0.05, 0.0, 6'400'000);
        s = spike_governor.apply(calm_six, confirm_b, healthy_safety, steady_gait, &six_legs_clean);
        s = spike_governor.finalizeRecovery(calm_six, confirm_b, healthy_safety, steady_gait, six_legs_clean, s);
        // Settling → RampOut after dwell.
        MotionIntent into_rampout = makeWalkIntent(0.05, 0.0, 6'700'000);
        s = spike_governor.apply(calm_six, into_rampout, healthy_safety, steady_gait, &six_legs_clean);
        s = spike_governor.finalizeRecovery(calm_six, into_rampout, healthy_safety, steady_gait, six_legs_clean, s);
        if (!expect(s.recovery_stage == RecoveryStage::RampOut,
                    "spike rejection setup: should reach RampOut after settling dwell")) {
            return EXIT_FAILURE;
        }

        // Single-tick body-rate spike well past the soft threshold. Filter has tau=80 ms,
        // persist=100 ms; one tick of breach (~10 ms apart) cannot persist.
        const RobotState spike = makeState(0.13, 1.8, 0.92, 0.03);
        MotionIntent spike_intent = makeWalkIntent(0.10, 0.0, 6'710'000);
        s = spike_governor.apply(spike, spike_intent, healthy_safety, steady_gait, &six_legs_clean);
        s = spike_governor.finalizeRecovery(spike, spike_intent, healthy_safety, steady_gait, six_legs_clean, s);
        if (!expect(s.recovery_stage == RecoveryStage::RampOut,
                    "filtered abort gate should ignore a single-tick body-rate spike")) {
            return EXIT_FAILURE;
        }
    }

    // --- New: body-height delta slews on RELEASE, engages instantly on PROTECT ---
    // After steady walking with non-trivial squat, dropping severity should not step the
    // body up by the full squat in one tick. Increase severity, however, must engage the
    // squat in one tick.
    {
        CommandGovernor slew_governor{{}, safety_config};
        // Burn in a sustained-stress scenario to set body_height_delta_m to its squatted target.
        const RobotState stressed = makeState(0.20, 1.10, 0.55, 0.30);
        const GaitState stressed_gait_local = makePreviousGait(-0.006, -0.010);
        for (uint64_t t = 7'000'000; t <= 7'120'000; t += 4000) {
            MotionIntent intent = makeWalkIntent(0.30, 0.50, t);
            (void)slew_governor.apply(stressed, intent, healthy_safety, stressed_gait_local);
        }
        MotionIntent stressed_intent = makeWalkIntent(0.30, 0.50, 7'124'000);
        const CommandGovernorState stressed_burnin =
            slew_governor.apply(stressed, stressed_intent, healthy_safety, stressed_gait_local);
        const double squatted_delta = stressed_burnin.body_height_delta_m;
        if (!expect(squatted_delta < -0.005,
                    "burn-in stress should drive body-height delta solidly negative")) {
            return EXIT_FAILURE;
        }
        // Now drop into a low-severity scenario one tick later (4 ms). With a 0.04 m/s slew
        // the release per tick is only ~0.00016 m, so the delta must NOT have stepped to ~0.
        const RobotState calm = makeState(0.03, 0.20, 0.95, 0.02);
        MotionIntent calm_intent = makeWalkIntent(0.04, 0.0, 7'128'000);
        const CommandGovernorState first_release_step =
            slew_governor.apply(calm, calm_intent, healthy_safety, steady_gait);
        const double release_step = first_release_step.body_height_delta_m - squatted_delta;
        if (!expect(release_step >= 0.0 && release_step < 0.001,
                    "body-height delta should slew up gradually, not step to target in one tick")) {
            return EXIT_FAILURE;
        }
        // Increase severity again — protective engage must be instant (full step down).
        MotionIntent stress_again = makeWalkIntent(0.30, 0.50, 7'132'000);
        const CommandGovernorState protect_engage =
            slew_governor.apply(stressed, stress_again, healthy_safety, stressed_gait_local);
        if (!expect(protect_engage.body_height_delta_m <= squatted_delta + 1e-6,
                    "protective squat must engage in one tick (no slew on the engage direction)")) {
            return EXIT_FAILURE;
        }
    }

    // --- New: RampOut respects the minimum-dwell guard ---
    // Even with a sustained breach, abort is gated by ramp_out_min_dwell_s. Within that
    // window, the FSM should remain in RampOut.
    {
        CommandGovernor dwell_governor{{}, safety_config};
        const SupportAssessment six_legs_clean = makeSupport(0.030, 6, 6);
        const RobotState calm_six = makeState(0.04, 0.40, 0.92, 0.03);

        MotionIntent enter = makeWalkIntent(0.20, 0.30, 8'000'000);
        const RobotState bad = makeState(0.22, 3.20, 0.85, 0.03);
        const SupportAssessment sparse = makeSupport(-0.020, 1);
        dwell_governor.apply(bad, enter, healthy_safety, steady_gait, &sparse);

        for (uint64_t t = 8'050'000; t <= 8'700'000; t += 300'000) {
            MotionIntent step = makeWalkIntent(0.05, 0.0, t);
            CommandGovernorState s = dwell_governor.apply(calm_six, step, healthy_safety, steady_gait, &six_legs_clean);
            s = dwell_governor.finalizeRecovery(calm_six, step, healthy_safety, steady_gait, six_legs_clean, s);
            (void)s;
        }
        // Drive a sustained dynamic breach immediately after entering RampOut.
        MotionIntent breach_a = makeWalkIntent(0.10, 0.0, 8'750'000);
        const RobotState sustained = makeState(0.18, 1.7, 0.92, 0.03);
        CommandGovernorState s_a = dwell_governor.apply(sustained, breach_a, healthy_safety, steady_gait, &six_legs_clean);
        s_a = dwell_governor.finalizeRecovery(sustained, breach_a, healthy_safety, steady_gait, six_legs_clean, s_a);
        // Even with sustained breach, the min-dwell guard should keep us in RampOut for now.
        if (!expect(s_a.recovery_stage == RecoveryStage::RampOut,
                    "min-dwell guard should keep RampOut from aborting too early")) {
            return EXIT_FAILURE;
        }
    }

    return EXIT_SUCCESS;
}
