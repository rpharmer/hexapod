#include "command_governor.hpp"

#include "motion_intent_utils.hpp"
#include "support_assessment.hpp"

#include <algorithm>
#include <cmath>

namespace {

constexpr DurationUs kRecoveryHealthyConfirmUs{300'000};
constexpr DurationUs kRecoverySettlingUs{250'000};
constexpr DurationUs kRecoveryRampOutUs{1'200'000};
constexpr double kRecoveryReleaseHeightBufferM = 0.01;
constexpr int kRecoveryReleaseSupportCount = kNumLegs;

double localClamp01(const double x) {
    return std::clamp(x, 0.0, 1.0);
}

double smoothStep01(const double x) {
    const double t = localClamp01(x);
    return t * t * (3.0 - 2.0 * t);
}

double pressureWhenValueIsBelowSoft(const double value, const double soft, const double hard) {
    if (hard >= soft) {
        return value < soft ? 1.0 : 0.0;
    }
    const double t = localClamp01((soft - value) / (soft - hard));
    return smoothStep01(t);
}

double pressureWhenValueIsAboveSoft(const double value, const double soft, const double hard) {
    if (hard <= soft) {
        return value > soft ? 1.0 : 0.0;
    }
    const double t = localClamp01((value - soft) / (hard - soft));
    return smoothStep01(t);
}

double meanAbsBodyRateRadps(const RobotState& est) {
    if (!est.has_imu || !est.imu.valid) {
        return 0.0;
    }
    return std::hypot(est.imu.gyro_radps.x, est.imu.gyro_radps.y);
}

double supportMarginEstimate(const GaitState& prev_gait, const control_config::CommandGovernorConfig& cfg) {
    double min_clearance = cfg.startup_support_margin_m;
    for (const double clearance : prev_gait.support_liftoff_clearance_m) {
        if (std::isfinite(clearance)) {
            min_clearance = std::min(min_clearance, clearance);
        }
    }
    const double margin = prev_gait.timestamp_us.isZero()
                              ? cfg.startup_support_margin_m
                              : std::min(prev_gait.static_stability_margin_m, min_clearance);
    return std::isfinite(margin) ? margin : cfg.startup_support_margin_m;
}

void scaleLocomotionIntent(MotionIntent& intent, const double scale) {
    const double s = std::clamp(scale, 0.0, 1.0);
    intent.cmd_vx_mps = LinearRateMps{intent.cmd_vx_mps.value * s};
    intent.cmd_vy_mps = LinearRateMps{intent.cmd_vy_mps.value * s};
    intent.cmd_yaw_radps = AngularRateRadPerSec{intent.cmd_yaw_radps.value * s};
    intent.speed_mps = LinearRateMps{intent.speed_mps.value * s};
    intent.twist.body_trans_mps.x *= s;
    intent.twist.body_trans_mps.y *= s;
    intent.twist.body_trans_mps.z *= s;
    intent.twist.twist_vel_radps.x *= s;
    intent.twist.twist_vel_radps.y *= s;
    intent.twist.twist_vel_radps.z *= s;
}

double bodyTiltMagnitudeRad(const RobotState& est) {
    if (!est.has_body_twist_state ||
        !std::isfinite(est.body_twist_state.twist_pos_rad.x) ||
        !std::isfinite(est.body_twist_state.twist_pos_rad.y)) {
        return 0.0;
    }
    return std::hypot(est.body_twist_state.twist_pos_rad.x, est.body_twist_state.twist_pos_rad.y);
}

double releaseBodyRateLimit(const control_config::CommandGovernorConfig& cfg,
                            const control_config::SafetyConfig& safety) {
    if (safety.rapid_body_rate_radps > 0.0) {
        return std::max(0.0, safety.rapid_body_rate_radps * 0.75);
    }
    return cfg.body_rate_soft_radps;
}

bool allLegsInStance(const GaitState& gait) {
    return std::all_of(gait.in_stance.begin(), gait.in_stance.end(), [](const bool in_stance) {
        return in_stance;
    });
}

double releaseHeightHeadroomM(const RobotState& est,
                              const MotionIntent& intent,
                              const control_config::SafetyConfig& safety) {
    const double commanded_body_height_m = intent.twist.body_trans_m.z;
    const double measured_body_height_m =
        (est.has_body_twist_state && std::isfinite(est.body_twist_state.body_trans_m.z))
            ? est.body_twist_state.body_trans_m.z
            : commanded_body_height_m;
    const double threshold_m = safety.body_height_collapse_min_safe_m > 0.0
                                   ? (safety.body_height_collapse_min_safe_m + kRecoveryReleaseHeightBufferM)
                                   : (commanded_body_height_m - safety.body_height_collapse_margin_m +
                                      kRecoveryReleaseHeightBufferM);
    return measured_body_height_m - threshold_m;
}

bool suppressBodySquatForRecoveryStage(const RecoveryStage stage) {
    return stage == RecoveryStage::ActiveHold ||
           stage == RecoveryStage::Settling ||
           stage == RecoveryStage::RampOut;
}

// First-order exponential moving average. Returns the post-step value of `state`.
// alpha is computed as 1 - exp(-dt/tau); for small dt/tau this approximates dt/tau.
double ewmaStep(double state, const double sample, const double dt_s, const double tau_s) {
    if (!(tau_s > 0.0) || !(dt_s > 0.0)) {
        return sample;
    }
    const double alpha = 1.0 - std::exp(-dt_s / tau_s);
    return state + alpha * (sample - state);
}

// Compute clock delta in seconds since `prev_clock`, advancing it to `now`. Returns 0
// when prev_clock is unset (first sample) or non-monotonic.
double advanceClock(TimePointUs& prev_clock, const TimePointUs now) {
    if (prev_clock.isZero() || now.value <= prev_clock.value) {
        prev_clock = now;
        return 0.0;
    }
    const double dt_s = static_cast<double>(now.value - prev_clock.value) * 1e-6;
    prev_clock = now;
    return dt_s;
}

// Slew `state` toward `target` by at most rate*dt; returns the post-step value.
double slewToward(const double state, const double target, const double rate_per_s, const double dt_s) {
    if (!(dt_s > 0.0) || !(rate_per_s > 0.0)) {
        return target;
    }
    const double max_step = rate_per_s * dt_s;
    const double diff = target - state;
    if (diff > max_step) {
        return state + max_step;
    }
    if (diff < -max_step) {
        return state - max_step;
    }
    return target;
}

// Normalised health margin in [0, 1]. 1.0 = healthy at the soft side; 0.0 = at or past the
// hard side. Used by the RampOut/Settling abort gate to give a single hysteretic signal
// instead of an OR of instantaneous breaches across four independent inputs.
double healthMarginFromPressure(const double value, const double soft, const double hard, const bool above) {
    return 1.0 - (above ? pressureWhenValueIsAboveSoft(value, soft, hard)
                        : pressureWhenValueIsBelowSoft(value, soft, hard));
}

} // namespace

const char* recoveryStageName(const RecoveryStage stage) {
    switch (stage) {
        case RecoveryStage::None:
            return "none";
        case RecoveryStage::ActiveHold:
            return "active_hold";
        case RecoveryStage::Settling:
            return "settling";
        case RecoveryStage::RampOut:
            return "ramp_out";
    }
    return "unknown";
}

CommandGovernor::CommandGovernor(control_config::CommandGovernorConfig config,
                                 control_config::SafetyConfig safety_config)
    : config_(config),
      safety_config_(safety_config) {}

void CommandGovernor::reset() {
    last_clock_ = TimePointUs{};
    last_requested_planar_speed_mps_ = 0.0;
    last_requested_yaw_rate_radps_ = 0.0;
    recovery_stage_ = RecoveryStage::None;
    recovery_stage_started_ = TimePointUs{};
    recovery_healthy_since_ = TimePointUs{};
    body_height_delta_state_ = 0.0;
    body_height_delta_clock_ = TimePointUs{};
    ramp_out_health_filtered_ = 1.0;
    ramp_out_health_clock_ = TimePointUs{};
    ramp_out_breach_since_ = TimePointUs{};
}

void CommandGovernor::latchRecoveryHold(TimePointUs now) {
    if (now.isZero()) {
        now = now_us();
    }
    enterRecoveryHold(now);
}

void CommandGovernor::setRecoveryStage(const RecoveryStage stage, const TimePointUs now) {
    if (recovery_stage_ == stage) {
        return;
    }
    recovery_stage_ = stage;
    recovery_stage_started_ = now;
    if (stage != RecoveryStage::ActiveHold) {
        recovery_healthy_since_ = TimePointUs{};
    }
}

void CommandGovernor::enterRecoveryHold(const TimePointUs now) {
    setRecoveryStage(RecoveryStage::ActiveHold, now);
    recovery_healthy_since_ = TimePointUs{};
}

void CommandGovernor::applyRecoveryStageState(const TimePointUs now, CommandGovernorState& out) const {
    out.recovery_stage = recovery_stage_;
    out.recovery_hold_active =
        recovery_stage_ == RecoveryStage::ActiveHold || recovery_stage_ == RecoveryStage::Settling;
    out.freeze_phase =
        recovery_stage_ == RecoveryStage::ActiveHold || recovery_stage_ == RecoveryStage::Settling;

    if (recovery_stage_ == RecoveryStage::ActiveHold || recovery_stage_ == RecoveryStage::Settling) {
        out.command_scale = 0.0;
        out.cadence_scale = 0.0;
        out.saturated = true;
        out.reasons |= CommandGovernorReason::RecoveryHold;
        return;
    }

    if (recovery_stage_ == RecoveryStage::RampOut) {
        const double elapsed_us = static_cast<double>((now - recovery_stage_started_).value);
        const double ramp_t = std::clamp(elapsed_us / static_cast<double>(kRecoveryRampOutUs.value), 0.0, 1.0);
        const double ramp_scale = smoothStep01(ramp_t);
        out.command_scale = std::min(out.command_scale, ramp_scale);
        out.cadence_scale = std::min(out.cadence_scale, ramp_scale);
        out.saturated = out.saturated || ramp_scale < 0.999;
    }
}

CommandGovernorState CommandGovernor::preview(const RobotState& est,
                                              MotionIntent& intent,
                                              const SafetyState& safety,
                                              const GaitState& previous_gait,
                                              const SupportAssessment* current_support) const {
    CommandGovernor snapshot = *this;
    return snapshot.apply(est, intent, safety, previous_gait, current_support);
}

CommandGovernorState CommandGovernor::apply(const RobotState& est,
                                            MotionIntent& intent,
                                            const SafetyState& safety,
                                            const GaitState& previous_gait,
                                            const SupportAssessment* current_support) {
    CommandGovernorState out{};
    const PlanarMotionCommand requested = planarMotionCommand(intent);
    out.requested_planar_speed_mps = std::hypot(requested.vx_mps, requested.vy_mps);
    out.requested_yaw_rate_radps = std::abs(requested.yaw_rate_radps);
    out.requested_body_height_m = intent.twist.body_trans_m.z;

    if (current_support != nullptr) {
        out.current_support_margin_m = current_support->static_margin_m;
        out.current_support_count = current_support->support_count;
        out.confirmed_support_count = current_support->confirmed_support_count;
        out.uncertain_support_count = current_support->uncertain_support_count;
        out.search_leg_count = current_support->search_leg_count;
        out.lost_candidate_count = current_support->lost_candidate_count;
        out.expected_touchdown_count = current_support->expected_touchdown_count;
        out.all_support_confirmed = current_support->all_support_confirmed;
    }

    if (intent.requested_mode != RobotMode::WALK || safety.inhibit_motion || safety.torque_cut) {
        recovery_stage_ = RecoveryStage::None;
        recovery_stage_started_ = TimePointUs{};
        recovery_healthy_since_ = TimePointUs{};
        out.governed_planar_speed_mps = out.requested_planar_speed_mps;
        out.governed_yaw_rate_radps = out.requested_yaw_rate_radps;
        out.governed_body_height_m = out.requested_body_height_m;
        out.swing_height_floor_m = gaitPresetSwingHeightFloor(intent.gait);
        last_clock_ = intent.timestamp_us.isZero() ? now_us() : intent.timestamp_us;
        last_requested_planar_speed_mps_ = out.requested_planar_speed_mps;
        last_requested_yaw_rate_radps_ = out.requested_yaw_rate_radps;
        out.recovery_collapse_headroom_m = releaseHeightHeadroomM(est, intent, safety_config_);
        return out;
    }

    const TimePointUs now = intent.timestamp_us.isZero() ? now_us() : intent.timestamp_us;
    double command_accel = 0.0;
    if (!last_clock_.isZero() && now.value > last_clock_.value) {
        const double dt_s = static_cast<double>(now.value - last_clock_.value) * 1e-6;
        if (dt_s > 1e-9) {
            const double dv = out.requested_planar_speed_mps - last_requested_planar_speed_mps_;
            const double dy = out.requested_yaw_rate_radps - last_requested_yaw_rate_radps_;
            command_accel = std::hypot(dv, dy) / dt_s;
        }
    }
    last_clock_ = now;
    last_requested_planar_speed_mps_ = out.requested_planar_speed_mps;
    last_requested_yaw_rate_radps_ = out.requested_yaw_rate_radps;

    const double support_margin = supportMarginEstimate(previous_gait, config_);
    const double body_tilt_rad = bodyTiltMagnitudeRad(est);
    const double body_rate_radps = meanAbsBodyRateRadps(est);
    const double fusion_trust = est.has_fusion_diagnostics ? std::clamp(est.fusion.model_trust, 0.0, 1.0) : 1.0;
    const double contact_mismatch =
        est.has_fusion_diagnostics ? std::clamp(est.fusion.residuals.contact_mismatch_ratio, 0.0, 1.0) : 0.0;

    const bool low_speed_regime =
        out.requested_planar_speed_mps <= config_.low_speed_planar_cutoff_mps &&
        out.requested_yaw_rate_radps <= config_.low_speed_yaw_cutoff_radps;

    double support_pressure =
        pressureWhenValueIsBelowSoft(support_margin, config_.support_margin_soft_m, config_.support_margin_hard_m);
    double tilt_pressure = pressureWhenValueIsAboveSoft(body_tilt_rad, config_.tilt_soft_rad, config_.tilt_hard_rad);
    double body_rate_pressure =
        pressureWhenValueIsAboveSoft(body_rate_radps, config_.body_rate_soft_radps, config_.body_rate_hard_radps);
    double trust_pressure =
        pressureWhenValueIsBelowSoft(fusion_trust, config_.fusion_trust_soft, config_.fusion_trust_hard);
    double contact_pressure = pressureWhenValueIsAboveSoft(
        contact_mismatch, config_.contact_mismatch_soft, config_.contact_mismatch_hard);
    double accel_pressure =
        pressureWhenValueIsAboveSoft(command_accel, config_.command_accel_soft_mps2, config_.command_accel_hard_mps2);

    if (low_speed_regime) {
        body_rate_pressure *= 0.5;
        accel_pressure *= 0.5;
    }

    constexpr double kSupportWeight = 0.28;
    constexpr double kTiltWeight = 0.22;
    constexpr double kBodyRateWeight = 0.18;
    constexpr double kTrustWeight = 0.14;
    constexpr double kContactWeight = 0.10;
    constexpr double kAccelWeight = 0.08;

    const double severity =
        localClamp01(kSupportWeight * support_pressure +
                     kTiltWeight * tilt_pressure +
                     kBodyRateWeight * body_rate_pressure +
                     kTrustWeight * trust_pressure +
                     kContactWeight * contact_pressure +
                     kAccelWeight * accel_pressure);

    const double min_scale = low_speed_regime ? config_.low_speed_min_scale : config_.active_min_scale;
    const double cadence_min_scale =
        low_speed_regime ? config_.low_speed_cadence_min_scale : config_.active_cadence_min_scale;

    out.severity = severity;
    out.support_margin_m = support_margin;
    if (current_support == nullptr) {
        out.current_support_margin_m = support_margin;
        out.current_support_count = static_cast<int>(
            std::count(previous_gait.in_stance.begin(), previous_gait.in_stance.end(), true));
    }
    out.body_tilt_rad = body_tilt_rad;
    out.body_rate_radps = body_rate_radps;
    out.fusion_trust = fusion_trust;
    out.contact_mismatch_ratio = contact_mismatch;
    out.command_accel_mps2 = command_accel;
    out.recovery_collapse_headroom_m = releaseHeightHeadroomM(est, intent, safety_config_);

    out.command_scale = std::clamp(1.0 - severity * (1.0 - min_scale), min_scale, 1.0);
    out.cadence_scale = std::clamp(1.0 - severity * (1.0 - cadence_min_scale), cadence_min_scale, 1.0);
    out.swing_height_floor_m =
        gaitPresetSwingHeightFloor(intent.gait) + config_.swing_floor_boost_m * severity;

    const double squat_severity = std::clamp(
        (severity - config_.body_height_squat_severity_threshold) /
            std::max(1.0 - config_.body_height_squat_severity_threshold, 1e-6),
        0.0,
        1.0);
    const double body_height_delta_target_unrecov =
        -config_.body_height_squat_max_m * smoothStep01(squat_severity);

    const int sparse_support_limit = std::max(1, safety_config_.rapid_body_rate_max_contacts - 1);
    const bool support_sparse_now =
        current_support != nullptr &&
        current_support->support_count <= sparse_support_limit &&
        current_support->uncertain_support_count == 0;
    const bool emergency_dynamic_risk =
        body_tilt_rad >= 0.20 ||
        body_rate_radps >= 3.0;
    const bool support_margin_bad_now =
        current_support != nullptr &&
        current_support->static_margin_m <= 0.0;
    const bool previous_support_blocked = std::none_of(
        previous_gait.support_liftoff_safe_to_lift.begin(),
        previous_gait.support_liftoff_safe_to_lift.end(),
        [](const bool safe) { return safe; });
    const bool zero_margin_deadlock_now =
        current_support != nullptr &&
        current_support->static_margin_m <= 0.0 &&
        current_support->support_count == kNumLegs &&
        previous_support_blocked;
    const bool trigger_recovery_hold =
        current_support != nullptr &&
        emergency_dynamic_risk &&
        ((support_sparse_now && support_margin_bad_now) ||
         zero_margin_deadlock_now);

    if (trigger_recovery_hold) {
        enterRecoveryHold(now);
    }

    applyRecoveryStageState(now, out);

    // Body-height delta uses an asymmetric rule: protective squat (delta becoming more
    // negative) engages instantly to preserve the original fast protective response under
    // aggressive commands; release (delta returning toward 0) is slew-rate-limited to
    // ~0.04 m/s so the body smoothly rises after severity drops at gait transitions. This
    // is what makes the `fast tripod transition should reduce sag promptly` regression
    // assertion pass without sacrificing the protective squat ramp on the way down.
    const double body_height_target = suppressBodySquatForRecoveryStage(out.recovery_stage)
                                          ? 0.0
                                          : body_height_delta_target_unrecov;
    const double bh_slew_dt_s = advanceClock(body_height_delta_clock_, now);
    if (body_height_target <= body_height_delta_state_) {
        body_height_delta_state_ = body_height_target;
    } else {
        body_height_delta_state_ = slewToward(
            body_height_delta_state_, body_height_target, config_.body_height_delta_slew_mps, bh_slew_dt_s);
    }
    out.body_height_delta_m = body_height_delta_state_;

    scaleLocomotionIntent(intent, out.command_scale);
    intent.twist.body_trans_m.z = std::max(0.0, intent.twist.body_trans_m.z + out.body_height_delta_m);

    const PlanarMotionCommand governed_planar = planarMotionCommand(intent);
    out.governed_planar_speed_mps = std::hypot(governed_planar.vx_mps, governed_planar.vy_mps);
    out.governed_yaw_rate_radps = std::abs(governed_planar.yaw_rate_radps);
    out.governed_body_height_m = intent.twist.body_trans_m.z;
    out.saturated = out.saturated || severity > 1e-6 || std::abs(out.body_height_delta_m) > 1e-6;

    if (support_pressure > 0.01) {
        out.reasons |= CommandGovernorReason::LowSupportMargin;
    }
    if (tilt_pressure > 0.01) {
        out.reasons |= CommandGovernorReason::HighTilt;
    }
    if (body_rate_pressure > 0.01) {
        out.reasons |= CommandGovernorReason::HighBodyRate;
    }
    if (trust_pressure > 0.01) {
        out.reasons |= CommandGovernorReason::LowFusionTrust;
    }
    if (contact_pressure > 0.01) {
        out.reasons |= CommandGovernorReason::HighContactMismatch;
    }
    if (accel_pressure > 0.01) {
        out.reasons |= CommandGovernorReason::HighCommandAccel;
    }
    if (low_speed_regime) {
        out.reasons |= CommandGovernorReason::LowSpeedRegime;
    }

    return out;
}

CommandGovernorState CommandGovernor::finalizeRecovery(const RobotState& est,
                                                       const MotionIntent& intent,
                                                       const SafetyState& safety,
                                                       const GaitState& gait,
                                                       const SupportAssessment& current_support,
                                                       CommandGovernorState state,
                                                       TimePointUs now) {
    if (now.isZero()) {
        now = intent.timestamp_us.isZero() ? now_us() : intent.timestamp_us;
    }

    state.current_support_margin_m = current_support.static_margin_m;
    state.current_support_count = current_support.support_count;
    state.confirmed_support_count = current_support.confirmed_support_count;
    state.uncertain_support_count = current_support.uncertain_support_count;
    state.search_leg_count = current_support.search_leg_count;
    state.lost_candidate_count = current_support.lost_candidate_count;
    state.expected_touchdown_count = current_support.expected_touchdown_count;
    state.all_support_confirmed = current_support.all_support_confirmed;
    state.body_tilt_rad = bodyTiltMagnitudeRad(est);
    state.body_rate_radps = meanAbsBodyRateRadps(est);
    state.fusion_trust = est.has_fusion_diagnostics ? std::clamp(est.fusion.model_trust, 0.0, 1.0) : 1.0;
    state.contact_mismatch_ratio =
        est.has_fusion_diagnostics ? std::clamp(est.fusion.residuals.contact_mismatch_ratio, 0.0, 1.0) : 0.0;
    state.recovery_collapse_headroom_m = releaseHeightHeadroomM(est, intent, safety_config_);

    if (intent.requested_mode != RobotMode::WALK || safety.inhibit_motion || safety.torque_cut) {
        setRecoveryStage(RecoveryStage::None, now);
        applyRecoveryStageState(now, state);
        state.recovery_release_ready = false;
        return state;
    }

    // Filtered health margin: min over normalised tilt / body-rate / support / headroom
    // pressures (each in [0,1] where 1.0 is healthy and 0.0 is at the hard threshold),
    // EWMA-filtered. This single signal replaces the OR-of-instantaneous abort gate. With
    // hysteresis (entry vs abort thresholds), min-dwell, and persist-window, stride-period
    // spikes can no longer cycle the FSM during steady-state walking.
    const double release_body_rate_limit = releaseBodyRateLimit(config_, safety_config_);
    const double m_tilt =
        healthMarginFromPressure(state.body_tilt_rad, config_.tilt_soft_rad, config_.tilt_hard_rad, true);
    const double m_rate = healthMarginFromPressure(
        state.body_rate_radps, release_body_rate_limit, std::max(release_body_rate_limit, config_.body_rate_hard_radps), true);
    const double m_supp = healthMarginFromPressure(
        current_support.static_margin_m, config_.support_margin_soft_m, config_.support_margin_hard_m, false);
    const double headroom_band = std::max(config_.support_margin_soft_m - config_.support_margin_hard_m, 0.005);
    const double m_head = healthMarginFromPressure(
        state.recovery_collapse_headroom_m, 0.0, -headroom_band, false);
    const double instantaneous_health = std::min(std::min(m_tilt, m_rate), std::min(m_supp, m_head));
    const double health_dt_s = advanceClock(ramp_out_health_clock_, now);
    ramp_out_health_filtered_ = ewmaStep(
        ramp_out_health_filtered_, instantaneous_health, health_dt_s, config_.ramp_out_health_filter_tau_s);

    const bool healthy_dynamic_state =
        state.body_tilt_rad <= config_.tilt_soft_rad &&
        state.body_rate_radps <= release_body_rate_limit &&
        state.fusion_trust >= config_.fusion_trust_soft &&
        state.contact_mismatch_ratio <= config_.contact_mismatch_soft;
    const int sparse_support_limit = std::max(1, safety_config_.rapid_body_rate_max_contacts);
    const bool healthy_support_state =
        allLegsInStance(gait) &&
        current_support.support_count == kRecoveryReleaseSupportCount &&
        current_support.confirmed_support_count == kRecoveryReleaseSupportCount &&
        current_support.uncertain_support_count == 0 &&
        current_support.search_leg_count == 0 &&
        current_support.lost_candidate_count == 0 &&
        current_support.expected_touchdown_count == 0 &&
        current_support.all_support_confirmed &&
        current_support.static_margin_m >= config_.support_margin_soft_m &&
        state.recovery_collapse_headroom_m >= 0.0;
    const bool healthy_release_frame = healthy_dynamic_state && healthy_support_state;
    // Split the original degraded_settling_state into a support-side and a dynamic-side
    // check. Support changes (leg counts, margin, headroom) are reliable, low-noise discrete
    // signals — abort on them instantly. Dynamics (tilt, body-rate, trust, contact-mismatch)
    // are noisy at stride frequency — abort on them only if filtered+persisted (handled
    // below in the per-stage gate).
    const bool degraded_settling_support =
        current_support.support_count < kRecoveryReleaseSupportCount ||
        current_support.confirmed_support_count < kRecoveryReleaseSupportCount ||
        current_support.uncertain_support_count > 0 ||
        current_support.search_leg_count > 0 ||
        current_support.lost_candidate_count > 0 ||
        current_support.expected_touchdown_count > 0 ||
        current_support.static_margin_m < config_.support_margin_soft_m ||
        state.recovery_collapse_headroom_m < 0.0;
    const bool degraded_settling_dynamic = !healthy_dynamic_state;
    const bool degraded_settling_state = degraded_settling_support || degraded_settling_dynamic;
    (void)degraded_settling_state; // referenced by older callers; kept for documentation
    const bool ramp_out_sparse_support =
        current_support.support_count <= sparse_support_limit;
    const bool ramp_out_margin_bad =
        current_support.static_margin_m <= 0.0 || state.support_margin_m <= config_.support_margin_hard_m;
    const bool ramp_out_walk_safe_state =
        current_support.static_margin_m >= config_.support_margin_soft_m &&
        state.recovery_collapse_headroom_m >= 0.0 &&
        state.body_tilt_rad <= config_.tilt_soft_rad &&
        state.body_rate_radps <= release_body_rate_limit &&
        state.fusion_trust >= config_.fusion_trust_soft &&
        state.contact_mismatch_ratio <= config_.contact_mismatch_soft;

    // Hard-fall override: a true emergency (negative static margin AND sparse support AND
    // negative headroom) aborts immediately regardless of dwell or filter state. Genuine
    // tip-over conditions still respond in one tick.
    const bool hard_fall_now =
        ramp_out_sparse_support && ramp_out_margin_bad && state.recovery_collapse_headroom_m < 0.0;

    // Sustained-breach gate for RampOut and Settling: filtered health below abort threshold
    // for at least `ramp_out_abort_persist_s`, AND in RampOut, at least `ramp_out_min_dwell_s`
    // since the stage started. Together this rejects single-tick spikes (now filtered into
    // ramp_out_health_filtered_) and stride-period transients while still responding to
    // sustained instability within ~100-300 ms.
    const bool health_below_abort = ramp_out_health_filtered_ < config_.ramp_out_health_abort;
    if (health_below_abort) {
        if (ramp_out_breach_since_.isZero()) {
            ramp_out_breach_since_ = now;
        }
    } else {
        ramp_out_breach_since_ = TimePointUs{};
    }
    const bool breach_persisted =
        !ramp_out_breach_since_.isZero() &&
        static_cast<double>((now - ramp_out_breach_since_).value) * 1e-6 >= config_.ramp_out_abort_persist_s;
    const double ramp_out_elapsed_s =
        static_cast<double>((now - recovery_stage_started_).value) * 1e-6;
    const bool ramp_out_min_dwell_met = ramp_out_elapsed_s >= config_.ramp_out_min_dwell_s;

    state.recovery_release_ready = healthy_release_frame;

    switch (recovery_stage_) {
        case RecoveryStage::None:
            break;
        case RecoveryStage::ActiveHold:
            if (healthy_release_frame) {
                if (recovery_healthy_since_.isZero()) {
                    recovery_healthy_since_ = now;
                }
                if ((now - recovery_healthy_since_).value >= kRecoveryHealthyConfirmUs.value) {
                    setRecoveryStage(RecoveryStage::Settling, now);
                }
            } else {
                recovery_healthy_since_ = TimePointUs{};
            }
            break;
        case RecoveryStage::Settling:
            // Settling aborts instantly on support-side degradation (reliable discrete signal)
            // or hard fall, but only on dynamics breach if it persists past the filter window.
            if (hard_fall_now || degraded_settling_support ||
                (degraded_settling_dynamic && (breach_persisted || health_below_abort))) {
                enterRecoveryHold(now);
            } else if ((now - recovery_stage_started_).value >= kRecoverySettlingUs.value &&
                       ramp_out_health_filtered_ >= config_.ramp_out_health_entry) {
                setRecoveryStage(RecoveryStage::RampOut, now);
            }
            break;
        case RecoveryStage::RampOut: {
            // Support-side collapse during RampOut (sparse + bad margin, or negative headroom)
            // is a reliable signal — abort instantly. Dynamic noise (tilt/body-rate spikes
            // during stride transitions) only aborts if filtered breach persists past
            // ramp_out_abort_persist_s AND we have at least ramp_out_min_dwell_s in the stage.
            // This is the change that breaks the active_hold→settling→ramp_out cycle.
            const bool support_collapse_now =
                (ramp_out_sparse_support && ramp_out_margin_bad) ||
                state.recovery_collapse_headroom_m < 0.0;
            if (hard_fall_now || support_collapse_now ||
                (ramp_out_min_dwell_met && breach_persisted)) {
                enterRecoveryHold(now);
            } else if ((now - recovery_stage_started_).value >= kRecoveryRampOutUs.value &&
                       ramp_out_walk_safe_state &&
                       current_support.uncertain_support_count == 0) {
                setRecoveryStage(RecoveryStage::None, now);
            }
            break;
        }
    }

    applyRecoveryStageState(now, state);
    // Same asymmetric slew as in apply(): instant engage of squat, slew-limited release.
    const double body_height_target_finalize =
        suppressBodySquatForRecoveryStage(state.recovery_stage) ? 0.0 : body_height_delta_state_;
    const double bh_slew_dt_s = advanceClock(body_height_delta_clock_, now);
    if (body_height_target_finalize <= body_height_delta_state_) {
        body_height_delta_state_ = body_height_target_finalize;
    } else {
        body_height_delta_state_ = slewToward(
            body_height_delta_state_, body_height_target_finalize, config_.body_height_delta_slew_mps, bh_slew_dt_s);
    }
    state.body_height_delta_m = body_height_delta_state_;
    if (recovery_stage_ == RecoveryStage::None) {
        state.recovery_release_ready = false;
    }
    return state;
}
