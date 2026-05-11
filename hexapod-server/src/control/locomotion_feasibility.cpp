#include "locomotion_feasibility.hpp"

#include "motion_intent_utils.hpp"

#include <algorithm>
#include <cmath>

namespace {

double bodyTiltMagnitudeRad(const RobotState& est) {
    if (!est.has_body_twist_state ||
        !std::isfinite(est.body_twist_state.twist_pos_rad.x) ||
        !std::isfinite(est.body_twist_state.twist_pos_rad.y)) {
        return 0.0;
    }
    return std::hypot(est.body_twist_state.twist_pos_rad.x, est.body_twist_state.twist_pos_rad.y);
}

double planarBodyRateRadps(const RobotState& est) {
    if (!est.has_imu || !est.imu.valid) {
        return 0.0;
    }
    return std::hypot(est.imu.gyro_radps.x, est.imu.gyro_radps.y);
}

double selectControlMargin(const SupportAssessment& support,
                           const control_config::ControlMarginSource source) {
    switch (source) {
        case control_config::ControlMarginSource::Nominal:
            return support.nominal_static_margin_m;
        case control_config::ControlMarginSource::Actual:
            return support.actual_static_margin_m;
        case control_config::ControlMarginSource::Conservative:
            return std::min(support.nominal_static_margin_m, support.actual_static_margin_m);
    }
    return support.nominal_static_margin_m;
}

} // namespace

const char* legContactModeName(const LegContactMode mode) {
    switch (mode) {
        case LegContactMode::PlannedStance:
            return "planned_stance";
        case LegContactMode::PlannedSwing:
            return "planned_swing";
        case LegContactMode::HeldStance:
            return "held_stance";
        case LegContactMode::ContactGrace:
            return "contact_grace";
        case LegContactMode::LostCandidate:
            return "lost_candidate";
        case LegContactMode::RecoveryTouchdown:
            return "recovery_touchdown";
        case LegContactMode::Disabled:
            return "disabled";
    }
    return "unknown";
}

const char* controlMarginSourceName(const control_config::ControlMarginSource source) {
    switch (source) {
        case control_config::ControlMarginSource::Nominal:
            return "nominal";
        case control_config::ControlMarginSource::Actual:
            return "actual";
        case control_config::ControlMarginSource::Conservative:
            return "conservative";
    }
    return "unknown";
}

std::array<LegContactDecision, kNumLegs> computeLegContactDecisions(const RobotState& est,
                                                                    const GaitState& gait,
                                                                    const SafetyState& safety) {
    std::array<LegContactDecision, kNumLegs> decisions{};
    const double duty = std::clamp(gait.duty_factor, 0.06, 0.94);
    const double swing_span = std::max(1.0 - duty, 1e-6);

    for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
        LegContactDecision& out = decisions[leg];
        out.phase = clamp01(gait.phase[leg]);
        out.planned_stance = out.phase < duty;
        out.raw_contact = est.foot_contacts[leg];
        out.fused_phase = est.foot_contact_fusion[leg].phase;
        out.fused_confidence = est.foot_contact_fusion[leg].confidence;
        out.stability_hold = gait.stability_hold_stance[leg];

        const bool held_support =
            out.stability_hold &&
            (out.raw_contact || out.fused_phase == ContactPhase::LostCandidate);
        out.effective_stance = out.planned_stance || held_support;
        const bool lost_candidate_grace =
            out.effective_stance && out.fused_phase == ContactPhase::LostCandidate;
        const bool recovery_touchdown =
            gait.stride_phase_rate_hz.value <= 1e-6 &&
            !out.effective_stance &&
            !out.raw_contact &&
            out.fused_phase != ContactPhase::LostCandidate;
        out.swing_tau = out.effective_stance ? 0.0 : clamp01((out.phase - duty) / swing_span);
        const bool liftoff_grace =
            !out.effective_stance &&
            out.raw_contact &&
            out.swing_tau < 0.45;

        if (!safety.leg_enabled[leg]) {
            out.mode = LegContactMode::Disabled;
            out.use_stance_kinematics = true;
        } else if (lost_candidate_grace) {
            out.mode = LegContactMode::LostCandidate;
            out.use_stance_kinematics = true;
        } else if (held_support) {
            out.mode = LegContactMode::HeldStance;
            out.use_stance_kinematics = true;
        } else if (recovery_touchdown) {
            out.mode = LegContactMode::RecoveryTouchdown;
            out.use_stance_kinematics = false;
        } else if (liftoff_grace) {
            out.mode = LegContactMode::ContactGrace;
            out.use_stance_kinematics = false;
        } else if (out.planned_stance) {
            out.mode = LegContactMode::PlannedStance;
            out.use_stance_kinematics = true;
        } else {
            out.mode = LegContactMode::PlannedSwing;
            out.use_stance_kinematics = out.raw_contact;
        }
    }

    return decisions;
}

LocomotionFeasibility computeLocomotionFeasibility(const RobotState& est,
                                                   const MotionIntent& intent,
                                                   const GaitState& gait,
                                                   const GaitState& previous_gait,
                                                   const SafetyState& safety,
                                                   const HexapodGeometry& geometry,
                                                   const control_config::LocomotionRedesignConfig& config) {
    LocomotionFeasibility out{};
    out.valid = true;
    out.enabled = config.emit_telemetry ||
                  config.enable_feasibility_recovery ||
                  config.enable_feasibility_lift_gating ||
                  config.enable_contact_mode_planning ||
                  config.enable_height_policy;
    out.control_margin_source = config.control_margin_source;
    out.support = assessSupportState(est, intent, gait, geometry);
    out.contact = computeLegContactDecisions(est, gait, safety);
    out.nominal_margin_m = out.support.nominal_static_margin_m;
    out.actual_margin_m = out.support.actual_static_margin_m;
    out.control_margin_m = selectControlMargin(out.support, config.control_margin_source);
    out.body_tilt_rad = bodyTiltMagnitudeRad(est);
    out.body_rate_radps = planarBodyRateRadps(est);

    const PlanarMotionCommand cmd = planarMotionCommand(intent);
    out.high_demand =
        std::hypot(cmd.vx_mps, cmd.vy_mps) >= config.high_demand_planar_mps ||
        std::abs(cmd.yaw_rate_radps) >= config.high_demand_yaw_radps;
    out.dynamic_risk =
        out.body_tilt_rad >= config.emergency_tilt_rad ||
        out.body_rate_radps >= config.emergency_body_rate_radps;

    double margin_need = config.min_liftoff_margin_m;
    margin_need += std::clamp(0.04 * out.body_tilt_rad, 0.0, 0.05);
    if (gait.stride_phase_rate_hz.value < 0.78) {
        margin_need *= 0.85;
    }

    bool any_safe_to_lift = false;
    for (int leg = 0; leg < kNumLegs; ++leg) {
        const std::size_t idx = static_cast<std::size_t>(leg);
        out.lift_clearance_m[idx] =
            supportPolygonClearanceExcludingLeg(out.support, leg, margin_need, config.support_inset_m);
        out.safe_to_lift[idx] =
            safety.leg_enabled[idx] &&
            out.lift_clearance_m[idx] >= 0.0 &&
            out.body_tilt_rad < config.emergency_tilt_rad;
        any_safe_to_lift = any_safe_to_lift || out.safe_to_lift[idx];
    }

    out.sparse_support = out.support.support_count <= 2 && out.support.uncertain_support_count == 0;
    const bool all_held = std::all_of(
        gait.stability_hold_stance.begin(), gait.stability_hold_stance.end(), [](const bool held) {
            return held;
        });
    out.deadlocked = !any_safe_to_lift && all_held;

    const bool previous_support_blocked = std::none_of(
        previous_gait.support_liftoff_safe_to_lift.begin(),
        previous_gait.support_liftoff_safe_to_lift.end(),
        [](const bool safe) { return safe; });
    const bool zero_margin_deadlock =
        out.support.support_count == kNumLegs &&
        out.control_margin_m <= 0.0 &&
        previous_support_blocked;
    out.recovery_recommended =
        intent.requested_mode == RobotMode::WALK &&
        out.high_demand &&
        out.dynamic_risk &&
        ((out.sparse_support && out.control_margin_m <= 0.0) || zero_margin_deadlock || out.deadlocked);

    return out;
}

HeightPolicySnapshot evaluateHeightPolicySnapshot(const RobotState& est,
                                                  const MotionIntent& intent,
                                                  const GaitState& gait,
                                                  const CommandGovernorState& governor,
                                                  const control_config::LocomotionRedesignConfig& config) {
    HeightPolicySnapshot out{};
    out.valid = true;
    out.enabled = config.enable_height_policy;
    out.commanded_body_height_m = governor.requested_body_height_m;
    if (out.commanded_body_height_m <= 0.0) {
        out.commanded_body_height_m = intent.twist.body_trans_m.z;
    }
    out.governor_delta_m = governor.body_height_delta_m;
    if (est.has_body_twist_state &&
        std::isfinite(est.body_twist_state.body_trans_m.z)) {
        out.measured_body_height_m = est.body_twist_state.body_trans_m.z;
        out.compliance_sag_m = std::max(0.0, out.commanded_body_height_m - out.measured_body_height_m);
    }
    if (est.has_body_twist_state &&
        std::isfinite(est.body_twist_state.twist_pos_rad.x) &&
        std::isfinite(est.body_twist_state.twist_pos_rad.y)) {
        const double tilt_mag = std::hypot(
            est.body_twist_state.twist_pos_rad.x,
            est.body_twist_state.twist_pos_rad.y);
        out.tilt_squat_request_m = std::clamp(0.12 * std::max(0.0, tilt_mag - 0.08), 0.0, 0.06);
    }
    const double support_deficit_m = std::max(0.0, config.min_liftoff_margin_m - gait.static_stability_margin_m);
    out.swing_clearance_request_m = std::clamp(out.compliance_sag_m + 0.5 * support_deficit_m, 0.0, 0.06);
    out.terrain_stance_request_m = 0.0;
    out.policy_body_height_m =
        out.commanded_body_height_m + out.governor_delta_m + out.compliance_sag_m - out.tilt_squat_request_m;
    return out;
}
