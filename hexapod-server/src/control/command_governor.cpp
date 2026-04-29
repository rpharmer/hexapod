#include "command_governor.hpp"

#include "motion_intent_utils.hpp"

#include <algorithm>
#include <cmath>

namespace {

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

} // namespace

CommandGovernor::CommandGovernor(control_config::CommandGovernorConfig config)
    : config_(config) {}

void CommandGovernor::reset() {
    last_clock_ = TimePointUs{};
    last_requested_planar_speed_mps_ = 0.0;
    last_requested_yaw_rate_radps_ = 0.0;
}

CommandGovernorState CommandGovernor::apply(const RobotState& est,
                                            MotionIntent& intent,
                                            const SafetyState& safety,
                                            const GaitState& previous_gait) {
    CommandGovernorState out{};
    const PlanarMotionCommand requested = planarMotionCommand(intent);
    out.requested_planar_speed_mps = std::hypot(requested.vx_mps, requested.vy_mps);
    out.requested_yaw_rate_radps = std::abs(requested.yaw_rate_radps);
    out.requested_body_height_m = intent.twist.body_trans_m.z;

    if (intent.requested_mode != RobotMode::WALK || safety.inhibit_motion || safety.torque_cut) {
        out.governed_planar_speed_mps = out.requested_planar_speed_mps;
        out.governed_yaw_rate_radps = out.requested_yaw_rate_radps;
        out.governed_body_height_m = out.requested_body_height_m;
        out.swing_height_floor_m = gaitPresetSwingHeightFloor(intent.gait);
        last_clock_ = intent.timestamp_us.isZero() ? now_us() : intent.timestamp_us;
        last_requested_planar_speed_mps_ = out.requested_planar_speed_mps;
        last_requested_yaw_rate_radps_ = out.requested_yaw_rate_radps;
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
    const double body_tilt_rad = (est.has_body_twist_state &&
                                  std::isfinite(est.body_twist_state.twist_pos_rad.x) &&
                                  std::isfinite(est.body_twist_state.twist_pos_rad.y))
                                     ? std::hypot(est.body_twist_state.twist_pos_rad.x,
                                                  est.body_twist_state.twist_pos_rad.y)
                                     : 0.0;
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
    out.body_tilt_rad = body_tilt_rad;
    out.body_rate_radps = body_rate_radps;
    out.fusion_trust = fusion_trust;
    out.contact_mismatch_ratio = contact_mismatch;
    out.command_accel_mps2 = command_accel;

    out.command_scale = std::clamp(1.0 - severity * (1.0 - min_scale), min_scale, 1.0);
    out.cadence_scale = std::clamp(1.0 - severity * (1.0 - cadence_min_scale), cadence_min_scale, 1.0);
    out.swing_height_floor_m =
        gaitPresetSwingHeightFloor(intent.gait) + config_.swing_floor_boost_m * severity;

    const double squat_severity = std::clamp(
        (severity - config_.body_height_squat_severity_threshold) /
            std::max(1.0 - config_.body_height_squat_severity_threshold, 1e-6),
        0.0,
        1.0);
    out.body_height_delta_m = -config_.body_height_squat_max_m * smoothStep01(squat_severity);

    scaleLocomotionIntent(intent, out.command_scale);
    intent.twist.body_trans_m.z = std::max(0.0, intent.twist.body_trans_m.z + out.body_height_delta_m);

    const PlanarMotionCommand governed_planar = planarMotionCommand(intent);
    out.governed_planar_speed_mps = std::hypot(governed_planar.vx_mps, governed_planar.vy_mps);
    out.governed_yaw_rate_radps = std::abs(governed_planar.yaw_rate_radps);
    out.governed_body_height_m = intent.twist.body_trans_m.z;
    out.saturated = severity > 1e-6 || std::abs(out.body_height_delta_m) > 1e-6;

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
