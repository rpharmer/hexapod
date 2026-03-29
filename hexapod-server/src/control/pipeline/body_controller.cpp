#include "body_controller.hpp"

#include <cmath>

#include "reach_envelope.hpp"

namespace {

constexpr double kDefaultBodyHeightM = 0.20;
constexpr double kLevelHoldRollPitchKp = 0.6;
constexpr double kLevelHoldMaxCorrectionRad = 0.18;
constexpr double kLevelHoldMaxCommandRad = 0.30;
constexpr int kLevelHoldMinSupportContacts = 3;
constexpr double kLevelHoldMinStabilityMarginM = 0.01;
constexpr double kLevelHoldEstimateLpfAlpha = 0.25;

double finiteOr(double value, double fallback) {
    return std::isfinite(value) ? value : fallback;
}

Vec3 clampVectorMagnitude(const Vec3& value, double max_magnitude) {
    if (!std::isfinite(max_magnitude) || max_magnitude <= 0.0) {
        return Vec3{};
    }
    const double magnitude = vecNorm(value);
    if (magnitude <= max_magnitude || magnitude <= 1e-12) {
        return value;
    }
    return value * (max_magnitude / magnitude);
}

} // namespace

BodyController::BodyController(control_config::MotionLimiterConfig config)
    : limiter_config_(config) {}

std::array<Vec3, kNumLegs> BodyController::nominalStance() const {
    std::array<Vec3, kNumLegs> nominal{};
    for (int leg = 0; leg < kNumLegs; ++leg) {
        const LegGeometry& leg_geo = geometry_.legGeometry[leg];
        const double neutral_leg_x =
            leg_geo.coxaLength.value + 0.55 * (leg_geo.femurLength.value + leg_geo.tibiaLength.value);
        const Vec3 neutral_leg_frame{neutral_leg_x, 0.0, -kDefaultBodyHeightM};
        const Mat3 body_from_leg = Mat3::rotZ(leg_geo.mountAngle.value);
        nominal[leg] = leg_geo.bodyCoxaOffset + (body_from_leg * neutral_leg_frame);
    }
    return nominal;
}

Vec3 BodyController::clampToReachEnvelope(int leg, const Vec3& target_body) const {
    const LegGeometry& leg_geo = geometry_.legGeometry[leg];
    const Vec3 relative_to_coxa = target_body - leg_geo.bodyCoxaOffset;
    const Mat3 leg_from_body = Mat3::rotZ(-leg_geo.mountAngle.value);
    const Vec3 target_leg = leg_from_body * relative_to_coxa;
    const Vec3 clamped_leg = kinematics::clampFootToReachEnvelope(target_leg, leg_geo);
    const Mat3 body_from_leg = Mat3::rotZ(leg_geo.mountAngle.value);
    return leg_geo.bodyCoxaOffset + (body_from_leg * clamped_leg);
}


void BodyController::setYawCommandSlewEnabled(bool enabled) {
    (void)enabled;
}

LegTargets BodyController::update(const RobotState& est,
                                  const MotionIntent& intent,
                                  const GaitState& gait,
                                  const SafetyState& safety) {
    RuntimeGaitPolicy fallback_policy{};
    for (int leg = 0; leg < kNumLegs; ++leg) {
        fallback_policy.per_leg[leg].step_length_m = LengthM{0.06};
        fallback_policy.per_leg[leg].swing_height_m = LengthM{0.03};
        fallback_policy.per_leg[leg].duty_cycle = 0.5;
    }
    return update(est, intent, gait, fallback_policy, safety);
}

LegTargets BodyController::update(const RobotState& est,
                                  const MotionIntent& intent,
                                  const GaitState& gait,
                                  const RuntimeGaitPolicy& policy,
                                  const SafetyState& safety) {
    LegTargets out{};
    out.timestamp_us = now_us();
    MotionLimiterTelemetry limiter_telemetry{};

    const std::array<Vec3, kNumLegs> nominal = nominalStance();
    MotionIntent safe_intent = intent;
    safe_intent.body_pose_setpoint.body_trans_m = Vec3{
        finiteOr(intent.body_pose_setpoint.body_trans_m.x, 0.0),
        finiteOr(intent.body_pose_setpoint.body_trans_m.y, 0.0),
        finiteOr(intent.body_pose_setpoint.body_trans_m.z, kDefaultBodyHeightM)};
    safe_intent.body_pose_setpoint.body_trans_mps = Vec3{
        finiteOr(intent.body_pose_setpoint.body_trans_mps.x, 0.0),
        finiteOr(intent.body_pose_setpoint.body_trans_mps.y, 0.0),
        finiteOr(intent.body_pose_setpoint.body_trans_mps.z, 0.0)};
    safe_intent.body_pose_setpoint.angular_velocity_radps = Vec3{
        finiteOr(intent.body_pose_setpoint.angular_velocity_radps.x, 0.0),
        finiteOr(intent.body_pose_setpoint.angular_velocity_radps.y, 0.0),
        finiteOr(intent.body_pose_setpoint.angular_velocity_radps.z, 0.0)};
    safe_intent.body_pose_setpoint.orientation_rad = Vec3{
        finiteOr(intent.body_pose_setpoint.orientation_rad.x, 0.0),
        finiteOr(intent.body_pose_setpoint.orientation_rad.y, 0.0),
        finiteOr(intent.body_pose_setpoint.orientation_rad.z, 0.0)};

    const bool walking =
        (safe_intent.requested_mode == RobotMode::WALK) &&
        !safety.inhibit_motion &&
        !safety.torque_cut;

    const auto applyLevelHoldAxis = [](double setpoint_rad, double estimated_rad) {
        if (!std::isfinite(setpoint_rad) || !std::isfinite(estimated_rad)) {
            return setpoint_rad;
        }
        const double error_rad = setpoint_rad - estimated_rad;
        const double correction_rad = std::clamp(
            kLevelHoldRollPitchKp * error_rad,
            -kLevelHoldMaxCorrectionRad,
            kLevelHoldMaxCorrectionRad);
        return std::clamp(
            setpoint_rad + correction_rad,
            -kLevelHoldMaxCommandRad,
            kLevelHoldMaxCommandRad);
    };

    const double roll_setpoint = safe_intent.body_pose_setpoint.orientation_rad.x;
    const double pitch_setpoint = safe_intent.body_pose_setpoint.orientation_rad.y;
    const bool support_quality_sufficient =
        (safety.support_contact_count >= kLevelHoldMinSupportContacts) &&
        (safety.stability_margin_m >= kLevelHoldMinStabilityMarginM);
    const bool level_hold_enabled =
        est.has_measured_body_pose_state ||
        (est.has_inferred_body_pose_state && support_quality_sufficient);

    const auto updateLevelHoldEstimate = [&](double roll_est, double pitch_est) {
        if (!std::isfinite(roll_est) || !std::isfinite(pitch_est)) {
            return std::array<double, 2>{roll_est, pitch_est};
        }

        if (!level_hold_filter_state_.initialized) {
            level_hold_filter_state_.roll_pitch_rad = {roll_est, pitch_est};
            level_hold_filter_state_.initialized = true;
            return level_hold_filter_state_.roll_pitch_rad;
        }

        level_hold_filter_state_.roll_pitch_rad[0] +=
            kLevelHoldEstimateLpfAlpha * (roll_est - level_hold_filter_state_.roll_pitch_rad[0]);
        level_hold_filter_state_.roll_pitch_rad[1] +=
            kLevelHoldEstimateLpfAlpha * (pitch_est - level_hold_filter_state_.roll_pitch_rad[1]);
        return level_hold_filter_state_.roll_pitch_rad;
    };
    const auto filtered_roll_pitch = updateLevelHoldEstimate(
        finiteOr(est.body_pose_state.orientation_rad.x, 0.0),
        finiteOr(est.body_pose_state.orientation_rad.y, 0.0));
    const double roll_cmd = level_hold_enabled
                                ? applyLevelHoldAxis(roll_setpoint, filtered_roll_pitch[0])
                                : roll_setpoint;
    const double pitch_cmd = level_hold_enabled
                                 ? applyLevelHoldAxis(pitch_setpoint, filtered_roll_pitch[1])
                                 : pitch_setpoint;
    const double yaw_cmd_raw = policy.suppression.suppress_turning ? 0.0 : safe_intent.body_pose_setpoint.orientation_rad.z;
    const double yaw_cmd = yaw_cmd_raw;
    const Mat3 body_rotation = Mat3::rotZ(yaw_cmd) * Mat3::rotY(pitch_cmd) * Mat3::rotX(roll_cmd);
    const Vec3 planar_body_offset = Vec3{
        safe_intent.body_pose_setpoint.body_trans_m.x,
        safe_intent.body_pose_setpoint.body_trans_m.y,
        0.0};

    double commanded_body_height_m = safe_intent.body_pose_setpoint.body_trans_m.z;
    if (commanded_body_height_m <= 1e-6) {
        commanded_body_height_m = kDefaultBodyHeightM;
    }

    for (int leg = 0; leg < kNumLegs; ++leg) {
        Vec3 target = nominal[leg];

        target.z += commanded_body_height_m - kDefaultBodyHeightM;
        target = target - planar_body_offset;

        const PlannedFoothold foothold =
            foothold_planner_.plan(leg, target, safe_intent, gait, policy, walking);
        target = body_rotation * foothold.pos_body_m;
        Vec3 target_vel = body_rotation * foothold.vel_body_mps;

        const Vec3 unclamped_reach_target = target;
        target = clampToReachEnvelope(leg, target);
        if (vecNorm(target - unclamped_reach_target) > 1e-9) {
            limiter_telemetry.hard_clamp_reach = true;
        }
        target_vel = target_vel + cross(safe_intent.body_pose_setpoint.angular_velocity_radps, target);

        const double velocity_limit_mps = std::max(1e-3, limiter_config_.foot_velocity_limit_mps);
        if (has_previous_targets_ &&
            !previous_targets_timestamp_.isZero() &&
            out.timestamp_us.value > previous_targets_timestamp_.value) {
            const double dt_s = static_cast<double>(out.timestamp_us.value - previous_targets_timestamp_.value) / 1'000'000.0;
            const double max_step_m = velocity_limit_mps * dt_s;
            const Vec3 previous = previous_targets_body_[leg];
            const Vec3 delta = target - previous;
            const double delta_norm = vecNorm(delta);
            if (delta_norm > max_step_m && delta_norm > 1e-12) {
                target = previous + (delta * (max_step_m / delta_norm));
                limiter_telemetry.hard_clamp_reach = true;
            }
        }
        target_vel = clampVectorMagnitude(target_vel, velocity_limit_mps);

        out.feet[leg].pos_body_m = target;
        out.feet[leg].vel_body_mps = target_vel;
        previous_targets_body_[leg] = target;
    }

    previous_targets_timestamp_ = out.timestamp_us;
    has_previous_targets_ = true;
    last_motion_limiter_telemetry_ = limiter_telemetry;
    return out;
}
