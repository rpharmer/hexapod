#include "foothold_planner.hpp"

#include <algorithm>
#include <cmath>

namespace {

Vec3 normalizedPlanar(const Vec3& in, const Vec3& fallback) {
    const double mag = std::sqrt(in.x * in.x + in.y * in.y);
    if (mag <= 1e-9) {
        return fallback;
    }
    return Vec3{in.x / mag, in.y / mag, 0.0};
}

double clampPhaseRate(const GaitState& gait) {
    return std::max(0.0, gait.stride_phase_rate_hz.value);
}

} // namespace

FootholdPlanner::FootholdPlanner(HexapodGeometry geometry)
    : geometry_(geometry) {}

PlannedFoothold FootholdPlanner::plan(int leg,
                                      const Vec3& nominal_body,
                                      const MotionIntent& intent,
                                      const GaitState& gait,
                                      const RuntimeGaitPolicy& policy,
                                      bool walking) const {
    PlannedFoothold foothold{};
    foothold.pos_body_m = nominal_body;

    const Vec3 body_vel = Vec3{-intent.twist.body_trans_mps.x,
                               -intent.twist.body_trans_mps.y,
                               -intent.twist.body_trans_mps.z};
    foothold.vel_body_mps = body_vel;

    if (!walking) {
        return foothold;
    }

    const double phase = clamp01(gait.phase[leg]);
    const double phase_rate = clampPhaseRate(gait);
    const double duty = std::clamp(policy.per_leg[leg].duty_cycle, 0.05, 0.95);

    const Vec3 step_dir = modeStepDirection(leg, intent, policy);
    const double step_length_m =
        policy.per_leg[leg].step_length_m.value * innerOuterScale(leg, intent, policy);

    if (gait.in_stance[leg]) {
        const double planar_phase = 2.0 * kPi * phase;
        const double step_delta = 0.5 * step_length_m * std::cos(planar_phase);
        const double step_vel = -kPi * step_length_m * phase_rate * std::sin(planar_phase);
        foothold.pos_body_m = foothold.pos_body_m + (step_dir * step_delta);
        foothold.vel_body_mps = foothold.vel_body_mps + (step_dir * step_vel);
        return foothold;
    }

    const double swing_alpha = clamp01((phase - duty) / (1.0 - duty));
    const double alpha_rate = phase_rate / (1.0 - duty);

    const double liftoff_delta = 0.5 * step_length_m * std::cos(2.0 * kPi * duty);
    const Vec3 liftoff = nominal_body + (step_dir * liftoff_delta);

    const double stance_duration_s = duty / std::max(phase_rate, 1e-6);
    const Vec3 anticipatory = foothold.vel_body_mps * (0.5 * stance_duration_s);
    const Vec3 touchdown = nominal_body + (step_dir * (0.5 * step_length_m)) + anticipatory;

    const double blend = (3.0 * swing_alpha * swing_alpha) - (2.0 * swing_alpha * swing_alpha * swing_alpha);
    const double blend_rate = 6.0 * swing_alpha * (1.0 - swing_alpha) * alpha_rate;
    foothold.pos_body_m = liftoff + ((touchdown - liftoff) * blend);
    foothold.vel_body_mps = foothold.vel_body_mps + ((touchdown - liftoff) * blend_rate);

    const double swing_height = policy.per_leg[leg].swing_height_m.value;
    const double swing_lift = 0.5 * (1.0 - std::cos(2.0 * kPi * swing_alpha));
    const double swing_z_vel = 2.0 * kPi * alpha_rate * swing_height * std::sin(2.0 * kPi * swing_alpha);
    foothold.pos_body_m.z += swing_lift * swing_height;
    foothold.vel_body_mps.z += swing_z_vel;

    return foothold;
}

Vec3 FootholdPlanner::modeStepDirection(int leg,
                                        const MotionIntent& intent,
                                        const RuntimeGaitPolicy& policy) const {
    const double heading = intent.heading_rad.value;
    const Vec3 crab_dir{std::cos(heading), std::sin(heading), 0.0};

    if (policy.turn_mode != TurnMode::IN_PLACE) {
        return crab_dir;
    }

    const Vec3 tangent{-geometry_.legGeometry[leg].bodyCoxaOffset.y,
                       geometry_.legGeometry[leg].bodyCoxaOffset.x,
                       0.0};
    const double yaw_sign = (intent.twist.twist_vel_radps.z >= 0.0) ? 1.0 : -1.0;
    return normalizedPlanar(tangent * yaw_sign, crab_dir);
}

double FootholdPlanner::innerOuterScale(int leg,
                                        const MotionIntent& intent,
                                        const RuntimeGaitPolicy& policy) const {
    if (policy.suppression.suppress_turning) {
        return 1.0;
    }

    const double yaw_rate = intent.twist.twist_vel_radps.z;
    if (std::abs(yaw_rate) <= 1e-5) {
        return 1.0;
    }

    const bool is_left_leg = geometry_.legGeometry[leg].bodyCoxaOffset.y >= 0.0;
    const bool is_inner_leg = (yaw_rate > 0.0) ? is_left_leg : !is_left_leg;

    if (policy.turn_mode == TurnMode::IN_PLACE) {
        return is_inner_leg ? 0.65 : 1.35;
    }

    return is_inner_leg ? 0.85 : 1.15;
}
