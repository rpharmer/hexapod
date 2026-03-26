#include "body_controller.hpp"

#include <algorithm>
#include <cmath>

#include "reach_envelope.hpp"

namespace {

constexpr double kDefaultBodyHeightM = 0.20;
constexpr double kContinuityPhaseBlendWidth = 0.25;

Vec3 lerp(const Vec3& a, const Vec3& b, double t) {
    return a + ((b - a) * t);
}

double smoothstep(double t) {
    const double alpha = clamp01(t);
    return alpha * alpha * (3.0 - (2.0 * alpha));
}

double phaseProgressForLeg(const GaitState& gait, int leg, const RuntimeGaitPolicy& policy) {
    const double phase = clamp01(gait.phase[leg]);
    const double duty = std::clamp(policy.per_leg[leg].duty_cycle, 0.05, 0.95);
    if (gait.in_stance[leg]) {
        return (duty <= 1e-6) ? 0.0 : clamp01(phase / duty);
    }
    return clamp01((phase - duty) / std::max(1.0 - duty, 1e-6));
}

struct LegContinuityState {
    bool initialized{false};
    bool in_stance{true};
    Vec3 transition_pos{};
    Vec3 transition_vel{};
    Vec3 last_pos{};
    Vec3 last_vel{};
};

std::array<LegContinuityState, kNumLegs> g_leg_continuity{};

} // namespace

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

    (void)est;

    const std::array<Vec3, kNumLegs> nominal = nominalStance();
    const bool walking =
        (intent.requested_mode == RobotMode::WALK) &&
        !safety.inhibit_motion &&
        !safety.torque_cut;

    const double roll_cmd = intent.twist.twist_pos_rad.x;
    const double pitch_cmd = intent.twist.twist_pos_rad.y;
    const double yaw_cmd = policy.suppression.suppress_turning ? 0.0 : intent.twist.twist_pos_rad.z;
    const Mat3 body_rotation = Mat3::rotZ(yaw_cmd) * Mat3::rotY(pitch_cmd) * Mat3::rotX(roll_cmd);
    const Vec3 planar_body_offset = Vec3{
        intent.twist.body_trans_m.x,
        intent.twist.body_trans_m.y,
        0.0};

    double commanded_body_height_m = intent.twist.body_trans_m.z;
    if (commanded_body_height_m <= 1e-6) {
        commanded_body_height_m = kDefaultBodyHeightM;
    }

    for (int leg = 0; leg < kNumLegs; ++leg) {
        Vec3 target = nominal[leg];

        target.z += commanded_body_height_m - kDefaultBodyHeightM;
        target = target - planar_body_offset;

        const PlannedFoothold foothold =
            foothold_planner_.plan(leg, target, intent, gait, policy, walking);

        // Apply planner trajectory first, then apply posture-bias as a continuous offset.
        const Vec3 planned_target = body_rotation * foothold.pos_body_m;
        const Vec3 planned_vel = body_rotation * foothold.vel_body_mps;

        LegContinuityState& continuity = g_leg_continuity[leg];
        if (!continuity.initialized) {
            continuity.initialized = true;
            continuity.in_stance = gait.in_stance[leg];
            continuity.transition_pos = planned_target;
            continuity.transition_vel = planned_vel;
            continuity.last_pos = planned_target;
            continuity.last_vel = planned_vel;
        } else if (continuity.in_stance != gait.in_stance[leg]) {
            continuity.in_stance = gait.in_stance[leg];
            continuity.transition_pos = continuity.last_pos;
            continuity.transition_vel = continuity.last_vel;
        }

        const double phase_progress = phaseProgressForLeg(gait, leg, policy);
        const double continuity_alpha = smoothstep(clamp01(phase_progress / kContinuityPhaseBlendWidth));

        target = lerp(continuity.transition_pos, planned_target, continuity_alpha);
        Vec3 target_vel = lerp(continuity.transition_vel, planned_vel, continuity_alpha);

        target = clampToReachEnvelope(leg, target);
        target_vel = target_vel + cross(intent.twist.twist_vel_radps, target);
        continuity.last_pos = target;
        continuity.last_vel = target_vel;

        out.feet[leg].pos_body_m = target;
        out.feet[leg].vel_body_mps = target_vel;
    }

    return out;
}
