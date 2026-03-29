#include "leg_ik.hpp"
#include "types.hpp"

#include <cmath>

#include "reach_envelope.hpp"

namespace {

constexpr double kTwoPi = 2.0 * kPi;

void apply_estimated_leg_fallback(const RobotState& est,
                                  JointTargets& joints,
                                  int legID)
{
  for (int joint = 0; joint < kJointsPerLeg; ++joint) {
    joints.leg_states[legID].joint_state[joint].pos_rad =
        est.leg_states[legID].joint_state[joint].pos_rad;
  }
}

double unwrapToNearest(double reference_rad, double candidate_rad) {
  const double turns = std::round((reference_rad - candidate_rad) / kTwoPi);
  return candidate_rad + (turns * kTwoPi);
}

} // namespace

LegIK::LegIK(HexapodGeometry geometry) : hexGeo(geometry) {}


JointTargets LegIK::solve(const RobotState& est,
                          const LegTargets& targets,
                          const SafetyState& safety) {
  JointTargets joints{};

  for (int legID = 0; legID < kNumLegs; legID++) {
    const LegState& unwrap_ref =
        unwrap_initialized_[static_cast<std::size_t>(legID)]
            ? last_unwrapped_command_.leg_states[legID]
            : est.leg_states[legID];

    const bool solved = solveOneLeg(unwrap_ref,
                                    joints.leg_states[legID],
                                    targets.feet[legID],
                                    hexGeo.legGeometry[legID]);
    const bool use_fallback = !solved || !safety.leg_enabled[legID];
    if (use_fallback) {
      apply_estimated_leg_fallback(est, joints, legID);
    }

    unwrap_initialized_[static_cast<std::size_t>(legID)] = true;
    last_unwrapped_command_.leg_states[legID] = joints.leg_states[legID];
  }

  return joints;
}

bool LegIK::solveOneLeg(const LegState& unwrap_reference,
                        LegState& out,
                        const FootTarget& foot,
                        const LegGeometry& leg) {
  // Transform foot target coordinates so they are relative to Coxa mount
  const Vec3 relativeToCoxa = foot.pos_body_m - leg.bodyCoxaOffset;
  const Mat3 R_leg = Mat3::rotZ(-leg.mountAngle.value);
  const Vec3 footLeg = R_leg * relativeToCoxa;

  const double x = footLeg.x;
  const double y = footLeg.y;

  // --------------------------------------------------------
  // 1) Coxa angle
  // --------------------------------------------------------
  const double q1 = std::atan2(y, x);
  
  // Clamp out-of-reach requests to the closest solvable configuration instead of
  // rejecting the whole leg update and snapping to estimator fallback angles.
  const Vec3 clamped_leg = kinematics::clampFootToReachEnvelope(footLeg, leg);
  const double rSolved = std::hypot(clamped_leg.x, clamped_leg.y);
  const double rhoSolved = rSolved - leg.coxaLength.value;
  const double zSolved = clamped_leg.z;

  // --------------------------------------------------------
  // 2) Tibia angle by law of cosines
  // --------------------------------------------------------
  const double D =
      (rhoSolved * rhoSolved + zSolved * zSolved - leg.femurLength.value * leg.femurLength.value -
       leg.tibiaLength.value * leg.tibiaLength.value) /
      (2.0 * leg.femurLength.value * leg.tibiaLength.value);

  const double Dclamped = clamp(D, -1.0, 1.0);

  // Choose one branch consistently.
  // Switch the sign of sqrt(...) if your knee bends the wrong way.
  const double q3 =
      std::atan2(-std::sqrt(std::max(0.0, 1.0 - Dclamped * Dclamped)), Dclamped);

  // --------------------------------------------------------
  // 3) Femur angle
  // --------------------------------------------------------
  const double q2 =
      std::atan2(zSolved, rhoSolved) -
      std::atan2(leg.tibiaLength.value * std::sin(q3),
                 leg.femurLength.value + leg.tibiaLength.value * std::cos(q3));

  out.joint_state[0].pos_rad = AngleRad{q1};
  out.joint_state[1].pos_rad = AngleRad{q2};
  out.joint_state[2].pos_rad = AngleRad{q3};
  out = leg.servo.toServoAngles(out);

  // Keep servo command continuity across +/-pi branch cuts. Unwrapping against
  // raw encoder feedback couples IK to measurement noise and causes hunting;
  // use the previous commanded solution once available (see solve()).
  for (int joint = 0; joint < kJointsPerLeg; ++joint) {
    out.joint_state[joint].pos_rad.value =
        unwrapToNearest(unwrap_reference.joint_state[joint].pos_rad.value,
                        out.joint_state[joint].pos_rad.value);
  }

  return true;
}
