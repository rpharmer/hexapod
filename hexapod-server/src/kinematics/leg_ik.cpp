#include "leg_ik.hpp"
#include "types.hpp"

#include <cmath>

namespace {

double wrapAngleNear(const double reference, double candidate) {
  constexpr double kPi = 3.14159265358979323846;
  double adjusted = candidate;
  while (adjusted - reference > kPi) {
    adjusted -= 2.0 * kPi;
  }
  while (adjusted - reference < -kPi) {
    adjusted += 2.0 * kPi;
  }
  return adjusted;
}

void apply_estimated_leg_fallback(const RobotState& est,
                                  JointTargets& joints,
                                  int legID)
{
  for (int joint = 0; joint < kJointsPerLeg; ++joint) {
    joints.leg_states[legID].joint_state[joint].pos_rad =
        est.leg_states[legID].joint_state[joint].pos_rad;
  }
}

} // namespace

LegIK::LegIK(HexapodGeometry geometry) : hexGeo(geometry) {}


JointTargets LegIK::solve(const RobotState& est,
                          const LegTargets& targets,
                          const SafetyState& safety) {
  JointTargets joints{};

  for (int legID = 0; legID < kNumLegs; legID++) {
    const bool solved = solveOneLeg(est.leg_states[legID],
                                    joints.leg_states[legID],
                                    targets.feet[legID],
                                    hexGeo.legGeometry[legID]);
    const bool use_fallback = !solved || !safety.leg_enabled[legID];
    if (use_fallback) {
      apply_estimated_leg_fallback(est, joints, legID);
    }
  }

  return joints;
}

bool LegIK::solveOneLeg(const LegState& est,
                        LegState& out,
                        const FootTarget& foot,
                        const LegGeometry& leg) {
  // Transform foot target coordinates so they are relative to Coxa mount
  const Vec3 relativeToCoxa = foot.pos_body_m - leg.bodyCoxaOffset;
  const Mat3 R_leg = Mat3::rotZ(-leg.mountAngle.value);
  const Vec3 footLeg = R_leg * relativeToCoxa;

  const double x = footLeg.x;
  const double y = footLeg.y;
  const double z = footLeg.z;

  // --------------------------------------------------------
  // 1) Coxa angle
  // --------------------------------------------------------
  const double q1 = std::atan2(y, x);
  
  // Horizontal distance from coxa axis to foot
  const double r = std::hypot(x, y);
  
  // Effective horizontal distance for femur+tibia plane
  const double rho = r - leg.coxaLength.value;
  
  // 2D distance from femur joint to foot
  const double d = std::hypot(rho, z);

  const double minReach = std::fabs(leg.femurLength.value - leg.tibiaLength.value);
  const double maxReach = leg.femurLength.value + leg.tibiaLength.value;

  // Clamp out-of-reach requests to the closest solvable configuration instead of
  // rejecting the whole leg update and snapping to estimator fallback angles.
  const double clampedD = clamp(d, minReach, maxReach);
  const double reachScale = (d > 1e-9) ? (clampedD / d) : 1.0;
  const double rhoSolved = rho * reachScale;
  const double zSolved = z * reachScale;

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

  const LegState est_joint = leg.servo.toJointAngles(est);
  const double q1_wrapped = wrapAngleNear(est_joint.joint_state[0].pos_rad.value, q1);
  const double q2_wrapped = wrapAngleNear(est_joint.joint_state[1].pos_rad.value, q2);
  const double q3_wrapped = wrapAngleNear(est_joint.joint_state[2].pos_rad.value, q3);

  out.joint_state[0].pos_rad = AngleRad{q1_wrapped};
  out.joint_state[1].pos_rad = AngleRad{q2_wrapped};
  out.joint_state[2].pos_rad = AngleRad{q3_wrapped};
  out = leg.servo.toServoAngles(out);

  return true;
}
