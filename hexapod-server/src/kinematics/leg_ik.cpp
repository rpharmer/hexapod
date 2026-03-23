#include "leg_ik.hpp"
#include "types.hpp"

#include <cmath>

namespace {

void apply_estimated_leg_fallback(const EstimatedState& est,
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


JointTargets LegIK::solve(const EstimatedState& est,
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
  (void)est;

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

  if (d < minReach || d > maxReach) {
    return false;
  }

  // --------------------------------------------------------
  // 2) Tibia angle by law of cosines
  // --------------------------------------------------------
  const double D =
      (rho * rho + z * z - leg.femurLength.value * leg.femurLength.value -
       leg.tibiaLength.value * leg.tibiaLength.value) /
      (2.0 * leg.femurLength.value * leg.tibiaLength.value);

  const double Dclamped = clamp(D, -1.0, 1.0);
  if (std::fabs(Dclamped - D) > 1e-9) {
    return false;
  }

  // Choose one branch consistently.
  // Switch the sign of sqrt(...) if your knee bends the wrong way.
  const double q3 =
      std::atan2(-std::sqrt(std::max(0.0, 1.0 - Dclamped * Dclamped)), Dclamped);

  // --------------------------------------------------------
  // 3) Femur angle
  // --------------------------------------------------------
  const double q2 =
      std::atan2(z, rho) -
      std::atan2(leg.tibiaLength.value * std::sin(q3),
                 leg.femurLength.value + leg.tibiaLength.value * std::cos(q3));

  out.joint_state[0].pos_rad = AngleRad{q1};
  out.joint_state[1].pos_rad = AngleRad{q2};
  out.joint_state[2].pos_rad = AngleRad{q3};
  out = leg.servo.toServoAngles(out);

  return true;
}
