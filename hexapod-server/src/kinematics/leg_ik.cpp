#include "leg_ik.hpp"
#include "types.hpp"

#include <cmath>

namespace {

struct IKCandidate {
  double q1{0.0};
  double q2{0.0};
  double q3{0.0};
  double knee_height{0.0};
};

IKCandidate buildCandidate(const double q1,
                           const double Dclamped,
                           const double s3,
                           const double rhoSolved,
                           const double zSolved,
                           const LegGeometry& leg)
{
  IKCandidate candidate{};
  candidate.q1 = q1;
  candidate.q3 = std::atan2(s3, Dclamped);
  candidate.q2 =
      std::atan2(zSolved, rhoSolved) -
      std::atan2(leg.tibiaLength.value * std::sin(candidate.q3),
                 leg.femurLength.value + leg.tibiaLength.value * std::cos(candidate.q3));
  // Compare femur joint height in the leg frame. The higher knee is the
  // physical knee-up branch, and servo sign mirroring is applied after this
  // joint-space choice is made.
  candidate.knee_height = leg.femurLength.value * std::sin(candidate.q2);
  return candidate;
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
    const bool solved = solveOneLeg(joints.leg_states[legID],
                                    targets.feet[legID],
                                    hexGeo.legGeometry[legID]);
    const bool use_fallback = !solved || !safety.leg_enabled[legID];
    if (use_fallback) {
      apply_estimated_leg_fallback(est, joints, legID);
    }
  }

  return joints;
}

bool LegIK::solveOneLeg(LegState& out,
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

  // The two sqrt(1 - D^2) signs are the two valid knee configurations. Choose the branch
  // with the higher femur joint, which is the physical knee-up solution.
  const double s3abs = std::sqrt(std::max(0.0, 1.0 - Dclamped * Dclamped));
  const IKCandidate branch_a =
      buildCandidate(q1, Dclamped, -s3abs, rhoSolved, zSolved, leg);
  const IKCandidate branch_b =
      buildCandidate(q1, Dclamped, +s3abs, rhoSolved, zSolved, leg);

  const IKCandidate& best = (branch_a.knee_height >= branch_b.knee_height) ? branch_a : branch_b;

  out.joint_state[0].pos_rad = AngleRad{best.q1};
  out.joint_state[1].pos_rad = AngleRad{best.q2};
  out.joint_state[2].pos_rad = AngleRad{best.q3};
  out = leg.servo.toServoAngles(out);

  return true;
}
