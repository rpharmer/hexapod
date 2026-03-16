#include "leg_ik.hpp"
#include "types.hpp"

#include <cmath>

LegIK::LegIK() {
  const double coxa_len = 0.043;
  const double femur_len = 0.060;
  const double tibia_len = 0.104;

  constexpr std::array<LegID, kNumLegs> leg_ids{
      LegID::R3, LegID::L3, LegID::R2, LegID::L2, LegID::R1, LegID::L1};

  const std::array<double, kNumLegs> mount_angles_deg{
      143.0, 217.0, 90.0, 270.0, 37.0, 323.0};

  constexpr std::array<Vec3, kNumLegs> coxa_offsets{{
    {0.063,   -0.0835, -0.007},
    {-0.063,  -0.0835, -0.007},
    { 0.0815,       0, -0.007},
    {-0.0815,       0, -0.007},
    { 0.063,   0.0835, -0.007},
    {-0.063,   0.0835, -0.007}
  }};

  constexpr double coxa_attach_deg = 0;
  constexpr std::array<double, kNumLegs> femur_attach_deg{35.0, -35.0, 35.0,
                                                           -35.0, 35.0, -35.0};
  constexpr std::array<double, kNumLegs> tibia_attach_deg{83.0, -83.0, 83.0,
                                                           -83.0, 83.0, -83.0};

  constexpr std::array<double, kNumLegs> side_sign{1.0, -1.0, 1.0, -1.0, 1.0,
                                                    -1.0};

  for (int leg = 0; leg < kNumLegs; ++leg) {
    auto& leg_geo = hexGeo.legGeometry[leg];
    leg_geo.legID = leg_ids[leg];
    leg_geo.bodyCoxaOffset = coxa_offsets[leg];
    leg_geo.mountAngle = deg2rad(mount_angles_deg[leg]);
    leg_geo.coxaLength = coxa_len;
    leg_geo.femurLength = femur_len;
    leg_geo.tibiaLength = tibia_len;

    leg_geo.servo.coxaOffset = deg2rad(coxa_attach_deg);
    leg_geo.servo.femurOffset = deg2rad(femur_attach_deg[leg]);
    leg_geo.servo.tibiaOffset = deg2rad(tibia_attach_deg[leg]);
    leg_geo.servo.coxaSign = side_sign[leg];
    leg_geo.servo.femurSign = side_sign[leg];
    leg_geo.servo.tibiaSign = side_sign[leg];
  }

  hexGeo.toBottom = 0.040;
}

JointTargets LegIK::solve(const EstimatedState& est,
                          const LegTargets& targets,
                          const SafetyState& safety) {
  JointTargets joints{};

  for (int legID = 0; legID < kNumLegs; legID++) {
    const bool solved = solveOneLeg(est.leg_states[legID],
                                    joints.leg_raw_states[legID],
                                    targets.feet[legID],
                                    hexGeo.legGeometry[legID]);
    if (!solved) {
      joints.leg_raw_states[legID] = est.leg_states[legID];
    }
    if (!safety.leg_enabled[legID]) {
      joints.leg_raw_states[legID] = est.leg_states[legID];
    }
  }

  return joints;
}

bool LegIK::solveOneLeg(const LegRawState& est,
                        LegRawState& out,
                        const FootTarget& foot,
                        const LegGeometry& leg) {
  (void)est;

  // Transform foot target coordinates so they are relative to Coxa mount
  const Vec3 relativeToCoxa = foot.pos_body_m - leg.bodyCoxaOffset;
  const Mat3 R_leg = Mat3::rotZ(-leg.mountAngle);
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
  const double rho = r - leg.coxaLength;
  
  // 2D distance from femur joint to foot
  const double d = std::hypot(rho, z);

  const double minReach = std::fabs(leg.femurLength - leg.tibiaLength);
  const double maxReach = leg.femurLength + leg.tibiaLength;

  if (d < minReach || d > maxReach) {
    return false;
  }

  // --------------------------------------------------------
  // 2) Tibia angle by law of cosines
  // --------------------------------------------------------
  const double D =
      (rho * rho + z * z - leg.femurLength * leg.femurLength -
       leg.tibiaLength * leg.tibiaLength) /
      (2.0 * leg.femurLength * leg.tibiaLength);

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
      std::atan2(leg.tibiaLength * std::sin(q3),
                 leg.femurLength + leg.tibiaLength * std::cos(q3));

  out.joint_raw_state[0].pos_rad = q1;
  out.joint_raw_state[1].pos_rad = q2;
  out.joint_raw_state[2].pos_rad = q3;
  out = leg.servo.toServoAngles(out);

  return true;
}
