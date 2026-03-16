#include "leg_fk.hpp"

#include <cmath>

// ------------------------------------------------------------
// Forward kinematics in LEG frame
//
// Given joint angles:
//
//   q1 = coxa yaw
//   q2 = femur pitch
//   q3 = tibia pitch
//
// return the foot position in the leg's local frame.
//
// Geometry:
//
//   rho = L2*cos(q2) + L3*cos(q2 + q3)
//   z   = L2*sin(q2) + L3*sin(q2 + q3)
//   r   = L1 + rho
//
// Then rotate into XY by q1:
//
//   x = r*cos(q1)
//   y = r*sin(q1)
//
// This is the exact inverse of the IK convention already used
// in solveOneLeg(), assuming the same axis conventions.
// ------------------------------------------------------------

LegTargets LegFK::solve(const RawHardwareState& raw, const SafetyState& safety)
{
  (void)raw;
  (void)safety;
  
  LegTargets out{};
  return out;
}
                       
                       
bool LegFK::solveOneLeg(const LegRawState& est, FootTarget& out,
                        const LegGeometry& leg)
{
  const std::array<JointRawState, kJointsPerLeg> joints = est.joint_raw_state;
  const double q1 = joints[0].pos_rad;
  const double q2 = joints[1].pos_rad;
  const double q3 = joints[2].pos_rad;

  // Effective reach of the femur+tibia chain in the leg plane
  const double rho =
      leg.femurLength * std::cos(q2) +
      leg.tibiaLength * std::cos(q2 + q3);

  // Vertical position in the leg plane
  const double z =
      leg.femurLength * std::sin(q2) +
      leg.tibiaLength * std::sin(q2 + q3);

  // Total radial distance from coxa axis to foot
  const double r = leg.coxaLength + rho;

  // Rotate radial distance by coxa yaw into x/y
  const double x = r * std::cos(q1);
  const double y = r * std::sin(q1);

  FootTarget foot{};
  foot.pos_body_m = {x, y, z};
  out = foot;

  return true;
}

