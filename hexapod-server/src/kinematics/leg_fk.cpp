#include "leg_fk.hpp"

#include "geometry_config.hpp"
#include "leg_kinematics_utils.hpp"
#include "logger.hpp"

LegFK::LegFK() : hexGeo(defaultHexapodGeometry()) {}

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

LegTargets LegFK::solve(const RobotState& raw, const SafetyState& safety)
{
  (void)safety;
  
  LegTargets out{};
  
  for(int leg = 0; leg < kNumLegs; leg++)
  {
    LegState legState = raw.leg_states[leg];
    if(!solveOneLeg(legState, out.feet[leg], hexGeo.legGeometry[leg]))
    {
      if (auto logger = logging::GetDefaultLogger()) { LOG_ERROR(logger, "forward kinematics invalid"); }
    }
  }
  
  return out;
}
                       
                       
bool LegFK::solveOneLeg(const LegState& est, FootTarget& out,
                        const LegGeometry& leg)
{
  FootTarget foot{};
  foot.pos_body_m = kinematics::footInLegFrame(est, leg);
  out = foot;

  return true;
}

// ------------------------------------------------------------
// Forward kinematics in BODY frame
//
// Steps:
//   1) Compute foot in leg-local frame.
//   2) Rotate by +mountAngle into body orientation.
//   3) Add the coxa mount position in body frame.
// ------------------------------------------------------------
FootTarget LegFK::footInBodyFrame(const LegState& est, const LegGeometry& leg){
  FootTarget out{};
  out.pos_body_m = kinematics::footInBodyFrame(est, leg);

  return out;
}

// ------------------------------------------------------------
// Forward kinematics in WORLD frame
//
// Steps:
//   1) Compute foot in body frame.
//   2) Rotate BODY -> WORLD.
//   3) Add body world position.
// ------------------------------------------------------------
FootTarget LegFK::footInWorldFrame(const LegState& est, const BodyPose& bodyPose,
                             const LegGeometry& leg){
  const Vec3 footBody = footInBodyFrame(est, leg).pos_body_m;

  // BODY -> WORLD
  const Mat3 R_bw = bodyPose.rotationBodyToWorld();

  FootTarget out{};
  out.pos_body_m = bodyPose.position + (R_bw * footBody);
  
  return out;
}
