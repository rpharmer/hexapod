#include "leg_fk.hpp"

#include <cmath>
#include "geometry_config.hpp"
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
  const std::array<JointState, kJointsPerLeg> joints = est.joint_state;
  const double q1 = joints[0].pos_rad.value;
  const double q2 = joints[1].pos_rad.value;
  const double q3 = joints[2].pos_rad.value;

  // Effective reach of the femur+tibia chain in the leg plane
  const double rho =
      leg.femurLength.value * std::cos(q2) +
      leg.tibiaLength.value * std::cos(q2 + q3);

  // Vertical position in the leg plane
  const double z =
      leg.femurLength.value * std::sin(q2) +
      leg.tibiaLength.value * std::sin(q2 + q3);

  // Total radial distance from coxa axis to foot
  const double r = leg.coxaLength.value + rho;

  // Rotate radial distance by coxa yaw into x/y
  const double x = r * std::cos(q1);
  const double y = r * std::sin(q1);

  FootTarget foot{};
  foot.pos_body_m = {x, y, z};
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

  FootTarget footLeg{};

  // Foot in the leg's own local frame
  solveOneLeg(est, footLeg, leg);

  // Leg frame -> body frame
  const Mat3 R_body_from_leg = Mat3::rotZ(leg.mountAngle.value);
  const Vec3 footRelativeToCoxa = R_body_from_leg * footLeg.pos_body_m;

  // Shift from coxa origin to body origin
  const Vec3 footBody = leg.bodyCoxaOffset + footRelativeToCoxa;
  FootTarget out{};
  out.pos_body_m = footBody;

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
