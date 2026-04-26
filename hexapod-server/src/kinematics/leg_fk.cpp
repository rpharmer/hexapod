#include "leg_fk.hpp"

#include <cmath>
#include "geometry_config.hpp"
#include "logger.hpp"

LegFK::LegFK() : hexGeo(defaultHexapodGeometry()) {}

// ------------------------------------------------------------
// Forward kinematics in LEG frame
//
// Joint angles (joint space, after servo calibration in IK):
//   q1 = coxa (rotation about leg-local +X)
//   q2 = femur pitch
//   q3 = tibia pitch
//
// Leg-local frame: +X along the coxa link from anchor toward the femur joint; +Z up.
// Mount offsets and per-leg yaw of this frame into the body are defined in
// `types.hpp` (ASCII diagram + table: localYaxisAngle / bodyCoxaOffset).
//
// Femur+tibia chain in the x/z plane:
//   rho   = L2*cos(q2) + L3*cos(q2 + q3)
//   z_pl  = L2*sin(q2) + L3*sin(q2 + q3)
//   x     = L1 + rho
//   y,z   = rotate z_pl by q1 about +X: y = -sin(q1)*z_pl, z = cos(q1)*z_pl
//
// `footInBodyFrame` then applies R_body_from_leg = rotZ(mountAngle) and adds bodyCoxaOffset.
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

  // Effective reach of the femur+tibia chain in the leg x/z plane
  const double rho =
      leg.femurLength.value * std::cos(q2) +
      leg.tibiaLength.value * std::cos(q2 + q3);

  // Vertical displacement before coxa-axis rotation
  const double z_plane =
      leg.femurLength.value * std::sin(q2) +
      leg.tibiaLength.value * std::sin(q2 + q3);

  // Coxa rotation axis is leg-local +X (in body plane). Rotating around +X keeps x fixed
  // and spins the distal chain within y/z.
  const double x = leg.coxaLength.value + rho;
  const double y = -std::sin(q1) * z_plane;
  const double z = std::cos(q1) * z_plane;

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
