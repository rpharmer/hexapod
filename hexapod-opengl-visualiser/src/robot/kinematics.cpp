#include "visualiser/robot/kinematics.hpp"

#include <cmath>

#include "visualiser/math/frame.hpp"
#include "visualiser/math/geometry.hpp"

namespace visualiser::robot {

using visualiser::math::Vec3;

Vec3 ServerToSceneVec(const Vec3& value) {
  return {value.x, value.z, -value.y};
}

Vec3 TransformBodyPoint(const Vec3& point, const HexapodBodyPoseState& pose) {
  if (!pose.valid) {
    return point;
  }
  const Vec3 scene_point = ServerToSceneVec(point);
  const Vec3 rotated = visualiser::math::RotateAroundSceneY(
      scene_point, -visualiser::math::frame_math::ServerYawToSceneYaw(pose.yaw_rad));
  const Vec3 origin = ServerToSceneVec(pose.position);
  return Vec3{origin.x + rotated.x, origin.y + rotated.y, origin.z + rotated.z};
}

RobotKinematics ComputeRobotLeg(const HexapodLegLayout& layout, const std::array<float, 3>& angles_deg) {
  RobotKinematics out{};
  const float coxa = (angles_deg[0] + layout.coxa_attach_deg) * visualiser::math::kPi / 180.0f;
  const float femur = (angles_deg[1] + layout.femur_attach_deg) * visualiser::math::kPi / 180.0f;
  const float tibia = (angles_deg[2] + layout.tibia_attach_deg) * visualiser::math::kPi / 180.0f;
  const float mount = layout.mount_angle_rad;

  out.coxa = layout.body_coxa_offset;
  const float hip_x = layout.coxa_mm * 0.001f * std::cos(mount + coxa);
  const float hip_y = layout.coxa_mm * 0.001f * std::sin(mount + coxa);
  out.femur = {out.coxa.x + hip_x, out.coxa.y + hip_y, out.coxa.z};

  const float femur_pitch = femur;
  const float knee_x = layout.femur_mm * 0.001f * std::cos(femur_pitch) * std::cos(mount + coxa);
  const float knee_y = layout.femur_mm * 0.001f * std::cos(femur_pitch) * std::sin(mount + coxa);
  const float knee_z = -layout.femur_mm * 0.001f * std::sin(femur_pitch);
  out.tibia = {out.femur.x + knee_x, out.femur.y + knee_y, out.femur.z + knee_z};

  const float ankle_pitch = femur + tibia;
  const float foot_x = layout.tibia_mm * 0.001f * std::cos(ankle_pitch) * std::cos(mount + coxa);
  const float foot_y = layout.tibia_mm * 0.001f * std::cos(ankle_pitch) * std::sin(mount + coxa);
  const float foot_z = -layout.tibia_mm * 0.001f * std::sin(ankle_pitch);
  out.foot = {out.tibia.x + foot_x, out.tibia.y + foot_y, out.tibia.z + foot_z};

  return out;
}

RobotKinematics ComputeRobotLegScene(const HexapodLegLayout& layout,
                                     const std::array<float, 3>& angles_deg,
                                     const HexapodBodyPoseState& body_pose) {
  RobotKinematics leg = ComputeRobotLeg(layout, angles_deg);
  leg.coxa = TransformBodyPoint(leg.coxa, body_pose);
  leg.femur = TransformBodyPoint(leg.femur, body_pose);
  leg.tibia = TransformBodyPoint(leg.tibia, body_pose);
  leg.foot = TransformBodyPoint(leg.foot, body_pose);
  return leg;
}

visualiser::scene::SceneBounds ComputeRobotBounds(const HexapodGeometryState& geometry,
                                                  const std::array<std::array<float, 3>, 6>& angles_deg,
                                                  const HexapodBodyPoseState& body_pose) {
  visualiser::scene::SceneBounds bounds{};
  for (std::size_t i = 0; i < geometry.legs.size(); ++i) {
    const RobotKinematics leg = ComputeRobotLegScene(geometry.legs[i], angles_deg[i], body_pose);
    visualiser::scene::ExpandBounds(bounds, leg.coxa);
    visualiser::scene::ExpandBounds(bounds, leg.femur);
    visualiser::scene::ExpandBounds(bounds, leg.tibia);
    visualiser::scene::ExpandBounds(bounds, leg.foot);
  }
  return bounds;
}

}  // namespace visualiser::robot
