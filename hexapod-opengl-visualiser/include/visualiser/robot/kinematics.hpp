#pragma once

#include <array>

#include "visualiser/robot/geometry_state.hpp"
#include "visualiser/scene/scene_bounds.hpp"

namespace visualiser::robot {

struct RobotKinematics {
  visualiser::math::Vec3 coxa{};
  visualiser::math::Vec3 femur{};
  visualiser::math::Vec3 tibia{};
  visualiser::math::Vec3 foot{};
};

visualiser::math::Vec3 ServerToSceneVec(const visualiser::math::Vec3& value);
visualiser::math::Vec3 TransformBodyPoint(const visualiser::math::Vec3& point, const HexapodBodyPoseState& pose);
RobotKinematics ComputeRobotLeg(const HexapodLegLayout& layout, const std::array<float, 3>& angles_deg);
RobotKinematics ComputeRobotLegScene(const HexapodLegLayout& layout,
                                     const std::array<float, 3>& angles_deg,
                                     const HexapodBodyPoseState& body_pose);
visualiser::scene::SceneBounds ComputeRobotBounds(const HexapodGeometryState& geometry,
                                                  const std::array<std::array<float, 3>, 6>& angles_deg,
                                                  const HexapodBodyPoseState& body_pose);

}  // namespace visualiser::robot
