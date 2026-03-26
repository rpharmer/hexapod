#pragma once

#include "types.hpp"

namespace kinematics {

Vec3 footInLegFrame(const LegState& joint_angles, const LegGeometry& geometry);
Vec3 footInBodyFrame(const LegState& joint_angles, const LegGeometry& geometry);
Point2 footPointInBodyPlane(const LegState& joint_angles, const LegGeometry& geometry);

}  // namespace kinematics
