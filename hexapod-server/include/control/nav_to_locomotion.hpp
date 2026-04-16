#pragma once

#include "nav_command.hpp"

struct MotionIntent;
struct RobotState;

/** World-frame planar pose from estimator / sim (`body_trans_m` XY + yaw from `twist_pos_rad.z`). */
NavPose2d navPose2dFromRobotState(const RobotState& est);

/** Writes `nav` into planar walk fields of `intent` and keeps polar `speed` / `heading` consistent. */
void applyNavCommandToMotionIntent(const NavCommand& nav, MotionIntent& intent);
