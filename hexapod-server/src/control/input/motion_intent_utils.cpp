#include "motion_intent_utils.hpp"

MotionIntent makeMotionIntent(RobotMode mode, GaitType gait, double body_height_m) {
    MotionIntent cmd{};
    cmd.requested_mode = mode;
    cmd.gait = gait;
    cmd.speed_mps = LinearRateMps{0.0};
    cmd.heading_rad = AngleRad{0.0};
    cmd.body_pose_setpoint.orientation_rad = {0.0, 0.0, 0.0};
    cmd.body_pose_setpoint.body_trans_m = {0.0, 0.0, body_height_m};
    cmd.body_pose_setpoint.body_trans_mps = {0.0, 0.0, 0.0};
    cmd.timestamp_us = now_us();
    return cmd;
}

MotionIntent makeMotionIntent(const ScenarioMotionIntent& motion) {
    MotionIntent cmd = makeMotionIntent(motion.mode, motion.gait, motion.body_height_m);
    cmd.speed_mps = LinearRateMps{motion.speed_mps};
    cmd.heading_rad = AngleRad{motion.heading_rad};
    cmd.body_pose_setpoint.orientation_rad.z = motion.yaw_rad;
    return cmd;
}
