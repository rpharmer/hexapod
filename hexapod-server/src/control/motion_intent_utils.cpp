#include "motion_intent_utils.hpp"

#include <atomic>
#include <cmath>

namespace {

std::atomic<std::uint64_t> g_intent_stream_sample_id{2};

} // namespace

void stampIntentStreamMotionFields(MotionIntent& intent) {
    intent.timestamp_us = now_us();
    intent.sample_id = g_intent_stream_sample_id.fetch_add(1, std::memory_order_relaxed);
}

PlanarMotionCommand planarMotionCommand(const MotionIntent& intent) {
    double vx = intent.cmd_vx_mps.value;
    double vy = intent.cmd_vy_mps.value;
    double yaw = intent.cmd_yaw_radps.value;

    const bool explicit_cmd =
        (std::abs(vx) + std::abs(vy) + std::abs(yaw)) > 1e-12;
    if (!explicit_cmd) {
        vx = intent.speed_mps.value * std::cos(intent.heading_rad.value);
        vy = intent.speed_mps.value * std::sin(intent.heading_rad.value);
        yaw = intent.twist.twist_vel_radps.z;
    }
    return PlanarMotionCommand{vx, vy, yaw};
}

PlanarMotionCommand planarMotionFromCommandTwist(const BodyTwist& cmd) {
    return PlanarMotionCommand{cmd.linear_mps.x, cmd.linear_mps.y, cmd.angular_radps.z};
}

MotionIntent makeMotionIntent(RobotMode mode, GaitType gait, double body_height_m) {
    MotionIntent cmd{};
    cmd.requested_mode = mode;
    cmd.gait = gait;
    cmd.speed_mps = LinearRateMps{0.0};
    cmd.heading_rad = AngleRad{0.0};
    cmd.cmd_vx_mps = LinearRateMps{0.0};
    cmd.cmd_vy_mps = LinearRateMps{0.0};
    cmd.cmd_yaw_radps = AngularRateRadPerSec{0.0};
    cmd.twist.twist_pos_rad = {0.0, 0.0, 0.0};
    cmd.twist.body_trans_m = {0.0, 0.0, body_height_m};
    cmd.twist.body_trans_mps = {0.0, 0.0, 0.0};
    cmd.timestamp_us = now_us();
    return cmd;
}

MotionIntent makeMotionIntent(const ScenarioMotionIntent& motion) {
    MotionIntent cmd = makeMotionIntent(motion.mode, motion.gait, motion.body_height_m);
    cmd.speed_mps = LinearRateMps{motion.speed_mps};
    cmd.heading_rad = AngleRad{motion.heading_rad};
    double vx = motion.speed_mps * std::cos(motion.heading_rad);
    double vy = motion.speed_mps * std::sin(motion.heading_rad);
    if (motion.has_direct_velocity) {
        vx = motion.vx_mps;
        vy = motion.vy_mps;
    }
    cmd.cmd_vx_mps = LinearRateMps{vx};
    cmd.cmd_vy_mps = LinearRateMps{vy};
    cmd.cmd_yaw_radps = AngularRateRadPerSec{motion.yaw_rate_radps};
    cmd.twist.twist_pos_rad.z = motion.yaw_rad;
    return cmd;
}
