#pragma once

/**
 * Navigation vs locomotion (hexapod)
 *
 * Navigation produces only planar velocity intent in the body frame: `NavCommand`.
 * It must not reference gait phase, which leg swings, swing curves, footholds, or IK.
 *
 * Locomotion (existing pipeline: `MotionIntent` + `LocomotionCommandProcessor` + gait scheduler +
 * body controller + IK) consumes that intent and owns gait choice, twist field, foothold planning,
 * and swing trajectories. Use `applyNavCommandToMotionIntent` as the narrow seam between layers.
 *
 * With `RobotRuntime`, a navigation-capable tick order is: `busStep()` → `estimatorStep()` → build
 * intent from `NavLocomotionBridge::mergeIntent` using `estimatedSnapshot()` → `setMotionIntent` →
 * `safetyStep()` → `controlStep()` so pose reflects the latest sim/hardware read before commanding.
 */

/** Planar velocity command in body frame (+x forward, +y left); yaw rate about +Z (rad/s, CCW from above). */
struct NavCommand {
    double vx_mps{};
    double vy_mps{};
    double yaw_rate_radps{};
    /** Hint for higher-level braking / goal proximity (locomotion may ignore until profiles exist). */
    bool stop_at_goal{false};
};

/** Planar pose used by navigation primitives (world frame, +Z up; yaw rotates body x into world). */
struct NavPose2d {
    double x_m{};
    double y_m{};
    double yaw_rad{};
};
