#include "body_controller.hpp"

#include <algorithm>
#include <cmath>

LegTargets BodyController::update(const EstimatedState& est,
                                  const MotionIntent& intent,
                                  const GaitState& gait,
                                  const SafetyState& safety) {
    LegTargets out{};
    out.timestamp_us = now_us();

  // suppress warning
  (void)est;
  (void)intent;
  (void)gait;
  (void)safety;

    /*const bool allow_motion =
        !safety.inhibit_motion &&
        !safety.torque_cut &&
        intent.requested_mode == RobotMode::WALK;

    for (int leg = 0; leg < kNumLegs; ++leg) {
        Vec3 foot = nominal_stance_[leg];
        foot.z = -intent.twist.body_height_m;

        if (!allow_motion) {
            out.feet[leg].pos_body_m = foot;
            out.feet[leg].vel_body_mps = {};
            continue;
        }

        const double p = gait.phase[leg];
        const bool stance = gait.in_stance[leg];

        const double step_length = std::clamp(std::abs(intent.twist.vx_mps) * 0.12, 0.02, 0.08);
        const double step_width = std::clamp(std::abs(intent.twist.vy_mps) * 0.10, 0.00, 0.05);
        const double step_height = 0.035;

        if (stance) {
            // Move foot backward in body frame during stance to emulate body moving forward.
            const double u = p / 0.5; // 0..1
            foot.x += (0.5 - u) * step_length * ((intent.twist.vx_mps >= 0.0) ? 1.0 : -1.0);
            foot.y += (0.5 - u) * step_width * ((intent.twist.vy_mps >= 0.0) ? 1.0 : -1.0);
        } else {
            // Swing: move forward and lift.
            const double u = (p - 0.5) / 0.5; // 0..1
            foot.x += (u - 0.5) * step_length * ((intent.twist.vx_mps >= 0.0) ? -1.0 : 1.0);
            foot.y += (u - 0.5) * step_width * ((intent.twist.vy_mps >= 0.0) ? -1.0 : 1.0);
            foot.z += step_height * std::sin(u * M_PI);
        }

        // Tiny body-level roll/pitch stabilization term (very simplified).
        foot.y += -0.02 * est.body_rpy_rad.x;
        foot.x += -0.02 * est.body_rpy_rad.y;

        out.feet[leg].pos_body_m = foot;
        out.feet[leg].vel_body_mps = {};
    }*/

    return out;
}