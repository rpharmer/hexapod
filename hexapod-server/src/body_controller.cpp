#include "body_controller.hpp"

LegTargets BodyController::update(const EstimatedState& est,
                                  const MotionIntent& intent,
                                  const GaitState& gait,
                                  const SafetyState& safety) {
    LegTargets out{};
    out.timestamp_us = now_us();

    // TODO(control): implement stance/swing foot placement policy.
    // Inputs are intentionally unused until the replacement controller lands.
    (void)est;
    (void)intent;
    (void)gait;
    (void)safety;

    return out;
}
