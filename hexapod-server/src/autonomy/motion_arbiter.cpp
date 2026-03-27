#include "autonomy/motion_arbiter.hpp"

namespace autonomy {

MotionDecision MotionArbiter::select(bool estop,
                                     bool hold,
                                     bool recovery,
                                     const NavigationUpdate& nav_update) const {
    if (estop) {
        return MotionDecision{
            .source = MotionSource::EStop,
            .allow_motion = false,
            .reason = "estop asserted",
            .nav_intent = {},
        };
    }

    if (hold) {
        return MotionDecision{
            .source = MotionSource::Hold,
            .allow_motion = false,
            .reason = "hold asserted",
            .nav_intent = {},
        };
    }

    if (recovery) {
        return MotionDecision{
            .source = MotionSource::Recovery,
            .allow_motion = false,
            .reason = "recovery active",
            .nav_intent = {},
        };
    }

    if (nav_update.has_intent && nav_update.status == NavigationStatus::Active) {
        return MotionDecision{
            .source = MotionSource::Nav,
            .allow_motion = true,
            .reason = {},
            .nav_intent = nav_update.intent,
        };
    }

    return MotionDecision{
        .source = MotionSource::None,
        .allow_motion = false,
        .reason = nav_update.reason,
        .nav_intent = {},
    };
}

} // namespace autonomy
