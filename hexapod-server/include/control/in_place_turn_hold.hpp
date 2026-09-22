#pragma once

#include "twist_field.hpp"

struct RobotState;
struct MotionIntent;

// Requires a drift-bounded absolute position source. This is not suitable for
// unconstrained hardware odometry. Existing campaign gains are unchanged.
class InPlaceTurnHold {
public:
    void reset() { valid_ = false; origin_ = {}; }
    void apply(const RobotState& estimated, const MotionIntent& intent,
               BodyTwist& command, bool enabled);
private:
    Vec3 origin_{};
    bool valid_{};
};
