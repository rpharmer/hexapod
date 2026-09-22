#pragma once
#include <algorithm>
#include <cmath>
#include "minphys3d/math/scalar.hpp"

namespace minphys3d {
inline Real ClampServoMotorImpulse(Real requested, Real accumulated, Real speed,
                                  Real inverseInertia, Real stallImpulse, Real noLoadSpeed) {
    if (!(noLoadSpeed > 0) || !std::isfinite(noLoadSpeed))
        return std::clamp(requested, -stallImpulse, stallImpulse);
    // Remove this row's current impulse before solving its torque-speed bound.
    // v_new = v_without_row + W*p; p <= T*dt*(1-v_new/v_no_load).
    // Solving for p avoids the noncontractive "full drive -> overspeed -> zero
    // drive -> full drive" iteration on light links. Braking still permits
    // stall torque; no separate velocity constraint or post-step clamp is used.
    const Real freeSpeed = speed - inverseInertia * accumulated;
    const Real denominator = noLoadSpeed + stallImpulse * inverseInertia;
    const Real positive = std::clamp((noLoadSpeed - freeSpeed) / denominator, 0.0, 1.0);
    const Real negative = std::clamp((noLoadSpeed + freeSpeed) / denominator, 0.0, 1.0);
    return std::clamp(requested, -stallImpulse * negative, stallImpulse * positive);
}
}
