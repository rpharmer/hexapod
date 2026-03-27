#include "motion_limiter.hpp"

#include <cstdlib>
#include <string_view>

namespace {

bool motionLimiterEnabled() {
    const char* raw = std::getenv("HEXAPOD_ENABLE_MOTION_LIMITER");
    if (raw == nullptr) {
        return false;
    }
    const std::string_view value{raw};
    return value == "1" || value == "true" || value == "TRUE" || value == "yes" || value == "YES";
}

} // namespace

MotionLimiterOutput MotionLimiter::update(const RobotState& estimated,
                                          const MotionIntent& intent,
                                          const RuntimeGaitPolicy& gait_policy,
                                          const SafetyState& safety_state,
                                          const DurationSec& loop_dt) const {
    (void)estimated;
    (void)safety_state;

    MotionLimiterOutput output{};
    output.limited_intent = intent;
    output.adapted_gait_policy = gait_policy;
    output.diagnostics.enabled = motionLimiterEnabled();
    output.diagnostics.loop_dt = loop_dt;
    return output;
}
