#pragma once

#include "autonomy/navigation_types.hpp"

#include <string>

namespace autonomy {

enum class MotionSource {
    None,
    Nav,
    Recovery,
    Hold,
    EStop,
};

struct MotionDecision {
    MotionSource source{MotionSource::None};
    bool allow_motion{false};
    std::string reason{};
    NavigationIntent nav_intent{};
};

class MotionArbiter {
public:
    MotionDecision select(bool estop,
                          bool hold,
                          bool recovery,
                          const NavigationUpdate& nav_update) const;
};

} // namespace autonomy
