#pragma once

#include "types.hpp"

#include <chrono>

namespace loop_timing {

void sleepUntil(const Clock::time_point& start,
                std::chrono::microseconds period);

}  // namespace loop_timing
