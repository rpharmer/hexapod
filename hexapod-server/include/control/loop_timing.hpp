#pragma once

#include "types.hpp"

#include <chrono>

namespace loop_timing {

void sleepUntil(const Clock::time_point& deadline);

}  // namespace loop_timing
