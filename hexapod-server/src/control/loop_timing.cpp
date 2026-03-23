#include "loop_timing.hpp"

#include <thread>

namespace loop_timing {

void sleepUntil(const Clock::time_point& start,
                std::chrono::microseconds period) {
    std::this_thread::sleep_until(start + period);
}

}  // namespace loop_timing
