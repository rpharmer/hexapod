#include "loop_timing.hpp"

#include <thread>

namespace loop_timing {

void sleepUntil(const Clock::time_point& deadline) {
    std::this_thread::sleep_until(deadline);
}

}  // namespace loop_timing
