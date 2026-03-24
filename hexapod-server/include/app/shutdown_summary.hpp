#pragma once

#include <cstddef>
#include <memory>

#include "logger.hpp"

namespace app {

void logShutdownSummary(const std::shared_ptr<logging::AsyncLogger>& logger,
                        bool teardown_ok,
                        std::size_t dropped_messages,
                        int runner_rc);

} // namespace app
