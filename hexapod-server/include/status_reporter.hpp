#pragma once

#include "logger.hpp"
#include "types.hpp"

#include <memory>

namespace status_reporter {

void logStatus(const std::shared_ptr<logging::AsyncLogger>& logger,
               const ControlStatus& status);

}  // namespace status_reporter
