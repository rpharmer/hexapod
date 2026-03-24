#pragma once

#include "logger.hpp"
#include "hardware_bridge.hpp"
#include "types.hpp"

#include <optional>
#include <memory>

namespace status_reporter {

void logStatus(const std::shared_ptr<logging::AsyncLogger>& logger,
               const ControlStatus& status,
               const std::optional<BridgeCommandResultMetadata>& bridge_result = std::nullopt);

}  // namespace status_reporter
