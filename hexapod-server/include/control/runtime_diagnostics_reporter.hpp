#pragma once

#include "freshness_policy.hpp"
#include "hardware_bridge.hpp"
#include "logger.hpp"
#include "types.hpp"

#include <cstdint>
#include <memory>
#include <optional>

class RuntimeDiagnosticsReporter {
public:
    RuntimeDiagnosticsReporter(std::shared_ptr<logging::AsyncLogger> logger,
                               const FreshnessPolicy& freshness_policy);

    void report(const ControlStatus& status,
                const std::optional<BridgeCommandResultMetadata>& bridge_result,
                uint64_t loops,
                uint64_t avg_control_dt_us,
                uint64_t max_control_jitter_us,
                uint64_t stale_intent_events,
                uint64_t stale_estimator_events) const;

private:
    std::shared_ptr<logging::AsyncLogger> logger_;
    const FreshnessPolicy& freshness_policy_;
};
