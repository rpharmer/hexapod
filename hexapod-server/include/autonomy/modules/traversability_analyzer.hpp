#pragma once

#include "autonomy/module_stubs.hpp"
#include "autonomy/common_types.hpp"
#include "autonomy/modules/module_data.hpp"

namespace autonomy {

class TraversabilityAnalyzerModuleShell : public AutonomyModuleStub {
public:
    explicit TraversabilityAnalyzerModuleShell(const TraversabilityPolicyConfig& config = {});

    TraversabilityReport analyze(const WorldModelSnapshot& world_model,
                                 TimestampMs timestamp_ms,
                                ContractEnvelope envelope = {});
    [[nodiscard]] TraversabilityReport report() const;

private:
    TraversabilityPolicyConfig config_{};
    TraversabilityReport report_{};
};

} // namespace autonomy
