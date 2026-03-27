#pragma once

#include "autonomy/module_stubs.hpp"
#include "autonomy/common_types.hpp"
#include "autonomy/modules/module_data.hpp"

namespace autonomy {

struct TraversabilityScoreCompositionPolicy {
    double aggregated_primary_weight{0.65};
    double aggregated_secondary_weight{0.35};
    double risk_aggregate_weight{0.75};
    double risk_peak_weight{0.25};
    double confidence_base_weight{0.8};
    double confidence_zone_weight{0.2};
    double confidence_zone_degraded_weight{0.5};
};

class TraversabilityAnalyzerModuleShell : public AutonomyModuleStub {
public:
    explicit TraversabilityAnalyzerModuleShell(const TraversabilityPolicyConfig& config = {},
                                               TraversabilityScoreCompositionPolicy composition_policy = {});

    TraversabilityReport analyze(const WorldModelSnapshot& world_model,
                                 TimestampMs timestamp_ms,
                                ContractEnvelope envelope = {});
    [[nodiscard]] TraversabilityReport report() const;

private:
    TraversabilityPolicyConfig config_{};
    TraversabilityScoreCompositionPolicy composition_policy_{};
    TraversabilityReport report_{};
};

} // namespace autonomy
