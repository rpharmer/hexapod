#include "autonomy/modules/traversability_analyzer.hpp"

#include "core/math_types.hpp"

#include <algorithm>

namespace autonomy {
namespace {

double normalizedWeightedBlend(double a,
                               double b,
                               double a_weight,
                               double b_weight) {
    const double total_weight = std::max(0.0, a_weight) + std::max(0.0, b_weight);
    if (total_weight <= 0.0) {
        return 0.0;
    }
    return ((std::max(0.0, a_weight) * a) + (std::max(0.0, b_weight) * b)) / total_weight;
}

} // namespace

TraversabilityAnalyzerModuleShell::TraversabilityAnalyzerModuleShell(const TraversabilityPolicyConfig& config)
    : AutonomyModuleStub("traversability_analyzer"),
      config_(config) {}

TraversabilityReport TraversabilityAnalyzerModuleShell::analyze(const WorldModelSnapshot& world_model,
                                                                TimestampMs timestamp_ms,
                                                                ContractEnvelope envelope) {
    (void)envelope;
    report_.timestamp_ms = timestamp_ms;
    report_.non_traversable_reasons.clear();

    report_.risk = ::clamp01(normalizedWeightedBlend(world_model.occupancy,
                                                     world_model.terrain_gradient,
                                                     config_.occupancy_risk_weight,
                                                     config_.gradient_risk_weight));
    report_.confidence = ::clamp01(world_model.risk_confidence);
    report_.cost = report_.risk + (std::max(0.0, config_.confidence_cost_weight) * (1.0 - report_.confidence));

    if (!world_model.has_map) {
        report_.non_traversable_reasons.emplace_back("missing-map");
    }
    if (report_.risk >= config_.risk_block_threshold) {
        report_.non_traversable_reasons.emplace_back("risk-threshold-exceeded");
    }
    if (report_.confidence < config_.confidence_block_threshold) {
        report_.non_traversable_reasons.emplace_back("confidence-below-threshold");
    }

    report_.traversable = report_.non_traversable_reasons.empty();
    report_.reason = report_.traversable ? "traversable" : report_.non_traversable_reasons.front();
    return report_;
}

TraversabilityReport TraversabilityAnalyzerModuleShell::report() const {
    return report_;
}

} // namespace autonomy
