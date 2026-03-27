#include "autonomy/modules/traversability_analyzer.hpp"

#include "core/math_types.hpp"

#include <algorithm>
#include <cmath>

namespace autonomy {
namespace {

double normalizedWeightedBlend(const std::initializer_list<std::pair<double, double>>& weighted_values) {
    double total_weight = 0.0;
    double weighted_sum = 0.0;
    for (const auto& [value, weight] : weighted_values) {
        const double clamped_weight = std::max(0.0, weight);
        total_weight += clamped_weight;
        weighted_sum += clamped_weight * value;
    }
    if (total_weight <= 0.0) {
        return 0.0;
    }
    return weighted_sum / total_weight;
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

    const double occupancy_risk = ::clamp01(world_model.occupancy);
    const double gradient_risk = ::clamp01(world_model.terrain_gradient);
    const double near_obstacle_risk = ::clamp01(world_model.obstacle_band_near);
    const double mid_obstacle_risk = ::clamp01(world_model.obstacle_band_mid);
    const double far_obstacle_risk = ::clamp01(world_model.obstacle_band_far);
    const double high_slope_risk = ::clamp01(world_model.slope_band_high);

    const double primary_risk = normalizedWeightedBlend({
        {occupancy_risk, config_.occupancy_risk_weight},
        {gradient_risk, config_.gradient_risk_weight},
    });
    const double secondary_risk = normalizedWeightedBlend({
        {near_obstacle_risk, config_.obstacle_near_risk_weight},
        {mid_obstacle_risk, config_.obstacle_mid_risk_weight},
        {far_obstacle_risk, config_.obstacle_far_risk_weight},
        {high_slope_risk, config_.slope_high_risk_weight},
    });
    const double aggregated_risk = std::max(primary_risk, 0.65 * primary_risk + 0.35 * secondary_risk);
    const double peak_risk = std::max({occupancy_risk, gradient_risk, near_obstacle_risk, high_slope_risk});
    report_.risk = ::clamp01(0.75 * aggregated_risk + 0.25 * peak_risk);

    const double base_confidence = ::clamp01(world_model.risk_confidence);
    const double zone_confidence = ::clamp01(world_model.confidence_zone_nominal +
                                             0.5 * world_model.confidence_zone_degraded);
    const double unknown_penalty = std::max(0.0, config_.confidence_unknown_penalty) *
                                   ::clamp01(world_model.confidence_zone_unknown);
    report_.confidence = ::clamp01((0.8 * base_confidence + 0.2 * zone_confidence) - unknown_penalty);
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
