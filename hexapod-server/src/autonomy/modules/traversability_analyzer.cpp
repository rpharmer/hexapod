#include "autonomy/modules/traversability_analyzer.hpp"

#include "core/math_types.hpp"

namespace autonomy {

TraversabilityAnalyzerModuleShell::TraversabilityAnalyzerModuleShell()
    : AutonomyModuleStub("traversability_analyzer") {}

TraversabilityReport TraversabilityAnalyzerModuleShell::analyze(const WorldModelSnapshot& world_model,
                                                                TimestampMs timestamp_ms,
                                                                ContractEnvelope envelope) {
    (void)envelope;
    report_.timestamp_ms = timestamp_ms;
    report_.risk = ::clamp01((0.65 * world_model.occupancy) +
                             (0.35 * world_model.terrain_gradient));
    report_.confidence = ::clamp01(world_model.risk_confidence);
    report_.cost = report_.risk + (1.0 - report_.confidence);
    report_.traversable = world_model.has_map && report_.risk < 0.85 && report_.confidence >= 0.3;
    return report_;
}

TraversabilityReport TraversabilityAnalyzerModuleShell::report() const {
    return report_;
}

} // namespace autonomy
