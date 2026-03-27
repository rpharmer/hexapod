#include "autonomy/modules/traversability_analyzer.hpp"

namespace autonomy {

TraversabilityAnalyzerModuleShell::TraversabilityAnalyzerModuleShell()
    : AutonomyModuleStub("traversability_analyzer") {}

namespace {
double clamp01(double value) {
    if (value < 0.0) {
        return 0.0;
    }
    if (value > 1.0) {
        return 1.0;
    }
    return value;
}
} // namespace

TraversabilityReport TraversabilityAnalyzerModuleShell::analyze(const WorldModelSnapshot& world_model,
                                                                uint64_t timestamp_ms,
                                                                ContractEnvelope envelope) {
    (void)envelope;
    report_.timestamp_ms = timestamp_ms;
    report_.risk = clamp01((0.65 * world_model.occupancy) +
                           (0.35 * world_model.terrain_gradient));
    report_.confidence = clamp01(world_model.risk_confidence);
    report_.cost = report_.risk + (1.0 - report_.confidence);
    report_.traversable = world_model.has_map && report_.risk < 0.85 && report_.confidence >= 0.3;
    return report_;
}

TraversabilityReport TraversabilityAnalyzerModuleShell::report() const {
    return report_;
}

} // namespace autonomy
