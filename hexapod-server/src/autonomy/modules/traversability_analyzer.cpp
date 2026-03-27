#include "autonomy/modules/traversability_analyzer.hpp"

namespace autonomy {

TraversabilityAnalyzerModuleShell::TraversabilityAnalyzerModuleShell()
    : AutonomyModuleStub("traversability_analyzer") {}

TraversabilityReport TraversabilityAnalyzerModuleShell::analyze(const WorldModelSnapshot& world_model,
                                                                uint64_t timestamp_ms) {
    report_.timestamp_ms = timestamp_ms;
    report_.cost = world_model.obstacle_risk;
    report_.traversable = world_model.has_map && world_model.obstacle_risk < 0.9;
    return report_;
}

TraversabilityReport TraversabilityAnalyzerModuleShell::report() const {
    return report_;
}

} // namespace autonomy
