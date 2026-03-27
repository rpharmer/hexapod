#include "autonomy/modules/world_model.hpp"

#include <cmath>

namespace autonomy {

WorldModelModuleShell::WorldModelModuleShell()
    : AutonomyModuleStub("world_model") {}

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

WorldModelSnapshot WorldModelModuleShell::update(const LocalizationEstimate& localization,
                                                 const MapSliceInput& map_slice_input,
                                                 bool blocked,
                                                 uint64_t timestamp_ms) {
    if (map_slice_input.has_occupancy) {
        snapshot_.occupancy = clamp01(map_slice_input.occupancy);
    }
    if (map_slice_input.has_elevation) {
        const double previous_elevation_m = snapshot_.elevation_m;
        snapshot_.elevation_m = map_slice_input.elevation_m;
        snapshot_.terrain_gradient = map_slice_input.has_occupancy
                                         ? map_slice_input.occupancy
                                         : clamp01(std::abs(snapshot_.elevation_m - previous_elevation_m));
    }
    if (map_slice_input.has_risk_confidence) {
        snapshot_.risk_confidence = clamp01(map_slice_input.risk_confidence);
    }

    if (blocked) {
        snapshot_.occupancy = 1.0;
        snapshot_.risk_confidence = 0.0;
    }

    snapshot_.has_map = localization.valid &&
                        (map_slice_input.has_occupancy || map_slice_input.has_elevation || map_slice_input.has_risk_confidence || snapshot_.timestamp_ms > 0);
    snapshot_.timestamp_ms = timestamp_ms;
    return snapshot_;
}

WorldModelSnapshot WorldModelModuleShell::snapshot() const {
    return snapshot_;
}

} // namespace autonomy
