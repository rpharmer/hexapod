#include "autonomy/modules/world_model.hpp"

#include "core/math_types.hpp"

#include <cmath>

namespace autonomy {

WorldModelModuleShell::WorldModelModuleShell()
    : AutonomyModuleStub("world_model") {}

WorldModelSnapshot WorldModelModuleShell::update(const LocalizationEstimate& localization,
                                                 const MapSliceInput& map_slice_input,
                                                 bool blocked,
                                                 TimestampMs timestamp_ms,
                                                 ContractEnvelope envelope) {
    (void)envelope;
    snapshot_.stale_input_rejected = false;
    snapshot_.frame_mismatch = false;
    snapshot_.non_monotonic_timestamp = false;

    const TimestampMs effective_map_timestamp =
        map_slice_input.has_timestamp ? map_slice_input.timestamp_ms : timestamp_ms;

    const bool localization_frame_supported =
        localization.frame_id == localization_map_frame_ || localization.frame_id == localization_odom_frame_;
    const bool localization_frame_valid = !localization.valid || localization_frame_supported;
    const bool map_frame_valid = map_slice_input.frame_id.empty() || map_slice_input.frame_id == map_frame_expected_;
    const bool frame_consistent = localization_frame_valid && map_frame_valid;
    snapshot_.frame_mismatch = !frame_consistent;
    if (!frame_consistent) {
        snapshot_.has_map = false;
        snapshot_.timestamp_ms = timestamp_ms;
        return snapshot_;
    }
    snapshot_.frame_id = map_frame_expected_;

    if (last_map_timestamp_ms_ > TimestampMs{} && effective_map_timestamp < last_map_timestamp_ms_) {
        snapshot_.non_monotonic_timestamp = true;
        snapshot_.stale_input_rejected = true;
        snapshot_.has_map = false;
        snapshot_.timestamp_ms = timestamp_ms;
        return snapshot_;
    }
    const uint64_t map_age_ms = (timestamp_ms >= effective_map_timestamp)
                                    ? static_cast<uint64_t>(timestamp_ms - effective_map_timestamp)
                                    : 0ULL;
    if (map_age_ms > stale_map_threshold_ms_) {
        snapshot_.stale_input_rejected = true;
        snapshot_.has_map = false;
        snapshot_.timestamp_ms = timestamp_ms;
        return snapshot_;
    }

    if (map_slice_input.has_occupancy) {
        snapshot_.occupancy = ::clamp01(map_slice_input.occupancy);
        snapshot_.occupancy_provenance = map_slice_input.occupancy_provenance;
    }
    if (map_slice_input.has_elevation) {
        const double previous_elevation_m = snapshot_.elevation_m;
        snapshot_.elevation_m = map_slice_input.elevation_m;
        snapshot_.terrain_gradient = map_slice_input.has_occupancy
                                         ? map_slice_input.occupancy
                                         : ::clamp01(std::abs(snapshot_.elevation_m - previous_elevation_m));
        snapshot_.elevation_provenance = map_slice_input.elevation_provenance;
    }
    if (map_slice_input.has_risk_confidence) {
        snapshot_.risk_confidence = ::clamp01(map_slice_input.risk_confidence);
        snapshot_.risk_confidence_provenance = map_slice_input.risk_confidence_provenance;
    }

    if (blocked) {
        snapshot_.occupancy = 1.0;
        snapshot_.risk_confidence = 0.0;
        snapshot_.occupancy_provenance = MapDataProvenance::SafetyOverride;
        snapshot_.risk_confidence_provenance = MapDataProvenance::SafetyOverride;
    }

    snapshot_.has_map = localization.valid && localization_frame_supported &&
                        (map_slice_input.has_occupancy ||
                         map_slice_input.has_elevation ||
                         map_slice_input.has_risk_confidence ||
                         snapshot_.timestamp_ms > TimestampMs{});
    rebuildPlannerBands();
    last_map_timestamp_ms_ = effective_map_timestamp;
    snapshot_.timestamp_ms = timestamp_ms;
    return snapshot_;
}

WorldModelSnapshot WorldModelModuleShell::snapshot() const {
    return snapshot_;
}

void WorldModelModuleShell::rebuildPlannerBands() {
    snapshot_.obstacle_band_near = ::clamp01(snapshot_.occupancy);
    snapshot_.obstacle_band_mid = ::clamp01((0.75 * snapshot_.occupancy) + (0.25 * snapshot_.terrain_gradient));
    snapshot_.obstacle_band_far = ::clamp01((0.50 * snapshot_.occupancy) + (0.50 * snapshot_.terrain_gradient));

    const double slope = ::clamp01(snapshot_.terrain_gradient);
    snapshot_.slope_band_low = ::clamp01(slope * 0.50);
    snapshot_.slope_band_mid = ::clamp01(slope * 0.80);
    snapshot_.slope_band_high = slope;

    snapshot_.confidence_zone_nominal = ::clamp01(snapshot_.risk_confidence);
    snapshot_.confidence_zone_degraded = ::clamp01(1.0 - snapshot_.risk_confidence);
    snapshot_.confidence_zone_unknown = snapshot_.risk_confidence_provenance == MapDataProvenance::Unknown
                                            ? ::clamp01(1.0 - snapshot_.confidence_zone_nominal)
                                            : 0.0;
}

} // namespace autonomy
