#include "physics_sim_local_map_source.hpp"

#include "types.hpp"

#include <algorithm>
#include <cmath>

namespace {

double normalizeSampleSpacing(const double sample_spacing_m) {
    return std::max(0.02, sample_spacing_m);
}

} // namespace

PhysicsSimLocalMapObservationSource::PhysicsSimLocalMapObservationSource(
    const IPhysicsSimObstacleFootprintProvider& provider,
    const double sample_spacing_m)
    : provider_(provider),
      sample_spacing_m_(normalizeSampleSpacing(sample_spacing_m)) {}

LocalMapObservation PhysicsSimLocalMapObservationSource::collect(const NavPose2d& /*pose*/,
                                                                 const RobotState& /*est*/,
                                                                 const TimePointUs now) {
    LocalMapObservation out{};
    out.timestamp_us = now;
    out.freshness_class = LocalMapObservationFreshnessClass::Auxiliary;

    const std::vector<PhysicsSimObstacleFootprint> footprints = provider_.latestObstacleFootprints();
    for (const PhysicsSimObstacleFootprint& footprint : footprints) {
        const double hx = std::max(0.0, footprint.half_extent_x_m);
        const double hy = std::max(0.0, footprint.half_extent_y_m);
        if (hx <= 0.0 || hy <= 0.0) {
            continue;
        }

        const int steps_x = std::max(1, static_cast<int>(std::ceil((2.0 * hx) / sample_spacing_m_)));
        const int steps_y = std::max(1, static_cast<int>(std::ceil((2.0 * hy) / sample_spacing_m_)));
        const double cos_yaw = std::cos(footprint.yaw_rad);
        const double sin_yaw = std::sin(footprint.yaw_rad);

        for (int ix = 0; ix <= steps_x; ++ix) {
            const double local_x =
                -hx + (2.0 * hx * static_cast<double>(ix) / static_cast<double>(steps_x));
            for (int iy = 0; iy <= steps_y; ++iy) {
                const double local_y =
                    -hy + (2.0 * hy * static_cast<double>(iy) / static_cast<double>(steps_y));
                const double world_x =
                    footprint.center_x_m + local_x * cos_yaw - local_y * sin_yaw;
                const double world_y =
                    footprint.center_y_m + local_x * sin_yaw + local_y * cos_yaw;
                out.samples.push_back(
                    LocalMapObservationSample{world_x, world_y, LocalMapCellState::Occupied});
            }
        }
    }

    return out;
}
