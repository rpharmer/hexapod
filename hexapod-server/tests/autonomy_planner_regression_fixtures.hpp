#pragma once

#include "autonomy/modules/module_data.hpp"

#include <string>
#include <vector>

namespace autonomy_regression_fixtures {

struct TerrainFixture {
    std::string name;
    autonomy::WorldModelSnapshot world;
    bool expect_traversable;
};

inline std::vector<TerrainFixture> plannerTerrainFixtures() {
    return {
        TerrainFixture{
            .name = "nominal",
            .world = autonomy::WorldModelSnapshot{
                .has_map = true,
                .occupancy = 0.10,
                .terrain_gradient = 0.12,
                .risk_confidence = 0.96,
                .obstacle_band_near = 0.05,
                .obstacle_band_mid = 0.08,
                .obstacle_band_far = 0.10,
                .slope_band_high = 0.08,
                .confidence_zone_nominal = 0.95,
                .confidence_zone_degraded = 0.04,
                .confidence_zone_unknown = 0.01,
            },
            .expect_traversable = true,
        },
        TerrainFixture{
            .name = "near_threshold",
            .world = autonomy::WorldModelSnapshot{
                .has_map = true,
                .occupancy = 0.73,
                .terrain_gradient = 0.66,
                .risk_confidence = 0.52,
                .obstacle_band_near = 0.68,
                .obstacle_band_mid = 0.62,
                .obstacle_band_far = 0.50,
                .slope_band_high = 0.70,
                .confidence_zone_nominal = 0.48,
                .confidence_zone_degraded = 0.32,
                .confidence_zone_unknown = 0.20,
            },
            .expect_traversable = true,
        },
        TerrainFixture{
            .name = "adversarial",
            .world = autonomy::WorldModelSnapshot{
                .has_map = true,
                .occupancy = 0.98,
                .terrain_gradient = 0.95,
                .risk_confidence = 0.15,
                .obstacle_band_near = 0.96,
                .obstacle_band_mid = 0.85,
                .obstacle_band_far = 0.70,
                .slope_band_high = 0.92,
                .confidence_zone_nominal = 0.15,
                .confidence_zone_degraded = 0.25,
                .confidence_zone_unknown = 0.60,
            },
            .expect_traversable = false,
        },
    };
}

} // namespace autonomy_regression_fixtures
