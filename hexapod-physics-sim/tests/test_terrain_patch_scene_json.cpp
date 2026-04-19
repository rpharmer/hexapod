#include "demo/scene_json.hpp"

#include "minphys3d/core/world.hpp"

#include <cmath>
#include <iostream>
#include <string>

namespace {

bool Near(float a, float b, float eps) {
    return std::abs(a - b) <= eps;
}

} // namespace

int main() {
    using namespace minphys3d;
    using namespace minphys3d::demo;

    const std::string json = R"JSON(
{
  "schema_version": 2,
  "terrain_patch": {
    "rows": 7,
    "cols": 9,
    "cell_size_m": 0.07,
    "base_margin_m": 0.11,
    "min_cell_thickness_m": 0.02,
    "influence_sigma_m": 0.17,
    "plane_confidence": 0.22,
    "confidence_half_life_s": 2.5,
    "base_update_blend": 0.4,
    "decay_update_boost": 0.6,
    "center": [1.25, 0.0, -0.5],
    "plane_height_m": 0.32,
    "plane_normal": [0.0, 0.97, 0.24]
  },
  "bodies": [
    {
      "shape": "plane",
      "static": true,
      "plane_normal": [0.0, 1.0, 0.0],
      "plane_offset": 0.0
    }
  ]
}
)JSON";

    World world;
    int solver_iterations = 8;
    std::string err;
    TerrainPatchConfig config{};
    TerrainPatchSeed seed{};
    if (!LoadWorldFromMinphysSceneJson(json, world, solver_iterations, err, nullptr, &config, &seed)) {
        std::cerr << "load failed: " << err << "\n";
        return 1;
    }

    if (world.GetBodyCount() != 1) {
        std::cerr << "expected one body, got " << world.GetBodyCount() << "\n";
        return 2;
    }
    if (!seed.has_config || !seed.has_center || !seed.has_plane_height || !seed.has_plane_normal) {
        std::cerr << "expected terrain patch seed fields to be populated\n";
        return 3;
    }
    if (config.rows != 7 || config.cols != 9) {
        std::cerr << "unexpected terrain patch grid dims\n";
        return 4;
    }
    if (!Near(config.cell_size_m, 0.07f, 1.0e-6f) || !Near(config.base_margin_m, 0.11f, 1.0e-6f)) {
        std::cerr << "unexpected terrain patch tuning\n";
        return 5;
    }
    if (!Near(seed.center.x, 1.25f, 1.0e-6f) || !Near(seed.center.z, -0.5f, 1.0e-6f)) {
        std::cerr << "unexpected terrain patch center\n";
        return 6;
    }
    if (!Near(seed.plane_height_m, 0.32f, 1.0e-6f)) {
        std::cerr << "unexpected terrain patch plane height\n";
        return 7;
    }
    if (seed.plane_normal.y < 0.95f) {
        std::cerr << "unexpected terrain patch plane normal\n";
        return 8;
    }

    std::cout << "test_terrain_patch_scene_json ok rows=" << config.rows << " cols=" << config.cols << "\n";
    return 0;
}

