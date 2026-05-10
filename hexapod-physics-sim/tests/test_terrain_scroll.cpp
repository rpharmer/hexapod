#include "demo/terrain_patch.hpp"

#include <cmath>
#include <iostream>

namespace {

bool Near(minphys3d::Real a, minphys3d::Real b, minphys3d::Real eps) {
    return std::abs(a - b) <= eps;
}

} // namespace

int main() {
    using namespace minphys3d;
    using namespace minphys3d::demo;

    World world;
    TerrainPatchConfig cfg{};
    cfg.rows = 11;
    cfg.cols = 11;
    cfg.cell_size_m = 0.1;
    cfg.scroll_world_fixed = true;
    cfg.base_update_blend = 1.0;
    cfg.decay_update_boost = 0.0;

    TerrainPatch patch{cfg};
    patch.initialize(world, Vec3{0.0, 0.0, 0.0}, 0.0);

    std::vector<TerrainSample> bump{
        TerrainSample{Vec3{0.0, 0.0, 0.0}, 0.35, 1.0},
    };
    patch.update(world, Vec3{0.0, 0.0, 0.0}, 0.0, Vec3{0.0, 1.0, 0.0}, bump, 0.0);
    const Real h_center_before = patch.SampleHeightWorld(0.0, 0.0);
    if (h_center_before < 0.08) {
        std::cerr << "expected bump at origin, got " << h_center_before << "\n";
        return 1;
    }

    const Vec3 shifted{0.2, 0.0, 0.0};
    // Re-apply the same samples after scrolling; an empty sample list would fuse purely toward the plane.
    patch.update(world, shifted, 0.0, Vec3{0.0, 1.0, 0.0}, bump, 0.0);
    const Real h_world_after = patch.SampleHeightWorld(0.0, 0.0);
    if (!Near(h_world_after, h_center_before, 0.06)) {
        std::cerr << "scroll blit lost bump: before=" << h_center_before << " after=" << h_world_after << "\n";
        return 2;
    }

    std::cout << "test_terrain_scroll ok h=" << h_world_after << "\n";
    return 0;
}
