#include "demo/terrain_patch.hpp"

#include <cmath>
#include <iostream>

namespace {

bool Near(float a, float b, float eps) {
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
    cfg.cell_size_m = 0.1f;
    cfg.scroll_world_fixed = true;
    cfg.base_update_blend = 1.0f;
    cfg.decay_update_boost = 0.0f;

    TerrainPatch patch{cfg};
    patch.initialize(world, Vec3{0.0f, 0.0f, 0.0f}, 0.0f);

    std::vector<TerrainSample> bump{
        TerrainSample{Vec3{0.0f, 0.0f, 0.0f}, 0.35f, 1.0f},
    };
    patch.update(world, Vec3{0.0f, 0.0f, 0.0f}, 0.0f, Vec3{0.0f, 1.0f, 0.0f}, bump, 0.0f);
    const float h_center_before = patch.SampleHeightWorld(0.0f, 0.0f);
    if (h_center_before < 0.08f) {
        std::cerr << "expected bump at origin, got " << h_center_before << "\n";
        return 1;
    }

    const Vec3 shifted{0.2f, 0.0f, 0.0f};
    // Re-apply the same samples after scrolling; an empty sample list would fuse purely toward the plane.
    patch.update(world, shifted, 0.0f, Vec3{0.0f, 1.0f, 0.0f}, bump, 0.0f);
    const float h_world_after = patch.SampleHeightWorld(0.0f, 0.0f);
    if (!Near(h_world_after, h_center_before, 0.06f)) {
        std::cerr << "scroll blit lost bump: before=" << h_center_before << " after=" << h_world_after << "\n";
        return 2;
    }

    std::cout << "test_terrain_scroll ok h=" << h_world_after << "\n";
    return 0;
}
