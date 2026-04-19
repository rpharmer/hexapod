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
    TerrainPatch patch{};
    patch.initialize(world, Vec3{0.0f, 0.0f, 0.0f}, 0.0f);

    const float flat_height = patch.SampleHeightWorld(0.0f, 0.0f);
    if (!Near(flat_height, 0.0f, 1.0e-3f)) {
        std::cerr << "expected flat terrain near 0, got " << flat_height << "\n";
        return 1;
    }

    const Vec3 flat_normal = patch.SampleNormalWorld(0.0f, 0.0f);
    if (flat_normal.y < 0.95f) {
        std::cerr << "expected upward normal on flat terrain, got y=" << flat_normal.y << "\n";
        return 2;
    }

    float flat_hit = 0.0f;
    if (!patch.RaycastWorld(Vec3{0.0f, 1.0f, 0.0f}, Vec3{0.0f, -1.0f, 0.0f}, flat_hit)) {
        std::cerr << "expected raycast hit on flat terrain\n";
        return 3;
    }
    if (!Near(flat_hit, 1.0f, 0.08f)) {
        std::cerr << "expected flat terrain hit near 1.0, got " << flat_hit << "\n";
        return 4;
    }

    std::vector<TerrainSample> bump_samples{
        TerrainSample{Vec3{0.0f, 0.0f, 0.0f}, 0.18f, 1.0f},
    };
    patch.update(world, Vec3{0.0f, 0.0f, 0.0f}, 0.0f, Vec3{0.0f, 1.0f, 0.0f}, bump_samples, 0.0f);

    const float bump_height = patch.SampleHeightWorld(0.0f, 0.0f);
    if (bump_height <= 0.03f) {
        std::cerr << "expected terrain bump to raise height, got " << bump_height << "\n";
        return 5;
    }

    float bump_hit = 0.0f;
    if (!patch.RaycastWorld(Vec3{0.0f, 1.0f, 0.0f}, Vec3{0.0f, -1.0f, 0.0f}, bump_hit)) {
        std::cerr << "expected raycast hit on bumped terrain\n";
        return 6;
    }
    if (!(bump_hit < flat_hit - 0.03f)) {
        std::cerr << "expected bumped terrain hit closer than flat hit (flat=" << flat_hit
                  << " bump=" << bump_hit << ")\n";
        return 7;
    }

    patch.update(world, Vec3{0.0f, 0.0f, 0.0f}, 0.0f, Vec3{0.0f, 1.0f, 0.0f}, {}, 5.0f);
    const float decayed_height = patch.SampleHeightWorld(0.0f, 0.0f);
    if (!(decayed_height < bump_height && decayed_height < 0.08f)) {
        std::cerr << "expected bump to decay back toward plane, bump=" << bump_height
                  << " decayed=" << decayed_height << "\n";
        return 8;
    }

    std::cout << "test_terrain_patch ok height=" << decayed_height << "\n";
    return 0;
}

