#include <cassert>
#include <cstdint>

#include "minphys3d/core/world.hpp"

int main() {
    using namespace minphys3d;

    World overlapWorld({0.0f, 0.0f, 0.0f});

    for (int i = 0; i < 12; ++i) {
        Body sphere;
        sphere.shape = ShapeType::Sphere;
        sphere.mass = 1.0f;
        sphere.radius = 0.6f;
        sphere.position = {static_cast<float>(i % 4) * 0.9f, static_cast<float>(i / 4) * 0.9f, 0.0f};
        overlapWorld.CreateBody(sphere);
    }

    assert(overlapWorld.BroadphasePairCount() == overlapWorld.BruteForcePairCount());

    World world({0.0f, 0.0f, 0.0f});
    const std::size_t bodyCount = 12;
    for (std::uint32_t i = 0; i < bodyCount; ++i) {
        Body sphere;
        sphere.shape = ShapeType::Sphere;
        sphere.mass = 1.0f;
        sphere.radius = 0.25f;
        sphere.position = {static_cast<float>(i) * 1.25f, 0.0f, 0.0f};
        world.CreateBody(sphere);
    }
    world.GetBody(0).velocity = {0.05f, 0.0f, 0.0f};
    world.GetBody(1).velocity = {-0.05f, 0.0f, 0.0f};

    world.Step(1.0f / 120.0f, 1);

    assert(world.BroadphasePairCount() == world.BruteForcePairCount());
    assert(world.LastBroadphaseMovedProxyCount() <= bodyCount / 2);

    return 0;
}
