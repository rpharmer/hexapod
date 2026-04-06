#include <cassert>
#include <cmath>

#include "minphys3d/minphys3d.hpp"

int main() {
    using namespace minphys3d;

    {
    World world({0.0f, -9.81f, 0.0f});

    Body plane;
    plane.shape = ShapeType::Plane;
    world.CreateBody(plane);

    Body sphere;
    sphere.shape = ShapeType::Sphere;
    sphere.position = {0.0f, 1.0f, 0.0f};
    sphere.radius = 0.25f;
    sphere.mass = 1.0f;
    const auto id = world.CreateBody(sphere);

    for (int i = 0; i < 60; ++i) {
        world.Step(1.0f / 60.0f, 8);
    }

    const Body& out = world.GetBody(id);
    assert(out.position.y > -0.01f);
    }

    {
    World world({0.0f, 0.0f, 0.0f});

    Body a;
    a.shape = ShapeType::Capsule;
    a.position = {0.0f, 0.0f, 0.0f};
    a.radius = 0.25f;
    a.halfHeight = 0.75f;
    a.mass = 1.0f;
    const auto aId = world.CreateBody(a);

    Body b = a;
    b.position = {0.0f, 0.0f, 0.0f};
    const auto bId = world.CreateBody(b);

    for (int i = 0; i < 120; ++i) {
        world.Step(1.0f / 120.0f, 12);
    }

    const Body& outA = world.GetBody(aId);
    const Body& outB = world.GetBody(bId);
    assert(std::isfinite(outA.position.x) && std::isfinite(outA.position.y) && std::isfinite(outA.position.z));
    assert(std::isfinite(outB.position.x) && std::isfinite(outB.position.y) && std::isfinite(outB.position.z));
    assert(Length(outB.position - outA.position) > 0.001f);
    }

    {
    auto runFastSphereAtThinBox = []() {
        World world({0.0f, 0.0f, 0.0f});

        Body target;
        target.shape = ShapeType::Box;
        target.isStatic = true;
        target.position = {0.0f, 0.8f, 0.0f};
        target.halfExtents = {0.08f, 0.45f, 0.45f};
        world.CreateBody(target);

        Body sphere;
        sphere.shape = ShapeType::Sphere;
        sphere.radius = 0.2f;
        sphere.position = {-3.0f, 0.8f, 0.0f};
        sphere.velocity = {70.0f, 0.0f, 0.0f};
        sphere.mass = 1.0f;
        sphere.restitution = 0.0f;
        const auto sphereId = world.CreateBody(sphere);

        for (int i = 0; i < 20; ++i) {
            world.Step(1.0f / 60.0f, 8);
        }

        return world.GetBody(sphereId).position;
    };

    const Vec3 firstRun = runFastSphereAtThinBox();
    const Vec3 secondRun = runFastSphereAtThinBox();

    assert(firstRun.x <= 0.30f);
    assert(std::abs(firstRun.x - secondRun.x) <= 1e-5f);
    assert(std::abs(firstRun.y - secondRun.y) <= 1e-5f);
    assert(std::abs(firstRun.z - secondRun.z) <= 1e-5f);
    }
    return 0;
}
