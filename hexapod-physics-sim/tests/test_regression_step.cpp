#include <cassert>
#include <cmath>
#include <cstdint>
#include <vector>

#include "minphys3d/minphys3d.hpp"

int main() {
    using namespace minphys3d;

    {
    World world({0.0f, 0.0f, 0.0f});

    Body a;
    a.shape = ShapeType::Sphere;
    a.radius = 0.25f;
    a.mass = 1.0f;
    a.position = {-0.5f, 0.0f, 0.0f};
    const auto aId = world.CreateBody(a);

    Body b = a;
    b.position = {0.5f, 0.0f, 0.0f};
    const auto bId = world.CreateBody(b);

    const std::uint32_t hingeId = world.CreateHingeJoint(
        aId,
        bId,
        {0.0f, 0.0f, 0.0f},
        {0.0f, 0.0f, 0.0f});
    assert(hingeId == World::kInvalidJointId);

    world.Step(1.0f / 120.0f, 8);
    const Body& outA = world.GetBody(aId);
    const Body& outB = world.GetBody(bId);
    assert(std::isfinite(outA.position.x) && std::isfinite(outA.position.y) && std::isfinite(outA.position.z));
    assert(std::isfinite(outB.position.x) && std::isfinite(outB.position.y) && std::isfinite(outB.position.z));
    }

    {
    World world({0.0f, -9.81f, 0.0f});
    ContactSolverConfig config = world.GetContactSolverConfig();
    config.bounceVelocityThreshold = 2.0f;
    config.restitutionSuppressionSpeed = 1.5f;
    config.restitutionVelocityCutoff = 1.25f;
    config.staticFrictionSpeedThreshold = 0.04f;
    config.staticToDynamicTransitionSpeed = 0.15f;
    config.useSplitImpulse = true;
    world.SetContactSolverConfig(config);
    const ContactSolverConfig roundtrip = world.GetContactSolverConfig();
    assert(roundtrip.useSplitImpulse);
    assert(std::abs(roundtrip.bounceVelocityThreshold - 2.0f) < 1e-6f);
    assert(std::abs(roundtrip.restitutionSuppressionSpeed - 1.5f) < 1e-6f);
    assert(std::abs(roundtrip.restitutionVelocityCutoff - 1.25f) < 1e-6f);
    assert(std::abs(roundtrip.staticFrictionSpeedThreshold - 0.04f) < 1e-6f);
    assert(std::abs(roundtrip.staticToDynamicTransitionSpeed - 0.15f) < 1e-6f);

    Body plane;
    plane.shape = ShapeType::Plane;
    plane.restitution = 1.0f;
    world.CreateBody(plane);

    Body sphere;
    sphere.shape = ShapeType::Sphere;
    sphere.position = {0.0f, 0.8f, 0.0f};
    sphere.velocity = {0.0f, -1.0f, 0.0f};
    sphere.radius = 0.25f;
    sphere.mass = 1.0f;
    sphere.restitution = 1.0f;
    const auto id = world.CreateBody(sphere);

    for (int i = 0; i < 180; ++i) {
        world.Step(1.0f / 120.0f, 10);
    }
    const Body& settled = world.GetBody(id);
    assert(std::isfinite(settled.position.y));
    assert(settled.position.y > 0.18f);
    }

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

    {
    World world({0.0f, 0.0f, 0.0f});

    Body left;
    left.shape = ShapeType::Sphere;
    left.radius = 0.25f;
    left.mass = 1.0f;
    left.position = {-0.7f, 0.0f, 0.0f};
    const auto leftId = world.CreateBody(left);

    Body mid = left;
    mid.position = {0.0f, 0.0f, 0.0f};
    const auto midId = world.CreateBody(mid);

    Body right = left;
    right.position = {0.7f, 0.0f, 0.0f};
    const auto rightId = world.CreateBody(right);

    world.CreateDistanceJoint(leftId, midId, world.GetBody(leftId).position, world.GetBody(midId).position, 1.0f, 0.2f);
    world.CreateDistanceJoint(midId, rightId, world.GetBody(midId).position, world.GetBody(rightId).position, 1.0f, 0.2f);

    for (int i = 0; i < 180; ++i) {
        world.Step(1.0f / 120.0f, 16);
    }

    assert(world.GetBody(leftId).isSleeping);
    assert(world.GetBody(midId).isSleeping);
    assert(world.GetBody(rightId).isSleeping);

    world.AddForce(leftId, {50.0f, 0.0f, 0.0f});
    world.Step(1.0f / 120.0f, 16);

    assert(!world.GetBody(leftId).isSleeping);
    assert(!world.GetBody(midId).isSleeping);
    assert(!world.GetBody(rightId).isSleeping);
    }

    {
    auto captureSelectedPairSequence = []() {
        World world({0.0f, -9.81f, 0.0f});
        ContactSolverConfig cfg = world.GetContactSolverConfig();
        cfg.useBlockSolver = true;
        cfg.useSplitImpulse = true;
        cfg.penetrationSlop = 0.005f;
        cfg.splitImpulseCorrectionFactor = 0.85f;
        world.SetContactSolverConfig(cfg);

        Body plane;
        plane.shape = ShapeType::Plane;
        world.CreateBody(plane);

        Body box;
        box.shape = ShapeType::Box;
        box.halfExtents = {0.55f, 0.20f, 0.35f};
        box.mass = 2.0f;
        box.position = {0.0f, 1.1f, 0.0f};
        box.orientation = Normalize(Quat{0.9990482f, 0.0f, 0.0436194f, 0.0f});
        box.velocity = {0.45f, 0.0f, 0.12f};
        box.angularVelocity = {0.0f, 0.3f, 0.0f};
        world.CreateBody(box);

        std::vector<std::uint64_t> sequence;
        sequence.reserve(640);
        std::uint64_t previousPair = 0;
        int transitions = 0;
        for (int i = 0; i < 640; ++i) {
            world.Step(1.0f / 120.0f, 16);
            std::uint64_t selectedPair = 0;
            for (const Manifold& manifold : world.DebugManifolds()) {
                if (manifold.contacts.size() != 4) {
                    continue;
                }
                const std::uint64_t lo = std::min(manifold.selectedBlockContactKeys[0], manifold.selectedBlockContactKeys[1]);
                const std::uint64_t hi = std::max(manifold.selectedBlockContactKeys[0], manifold.selectedBlockContactKeys[1]);
                selectedPair = (lo << 1) ^ (hi * 1099511628211ull);
                break;
            }
            if (selectedPair != 0 && previousPair != 0 && selectedPair != previousPair) {
                ++transitions;
            }
            previousPair = selectedPair;
            sequence.push_back(selectedPair);
        }
        assert(transitions <= 24);
        return sequence;
    };

    const std::vector<std::uint64_t> first = captureSelectedPairSequence();
    const std::vector<std::uint64_t> second = captureSelectedPairSequence();
    assert(first == second);
    }
    return 0;
}
