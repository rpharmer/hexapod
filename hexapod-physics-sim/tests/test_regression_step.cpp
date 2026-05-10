#include <cassert>
#include <array>
#include <cmath>
#include <cstdint>
#include <unordered_set>
#include <vector>

#include "minphys3d/core/world.hpp"

int main() {
    using namespace minphys3d;

    {
    World world({0.0, 0.0, 0.0});

    Body a;
    a.shape = ShapeType::Sphere;
    a.radius = 0.25;
    a.mass = 1.0;
    a.position = {-0.5, 0.0, 0.0};
    const auto aId = world.CreateBody(a);

    Body b = a;
    b.position = {0.5, 0.0, 0.0};
    const auto bId = world.CreateBody(b);

    const std::uint32_t hingeId = world.CreateHingeJoint(
        aId,
        bId,
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0});
    assert(hingeId == World::kInvalidJointId);

    world.Step(1.0 / 120.0, 8);
    const Body& outA = world.GetBody(aId);
    const Body& outB = world.GetBody(bId);
    assert(std::isfinite(outA.position.x) && std::isfinite(outA.position.y) && std::isfinite(outA.position.z));
    assert(std::isfinite(outB.position.x) && std::isfinite(outB.position.y) && std::isfinite(outB.position.z));
    }

    {
    World world({0.0, -9.81, 0.0});
    ContactSolverConfig config = world.GetContactSolverConfig();
    config.bounceVelocityThreshold = 2.0;
    config.restitutionSuppressionSpeed = 1.5;
    config.restitutionVelocityCutoff = 1.25;
    config.staticFrictionSpeedThreshold = 0.04;
    config.staticToDynamicTransitionSpeed = 0.15;
    config.frictionBudgetNormalSupportSource = FrictionBudgetNormalSupportSource::BlendedSelectedPairAndManifold;
    config.frictionBudgetSelectedPairBlendWeight = 3.5;
    config.useSplitImpulse = true;
    world.SetContactSolverConfig(config);
    const ContactSolverConfig roundtrip = world.GetContactSolverConfig();
    assert(roundtrip.useSplitImpulse);
    assert(std::abs(roundtrip.bounceVelocityThreshold - 2.0) < 1e-6);
    assert(std::abs(roundtrip.restitutionSuppressionSpeed - 1.5) < 1e-6);
    assert(std::abs(roundtrip.restitutionVelocityCutoff - 1.25) < 1e-6);
    assert(std::abs(roundtrip.staticFrictionSpeedThreshold - 0.04) < 1e-6);
    assert(std::abs(roundtrip.staticToDynamicTransitionSpeed - 0.15) < 1e-6);
    assert(roundtrip.frictionBudgetNormalSupportSource == FrictionBudgetNormalSupportSource::BlendedSelectedPairAndManifold);
    assert(std::abs(roundtrip.frictionBudgetSelectedPairBlendWeight - 3.5) < 1e-6);
    assert(!roundtrip.useFace4PointNormalBlock);

    Body plane;
    plane.shape = ShapeType::Plane;
    plane.restitution = 1.0;
    world.CreateBody(plane);

    Body sphere;
    sphere.shape = ShapeType::Sphere;
    sphere.position = {0.0, 0.8, 0.0};
    sphere.velocity = {0.0, -1.0, 0.0};
    sphere.radius = 0.25;
    sphere.mass = 1.0;
    sphere.restitution = 1.0;
    const auto id = world.CreateBody(sphere);

    for (int i = 0; i < 180; ++i) {
        world.Step(1.0 / 120.0, 10);
    }
    const Body& settled = world.GetBody(id);
    assert(std::isfinite(settled.position.y));
    assert(settled.position.y > 0.18);
    }

    {
    World world({0.0, 0.0, 0.0});
    JointSolverConfig jointConfig = world.GetJointSolverConfig();
    jointConfig.useBlockSolver = true;
    jointConfig.blockDeterminantEpsilon = 1e-9;
    jointConfig.blockDiagonalMinimum = 1e-7;
    jointConfig.blockConditionEstimateMax = 1e6;
    jointConfig.hingeAnchorBiasFactor = 0.25;
    jointConfig.hingeAnchorDampingFactor = 0.12;
    world.SetJointSolverConfig(jointConfig);
    const JointSolverConfig roundtrip = world.GetJointSolverConfig();
    assert(roundtrip.useBlockSolver);
    assert(std::abs(roundtrip.blockDeterminantEpsilon - 1e-9) < 1e-12);
    assert(std::abs(roundtrip.blockDiagonalMinimum - 1e-7) < 1e-10);
    assert(std::abs(roundtrip.blockConditionEstimateMax - 1e6) < 1e-3);
    assert(std::abs(roundtrip.hingeAnchorBiasFactor - 0.25) < 1e-6);
    assert(std::abs(roundtrip.hingeAnchorDampingFactor - 0.12) < 1e-6);
    assert(roundtrip.servoPositionPasses == 4u);

    Body heavy;
    heavy.shape = ShapeType::Sphere;
    heavy.radius = 0.25;
    heavy.mass = 50.0;
    heavy.position = {-0.35, 0.0, 0.0};
    const auto heavyId = world.CreateBody(heavy);

    Body light = heavy;
    light.mass = 0.5;
    light.position = {0.35, 0.0, 0.0};
    const auto lightId = world.CreateBody(light);

    const auto hingeId = world.CreateHingeJoint(heavyId, lightId, {0.0, 0.0, 0.0}, {0.0, 1.0, 0.0});
    assert(hingeId != World::kInvalidJointId);
    world.GetBody(lightId).velocity = {0.0, 1.5, 0.0};

    for (int i = 0; i < 180; ++i) {
        world.Step(1.0 / 120.0, 12);
    }

    const Body& outHeavy = world.GetBody(heavyId);
    const Body& outLight = world.GetBody(lightId);
    assert(std::isfinite(outHeavy.position.x) && std::isfinite(outHeavy.position.y) && std::isfinite(outHeavy.position.z));
    assert(std::isfinite(outLight.position.x) && std::isfinite(outLight.position.y) && std::isfinite(outLight.position.z));
    assert(Length(outLight.position - outHeavy.position) < 1.2);
    }

    {
    World world({0.0, -9.81, 0.0});

    Body plane;
    plane.shape = ShapeType::Plane;
    world.CreateBody(plane);

    Body sphere;
    sphere.shape = ShapeType::Sphere;
    sphere.position = {0.0, 1.0, 0.0};
    sphere.radius = 0.25;
    sphere.mass = 1.0;
    const auto id = world.CreateBody(sphere);

    for (int i = 0; i < 60; ++i) {
        world.Step(1.0 / 60.0, 8);
    }

    const Body& out = world.GetBody(id);
    assert(out.position.y > -0.01);
    }

    {
    World world({0.0, 0.0, 0.0});

    Body a;
    a.shape = ShapeType::Capsule;
    a.position = {0.0, 0.0, 0.0};
    a.radius = 0.25;
    a.halfHeight = 0.75;
    a.mass = 1.0;
    const auto aId = world.CreateBody(a);

    Body b = a;
    b.position = {0.0, 0.0, 0.0};
    const auto bId = world.CreateBody(b);

    for (int i = 0; i < 120; ++i) {
        world.Step(1.0 / 120.0, 12);
    }

    const Body& outA = world.GetBody(aId);
    const Body& outB = world.GetBody(bId);
    assert(std::isfinite(outA.position.x) && std::isfinite(outA.position.y) && std::isfinite(outA.position.z));
    assert(std::isfinite(outB.position.x) && std::isfinite(outB.position.y) && std::isfinite(outB.position.z));
    assert(Length(outB.position - outA.position) > 0.001);
    }

    {
    auto runFastSphereAtThinBox = []() {
        World world({0.0, 0.0, 0.0});

        Body target;
        target.shape = ShapeType::Box;
        target.isStatic = true;
        target.position = {0.0, 0.8, 0.0};
        target.halfExtents = {0.08, 0.45, 0.45};
        world.CreateBody(target);

        Body sphere;
        sphere.shape = ShapeType::Sphere;
        sphere.radius = 0.2;
        sphere.position = {-3.0, 0.8, 0.0};
        sphere.velocity = {70.0, 0.0, 0.0};
        sphere.mass = 1.0;
        sphere.restitution = 0.0;
        const auto sphereId = world.CreateBody(sphere);

        for (int i = 0; i < 20; ++i) {
            world.Step(1.0 / 60.0, 8);
        }

        return world.GetBody(sphereId).position;
    };

    const Vec3 firstRun = runFastSphereAtThinBox();
    const Vec3 secondRun = runFastSphereAtThinBox();

    assert(firstRun.x <= 0.30);
    assert(std::abs(firstRun.x - secondRun.x) <= 1e-5);
    assert(std::abs(firstRun.y - secondRun.y) <= 1e-5);
    assert(std::abs(firstRun.z - secondRun.z) <= 1e-5);
    }

    {
    World world({0.0, 0.0, 0.0});

    Body left;
    left.shape = ShapeType::Sphere;
    left.radius = 0.25;
    left.mass = 1.0;
    left.position = {-0.7, 0.0, 0.0};
    const auto leftId = world.CreateBody(left);

    Body mid = left;
    mid.position = {0.0, 0.0, 0.0};
    const auto midId = world.CreateBody(mid);

    Body right = left;
    right.position = {0.7, 0.0, 0.0};
    const auto rightId = world.CreateBody(right);

    world.CreateDistanceJoint(leftId, midId, world.GetBody(leftId).position, world.GetBody(midId).position, 1.0, 0.2);
    world.CreateDistanceJoint(midId, rightId, world.GetBody(midId).position, world.GetBody(rightId).position, 1.0, 0.2);

    for (int i = 0; i < 180; ++i) {
        world.Step(1.0 / 120.0, 16);
    }

    assert(world.GetBody(leftId).isSleeping);
    assert(world.GetBody(midId).isSleeping);
    assert(world.GetBody(rightId).isSleeping);

    world.AddForce(leftId, {50.0, 0.0, 0.0});
    world.Step(1.0 / 120.0, 16);

    assert(!world.GetBody(leftId).isSleeping);
    assert(!world.GetBody(midId).isSleeping);
    assert(!world.GetBody(rightId).isSleeping);
    }

    {
    auto captureSelectedPairSequence = []() {
        World world({0.0, -9.81, 0.0});
        ContactSolverConfig cfg = world.GetContactSolverConfig();
        cfg.useBlockSolver = true;
        cfg.useSplitImpulse = true;
        cfg.penetrationSlop = 0.005;
        cfg.splitImpulseCorrectionFactor = 0.85;
        world.SetContactSolverConfig(cfg);

        Body plane;
        plane.shape = ShapeType::Plane;
        world.CreateBody(plane);

        Body box;
        box.shape = ShapeType::Box;
        box.halfExtents = {0.55, 0.20, 0.35};
        box.mass = 2.0;
        box.position = {0.0, 1.1, 0.0};
        box.orientation = Normalize(Quat{0.9990482, 0.0, 0.0436194, 0.0});
        box.velocity = {0.45, 0.0, 0.12};
        box.angularVelocity = {0.0, 0.3, 0.0};
        world.CreateBody(box);

        std::vector<std::uint64_t> sequence;
        sequence.reserve(640);
        std::uint64_t previousPair = 0;
        int transitions = 0;
        for (int i = 0; i < 640; ++i) {
            world.Step(1.0 / 120.0, 16);
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

    {
    using minphys3d::ContactKey;
    using minphys3d::ContactKeyHash;
    using minphys3d::ManifoldKey;
    using minphys3d::ManifoldKeyHash;
    using minphys3d::PersistentPointKey;
    using minphys3d::PersistentPointKeyHash;

    const ContactKey k1{1u, 65537u, 9u, 0x1234u};
    const ContactKey k2{1u, 65537u, 9u, 0x1234u};
    const ContactKey k3{1u, 65537u, 9u, 0x1235u};
    std::unordered_set<ContactKey, ContactKeyHash> contacts;
    assert(contacts.insert(k1).second);
    assert(!contacts.insert(k2).second);
    assert(contacts.insert(k3).second);
    assert(contacts.size() == 2u);

    const ManifoldKey m1{1u, 2u, 4u};
    const ManifoldKey m2{1u, 2u, 5u};
    std::unordered_set<ManifoldKey, ManifoldKeyHash> manifolds;
    assert(manifolds.insert(m1).second);
    assert(manifolds.insert(m2).second);
    assert(manifolds.size() == 2u);

    std::unordered_set<PersistentPointKey, PersistentPointKeyHash> points;
    assert(points.insert(PersistentPointKey{m1, 0xabu, 0u}).second);
    assert(points.insert(PersistentPointKey{m1, 0xabu, 1u}).second);
    assert(points.insert(PersistentPointKey{m2, 0xabu, 0u}).second);
    assert(points.size() == 3u);
    }

    {
    auto runFrequentFeatureReplay = []() {
        World world({0.0, -9.81, 0.0});
        ContactSolverConfig cfg = world.GetContactSolverConfig();
        cfg.useBlockSolver = true;
        cfg.useSplitImpulse = true;
        world.SetContactSolverConfig(cfg);

        Body plane;
        plane.shape = ShapeType::Plane;
        world.CreateBody(plane);

        std::vector<std::uint32_t> ids;
        for (int z = 0; z < 4; ++z) {
            for (int x = 0; x < 4; ++x) {
                Body sphere;
                sphere.shape = ShapeType::Sphere;
                sphere.radius = 0.18;
                sphere.mass = 1.0;
                sphere.position = {-1.2 + 0.8 * static_cast<float>(x), 0.8 + 0.45 * static_cast<float>(z), 0.15 * static_cast<float>((x + z) % 2)};
                sphere.velocity = {0.1 * static_cast<float>((x % 2) ? 1 : -1), 0.0, 0.0};
                ids.push_back(world.CreateBody(sphere));
            }
        }

        std::vector<std::uint64_t> replaySignature;
        replaySignature.reserve(320);
        int stableWarmStartContacts = 0;
        for (int step = 0; step < 320; ++step) {
            world.Step(1.0 / 120.0, 16);
            std::unordered_set<PersistentPointKey, PersistentPointKeyHash> uniqueKeys;
            std::uint64_t stepHash = 1469598103934665603ull;

            for (const Manifold& manifold : world.DebugManifolds()) {
                const ManifoldKey manifoldKey{std::min(manifold.a, manifold.b), std::max(manifold.a, manifold.b), manifold.manifoldType};
                std::uint8_t ordinal = 0;
                for (const Contact& contact : manifold.contacts) {
                    const PersistentPointKey pointKey{manifoldKey, contact.featureKey, ordinal++};
                    assert(uniqueKeys.insert(pointKey).second);
                    if (step > 60 && contact.persistenceAge >= 2 && std::abs(contact.normalImpulseSum) > 1e-5) {
                        ++stableWarmStartContacts;
                    }

                    stepHash ^= static_cast<std::uint64_t>(pointKey.manifold.loBody + 1u) * 1099511628211ull;
                    stepHash ^= static_cast<std::uint64_t>(pointKey.manifold.hiBody + 7u) * 1469598103934665603ull;
                    stepHash ^= static_cast<std::uint64_t>(pointKey.manifold.manifoldType + 13u);
                    stepHash ^= pointKey.canonicalFeatureId * 1099511628211ull;
                    stepHash ^= static_cast<std::uint64_t>(pointKey.ordinal + 17u);
                    stepHash ^= static_cast<std::uint64_t>(std::lround(contact.normalImpulseSum * 10000.0));
                    stepHash ^= static_cast<std::uint64_t>(contact.persistenceAge + 31u);
                }
            }
            replaySignature.push_back(stepHash);
        }

        assert(stableWarmStartContacts > 900);
        return replaySignature;
    };

    const std::vector<std::uint64_t> first = runFrequentFeatureReplay();
    const std::vector<std::uint64_t> second = runFrequentFeatureReplay();
    assert(first == second);
    }

    {
    auto runPersistenceScenario = [](const Vec3& initialVelocity, const Vec3& angularVelocity, int steps) {
        World world({0.0, -9.81, 0.0});
        ContactSolverConfig cfg = world.GetContactSolverConfig();
        cfg.useBlockSolver = true;
        cfg.useSplitImpulse = true;
        cfg.penetrationSlop = 0.004;
        world.SetContactSolverConfig(cfg);

        Body base;
        base.shape = ShapeType::Box;
        base.isStatic = true;
        base.position = {0.0, 0.0, 0.0};
        base.halfExtents = {1.2, 0.25, 1.2};
        world.CreateBody(base);

        Body top;
        top.shape = ShapeType::Box;
        top.mass = 2.0;
        top.position = {0.0, 0.70, 0.0};
        top.halfExtents = {0.55, 0.35, 0.55};
        top.velocity = initialVelocity;
        top.angularVelocity = angularVelocity;
        const std::uint32_t topId = world.CreateBody(top);

        std::uint64_t matched = 0;
        std::uint64_t dropped = 0;
        std::uint64_t created = 0;
        Real maxChurn = 0.0;
        int persistentSteps = 0;
        for (int i = 0; i < steps; ++i) {
            if (i == steps / 2) {
                world.GetBody(topId).velocity.x *= -1.0;
            }
            world.Step(1.0 / 120.0, 14);
            const World::PersistenceMatchDiagnostics& diag = world.GetPersistenceMatchDiagnostics();
            matched += diag.matchedPoints;
            dropped += diag.droppedPoints;
            created += diag.newPoints;
            maxChurn = std::max(maxChurn, diag.churnRatio);
            if (diag.matchedPoints > 0u) {
                ++persistentSteps;
            }
        }
        assert(matched + created > 40u);
        assert(dropped < static_cast<std::uint64_t>(steps) * 8u);
        assert(std::isfinite(maxChurn));
        assert(persistentSteps > 10);
    };

    // Resting box-on-box.
    runPersistenceScenario({0.0, 0.0, 0.0}, {0.02, 0.0, -0.02}, 260);
    // Mild rocking.
    runPersistenceScenario({0.0, 0.0, 0.0}, {0.8, 0.0, 0.5}, 300);
    // Stick-slip with ordering churn from direction reversal.
    runPersistenceScenario({0.9, 0.0, 0.0}, {0.0, 0.6, 0.0}, 320);
    }

    {
    ContactSolverConfig defaults{};
    assert(ValidateContactSolverConfig(defaults));

    ContactSolverConfig invalid = defaults;
    invalid.toi.max_iterations = 0;
    assert(!ValidateContactSolverConfig(invalid));

    invalid = defaults;
    invalid.toi.min_time_step = -1.0;
    assert(!ValidateContactSolverConfig(invalid));

    invalid = defaults;
    invalid.block4.symmetry_tolerance = -0.1;
    assert(!ValidateContactSolverConfig(invalid));

    const ContactSolverConfig sanitized = SanitizeContactSolverConfig(invalid);
    assert(ValidateContactSolverConfig(sanitized));
    assert(std::abs(sanitized.block4.symmetry_tolerance - defaults.block4.symmetry_tolerance) < 1e-6);
    }

    {
    auto runFastSphereAtThinBoxWithConfig = [](const ContactSolverConfig& config) {
        World world({0.0, 0.0, 0.0});
        world.SetContactSolverConfig(config);

        Body target;
        target.shape = ShapeType::Box;
        target.isStatic = true;
        target.position = {0.0, 0.8, 0.0};
        target.halfExtents = {0.08, 0.45, 0.45};
        world.CreateBody(target);

        Body sphere;
        sphere.shape = ShapeType::Sphere;
        sphere.radius = 0.2;
        sphere.position = {-3.0, 0.8, 0.0};
        sphere.velocity = {70.0, 0.0, 0.0};
        sphere.mass = 1.0;
        sphere.restitution = 0.0;
        const auto sphereId = world.CreateBody(sphere);

        for (int i = 0; i < 20; ++i) {
            world.Step(1.0 / 60.0, 8);
        }

        const Body& out = world.GetBody(sphereId);
        return std::array<Real, 6>{out.position.x, out.position.y, out.position.z, out.velocity.x, out.velocity.y, out.velocity.z};
    };

    ContactSolverConfig legacyConfig{};
    legacyConfig.toi.max_iterations = 8;
    legacyConfig.toi.min_time_step = 1e-6;
    legacyConfig.block4.symmetry_tolerance = 2e-3;
    legacyConfig.ordering.support_depth_relaxation_passes = 4;

    const std::array<Real, 6> defaultOutcome = runFastSphereAtThinBoxWithConfig(ContactSolverConfig{});
    const std::array<Real, 6> legacyOutcome = runFastSphereAtThinBoxWithConfig(legacyConfig);
    for (std::size_t i = 0; i < defaultOutcome.size(); ++i) {
        assert(std::abs(defaultOutcome[i] - legacyOutcome[i]) <= 1e-6);
    }
    }

    return 0;
}
