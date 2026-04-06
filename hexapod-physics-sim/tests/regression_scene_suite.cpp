#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "minphys3d/minphys3d.hpp"

namespace {

using minphys3d::Body;
using minphys3d::Dot;
using minphys3d::Length;
using minphys3d::LengthSquared;
using minphys3d::Normalize;
using minphys3d::Quat;
using minphys3d::Rotate;
using minphys3d::ShapeType;
using minphys3d::Vec3;
using minphys3d::World;

struct PositionExpectation {
    std::uint32_t bodyId;
    Vec3 expected;
    float tolerance;
};

struct ScenarioResult {
    std::string name;
    bool pass = true;
    bool exploded = false;
    bool hasNan = false;
    bool tunneled = false;
    float maxPenetration = 0.0f;
    float energyDrift = 0.0f;
    int sleepingBodiesAfterSettle = 0;
    bool finalPositionsWithinTolerance = true;
};

struct ScenarioConfig {
    std::string name;
    World world;
    std::vector<std::uint32_t> dynamicIds;
    std::vector<std::pair<std::uint32_t, std::uint32_t>> penetrationPairs;
    std::vector<PositionExpectation> expectedFinalPositions;
    std::vector<std::uint32_t> settleCountIds;
    std::uint32_t tunnelProbeBody = std::numeric_limits<std::uint32_t>::max();
    std::uint32_t tunnelTargetBody = std::numeric_limits<std::uint32_t>::max();
    float dt = 1.0f / 120.0f;
    int solverIterations = 16;
    int steps = 600;
    float maxEnergyDriftAllowed = 60.0f;
    float maxPenetrationAllowed = 0.20f;
};

bool IsFinite(const Vec3& v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

float ComputePenetrationSpherePlane(const Body& sphere, const Body& plane) {
    const Vec3 n = Normalize(plane.planeNormal);
    const float signedDistance = Dot(n, sphere.position) - plane.planeOffset;
    return std::max(0.0f, sphere.radius - signedDistance);
}

float ComputePenetrationCapsulePlane(const Body& capsule, const Body& plane) {
    const Vec3 n = Normalize(plane.planeNormal);
    const Vec3 axis = Normalize(Rotate(capsule.orientation, {0.0f, 1.0f, 0.0f}));
    const Vec3 ends[2] = {capsule.position - axis * capsule.halfHeight, capsule.position + axis * capsule.halfHeight};
    float penetration = 0.0f;
    for (const Vec3& end : ends) {
        const float signedDistance = Dot(n, end) - plane.planeOffset;
        penetration = std::max(penetration, std::max(0.0f, capsule.radius - signedDistance));
    }
    return penetration;
}

float ProjectBoxOntoAxis(const Body& box, const Vec3& axis) {
    const Vec3 u0 = Rotate(box.orientation, {1.0f, 0.0f, 0.0f});
    const Vec3 u1 = Rotate(box.orientation, {0.0f, 1.0f, 0.0f});
    const Vec3 u2 = Rotate(box.orientation, {0.0f, 0.0f, 1.0f});
    return std::abs(Dot(axis, u0)) * box.halfExtents.x
         + std::abs(Dot(axis, u1)) * box.halfExtents.y
         + std::abs(Dot(axis, u2)) * box.halfExtents.z;
}

float ComputePenetrationBoxPlane(const Body& box, const Body& plane) {
    const Vec3 n = Normalize(plane.planeNormal);
    const float projectionRadius = ProjectBoxOntoAxis(box, n);
    const float signedDistance = Dot(n, box.position) - plane.planeOffset;
    return std::max(0.0f, projectionRadius - signedDistance);
}

Vec3 ClosestPointOnBox(const Body& box, const Vec3& point) {
    const Quat invQ = minphys3d::Conjugate(Normalize(box.orientation));
    const Vec3 local = Rotate(invQ, point - box.position);
    const Vec3 clamped = {
        std::clamp(local.x, -box.halfExtents.x, box.halfExtents.x),
        std::clamp(local.y, -box.halfExtents.y, box.halfExtents.y),
        std::clamp(local.z, -box.halfExtents.z, box.halfExtents.z),
    };
    return box.position + Rotate(box.orientation, clamped);
}

float ComputePenetrationSphereBox(const Body& sphere, const Body& box) {
    const Vec3 closest = ClosestPointOnBox(box, sphere.position);
    const float distSq = LengthSquared(sphere.position - closest);
    if (distSq > sphere.radius * sphere.radius) {
        return 0.0f;
    }
    const float dist = std::sqrt(std::max(distSq, minphys3d::kEpsilon));
    return std::max(0.0f, sphere.radius - dist);
}

float ComputePenetrationBoxBox(const Body& a, const Body& b) {
    const auto aAABB = a.ComputeAABB();
    const auto bAABB = b.ComputeAABB();
    const float px = std::min(aAABB.max.x, bAABB.max.x) - std::max(aAABB.min.x, bAABB.min.x);
    const float py = std::min(aAABB.max.y, bAABB.max.y) - std::max(aAABB.min.y, bAABB.min.y);
    const float pz = std::min(aAABB.max.z, bAABB.max.z) - std::max(aAABB.min.z, bAABB.min.z);
    if (px <= 0.0f || py <= 0.0f || pz <= 0.0f) {
        return 0.0f;
    }
    return std::min({px, py, pz});
}

std::pair<float, float> ClosestSegmentParameters(const Vec3& p1, const Vec3& q1, const Vec3& p2, const Vec3& q2) {
    const Vec3 d1 = q1 - p1;
    const Vec3 d2 = q2 - p2;
    const Vec3 r = p1 - p2;
    const float aLen = Dot(d1, d1);
    const float eLen = Dot(d2, d2);
    const float f = Dot(d2, r);

    float s = 0.0f;
    float t = 0.0f;
    if (aLen <= minphys3d::kEpsilon && eLen <= minphys3d::kEpsilon) {
        return {0.0f, 0.0f};
    }
    if (aLen <= minphys3d::kEpsilon) {
        t = std::clamp(f / eLen, 0.0f, 1.0f);
        return {0.0f, t};
    }

    const float cTerm = Dot(d1, r);
    if (eLen <= minphys3d::kEpsilon) {
        s = std::clamp(-cTerm / aLen, 0.0f, 1.0f);
        return {s, 0.0f};
    }

    const float bDot = Dot(d1, d2);
    const float denom = aLen * eLen - bDot * bDot;
    if (std::abs(denom) > minphys3d::kEpsilon) {
        s = std::clamp((bDot * f - cTerm * eLen) / denom, 0.0f, 1.0f);
    }

    t = (bDot * s + f) / eLen;
    if (t < 0.0f) {
        t = 0.0f;
        s = std::clamp(-cTerm / aLen, 0.0f, 1.0f);
    } else if (t > 1.0f) {
        t = 1.0f;
        s = std::clamp((bDot - cTerm) / aLen, 0.0f, 1.0f);
    }
    return {s, t};
}

float ComputePenetrationCapsuleCapsule(const Body& a, const Body& b) {
    const Vec3 axisA = Normalize(Rotate(a.orientation, {0.0f, 1.0f, 0.0f}));
    const Vec3 axisB = Normalize(Rotate(b.orientation, {0.0f, 1.0f, 0.0f}));
    const Vec3 p1 = a.position - axisA * a.halfHeight;
    const Vec3 q1 = a.position + axisA * a.halfHeight;
    const Vec3 p2 = b.position - axisB * b.halfHeight;
    const Vec3 q2 = b.position + axisB * b.halfHeight;
    const auto [s, t] = ClosestSegmentParameters(p1, q1, p2, q2);
    const Vec3 c1 = p1 + (q1 - p1) * s;
    const Vec3 c2 = p2 + (q2 - p2) * t;
    const float distance = Length(c2 - c1);
    return std::max(0.0f, (a.radius + b.radius) - distance);
}

float ComputePairPenetration(const Body& a, const Body& b) {
    if (a.shape == ShapeType::Sphere && b.shape == ShapeType::Plane) return ComputePenetrationSpherePlane(a, b);
    if (a.shape == ShapeType::Plane && b.shape == ShapeType::Sphere) return ComputePenetrationSpherePlane(b, a);
    if (a.shape == ShapeType::Capsule && b.shape == ShapeType::Plane) return ComputePenetrationCapsulePlane(a, b);
    if (a.shape == ShapeType::Plane && b.shape == ShapeType::Capsule) return ComputePenetrationCapsulePlane(b, a);
    if (a.shape == ShapeType::Box && b.shape == ShapeType::Plane) return ComputePenetrationBoxPlane(a, b);
    if (a.shape == ShapeType::Plane && b.shape == ShapeType::Box) return ComputePenetrationBoxPlane(b, a);
    if (a.shape == ShapeType::Sphere && b.shape == ShapeType::Box) return ComputePenetrationSphereBox(a, b);
    if (a.shape == ShapeType::Box && b.shape == ShapeType::Sphere) return ComputePenetrationSphereBox(b, a);
    if (a.shape == ShapeType::Box && b.shape == ShapeType::Box) return ComputePenetrationBoxBox(a, b);
    if (a.shape == ShapeType::Capsule && b.shape == ShapeType::Capsule) return ComputePenetrationCapsuleCapsule(a, b);
    return 0.0f;
}

float ComputeTotalEnergy(const World& world, const std::vector<std::uint32_t>& bodyIds, const Vec3& gravity) {
    float total = 0.0f;
    for (std::uint32_t id : bodyIds) {
        const Body& body = world.GetBody(id);
        if (body.invMass == 0.0f) {
            continue;
        }
        const float kinetic = 0.5f * body.mass * LengthSquared(body.velocity);
        const float potential = -body.mass * Dot(gravity, body.position);
        total += kinetic + potential;
    }
    return total;
}

ScenarioResult RunScenario(ScenarioConfig cfg) {
    ScenarioResult result;
    result.name = cfg.name;

    constexpr float kExplodePosition = 5000.0f;
    const Vec3 gravity{0.0f, -9.81f, 0.0f};
    const float initialEnergy = ComputeTotalEnergy(cfg.world, cfg.dynamicIds, gravity);

    float minProbeTargetDistance = std::numeric_limits<float>::infinity();
    bool probeEverPastTarget = false;

    for (int step = 0; step < cfg.steps; ++step) {
        cfg.world.Step(cfg.dt, cfg.solverIterations);

        for (std::uint32_t id : cfg.dynamicIds) {
            const Body& body = cfg.world.GetBody(id);
            if (!IsFinite(body.position) || !IsFinite(body.velocity)) {
                result.hasNan = true;
            }
            if (Length(body.position) > kExplodePosition || Length(body.velocity) > kExplodePosition) {
                result.exploded = true;
            }
        }

        for (const auto& [aId, bId] : cfg.penetrationPairs) {
            const Body& a = cfg.world.GetBody(aId);
            const Body& b = cfg.world.GetBody(bId);
            result.maxPenetration = std::max(result.maxPenetration, ComputePairPenetration(a, b));
        }

        if (cfg.tunnelProbeBody != std::numeric_limits<std::uint32_t>::max()) {
            const Body& probe = cfg.world.GetBody(cfg.tunnelProbeBody);
            const Body& target = cfg.world.GetBody(cfg.tunnelTargetBody);
            const float distance = Length(probe.position - target.position);
            minProbeTargetDistance = std::min(minProbeTargetDistance, distance);
            if (probe.position.x > target.position.x + 0.8f) {
                probeEverPastTarget = true;
            }
        }
    }

    const float finalEnergy = ComputeTotalEnergy(cfg.world, cfg.dynamicIds, gravity);
    result.energyDrift = std::abs(finalEnergy - initialEnergy);

    int sleeping = 0;
    for (std::uint32_t id : cfg.settleCountIds) {
        if (cfg.world.GetBody(id).isSleeping) {
            ++sleeping;
        }
    }
    result.sleepingBodiesAfterSettle = sleeping;

    for (const PositionExpectation& expectation : cfg.expectedFinalPositions) {
        const Body& body = cfg.world.GetBody(expectation.bodyId);
        const float delta = Length(body.position - expectation.expected);
        if (delta > expectation.tolerance) {
            result.finalPositionsWithinTolerance = false;
            break;
        }
    }

    if (cfg.tunnelProbeBody != std::numeric_limits<std::uint32_t>::max()) {
        const Body& probe = cfg.world.GetBody(cfg.tunnelProbeBody);
        const Body& target = cfg.world.GetBody(cfg.tunnelTargetBody);
        const float targetRadius =
            (target.shape == ShapeType::Sphere) ? target.radius :
            (target.shape == ShapeType::Box) ? std::max({target.halfExtents.x, target.halfExtents.y, target.halfExtents.z}) : 0.5f;
        const bool neverCloseToTarget = minProbeTargetDistance > (targetRadius + 0.25f);
        result.tunneled = probeEverPastTarget && neverCloseToTarget && probe.position.x > target.position.x;
    }

    result.pass = !result.exploded
        && !result.hasNan
        && !result.tunneled
        && result.finalPositionsWithinTolerance
        && (result.maxPenetration <= cfg.maxPenetrationAllowed)
        && (result.energyDrift <= cfg.maxEnergyDriftAllowed);

    return result;
}

Body MakePlane() {
    Body plane;
    plane.shape = ShapeType::Plane;
    plane.planeNormal = {0.0f, 1.0f, 0.0f};
    plane.planeOffset = 0.0f;
    plane.restitution = 0.05f;
    plane.staticFriction = 0.9f;
    plane.dynamicFriction = 0.65f;
    return plane;
}

ScenarioConfig BuildSphereDropOnPlane() {
    ScenarioConfig cfg;
    cfg.name = "sphere drop on plane";
    cfg.world = World({0.0f, -9.81f, 0.0f});
    const std::uint32_t plane = cfg.world.CreateBody(MakePlane());

    Body sphere;
    sphere.shape = ShapeType::Sphere;
    sphere.position = {0.0f, 3.0f, 0.0f};
    sphere.radius = 0.5f;
    sphere.mass = 1.0f;
    const std::uint32_t sphereId = cfg.world.CreateBody(sphere);

    cfg.dynamicIds = {sphereId};
    cfg.penetrationPairs = {{sphereId, plane}};
    cfg.settleCountIds = {sphereId};
    cfg.expectedFinalPositions = {{sphereId, {0.0f, 0.5f, 0.0f}, 0.20f}};
    return cfg;
}

ScenarioConfig BuildBoxStackOnPlane() {
    ScenarioConfig cfg;
    cfg.name = "box stack on plane";
    cfg.world = World({0.0f, -9.81f, 0.0f});
    const std::uint32_t plane = cfg.world.CreateBody(MakePlane());

    Body bottom;
    bottom.shape = ShapeType::Box;
    bottom.halfExtents = {0.4f, 0.4f, 0.4f};
    bottom.position = {0.0f, 0.6f, 0.0f};
    bottom.mass = 3.0f;
    const std::uint32_t bottomId = cfg.world.CreateBody(bottom);

    Body top = bottom;
    top.position = {0.03f, 1.55f, 0.0f};
    top.mass = 1.8f;
    const std::uint32_t topId = cfg.world.CreateBody(top);

    cfg.dynamicIds = {bottomId, topId};
    cfg.penetrationPairs = {{bottomId, plane}, {topId, bottomId}};
    cfg.settleCountIds = {bottomId, topId};
    cfg.expectedFinalPositions = {
        {bottomId, {0.0f, 0.4f, 0.0f}, 100.0f},
        {topId, {0.0f, 1.2f, 0.0f}, 100.0f},
    };
    cfg.steps = 700;
    cfg.maxPenetrationAllowed = 0.35f;
    return cfg;
}

ScenarioConfig BuildBoxOnBoxRestContact() {
    ScenarioConfig cfg;
    cfg.name = "box-on-box rest contact";
    cfg.world = World({0.0f, -9.81f, 0.0f});

    Body base;
    base.shape = ShapeType::Box;
    base.isStatic = true;
    base.halfExtents = {0.7f, 0.25f, 0.7f};
    base.position = {0.0f, 0.25f, 0.0f};
    const std::uint32_t baseId = cfg.world.CreateBody(base);

    Body box;
    box.shape = ShapeType::Box;
    box.halfExtents = {0.4f, 0.4f, 0.4f};
    box.position = {0.0f, 1.8f, 0.0f};
    box.mass = 2.5f;
    const std::uint32_t boxId = cfg.world.CreateBody(box);

    cfg.dynamicIds = {boxId};
    cfg.penetrationPairs = {{boxId, baseId}};
    cfg.settleCountIds = {boxId};
    cfg.expectedFinalPositions = {{boxId, {0.0f, 0.9f, 0.0f}, 100.0f}};
    return cfg;
}

ScenarioConfig BuildCapsuleRestingOnPlane() {
    ScenarioConfig cfg;
    cfg.name = "capsule resting on plane";
    cfg.world = World({0.0f, -9.81f, 0.0f});
    const std::uint32_t plane = cfg.world.CreateBody(MakePlane());

    Body capsule;
    capsule.shape = ShapeType::Capsule;
    capsule.position = {0.0f, 2.0f, 0.0f};
    capsule.radius = 0.25f;
    capsule.halfHeight = 0.65f;
    capsule.mass = 1.6f;
    const std::uint32_t capsuleId = cfg.world.CreateBody(capsule);

    cfg.dynamicIds = {capsuleId};
    cfg.penetrationPairs = {{capsuleId, plane}};
    cfg.settleCountIds = {capsuleId};
    cfg.expectedFinalPositions = {{capsuleId, {0.0f, 0.90f, 0.0f}, 0.25f}};
    return cfg;
}


ScenarioConfig BuildSphereFiredFastAtPlane() {
    ScenarioConfig cfg;
    cfg.name = "sphere fired fast at plane";
    cfg.world = World({0.0f, -9.81f, 0.0f});
    const std::uint32_t plane = cfg.world.CreateBody(MakePlane());

    Body sphere;
    sphere.shape = ShapeType::Sphere;
    sphere.radius = 0.2f;
    sphere.position = {0.0f, 1.4f, 0.0f};
    sphere.velocity = {0.0f, -35.0f, 0.0f};
    sphere.mass = 1.0f;
    sphere.restitution = 0.15f;
    const std::uint32_t sphereId = cfg.world.CreateBody(sphere);

    cfg.dynamicIds = {sphereId};
    cfg.penetrationPairs = {{sphereId, plane}};
    cfg.settleCountIds = {sphereId};
    cfg.expectedFinalPositions = {{sphereId, {0.0f, 0.2f, 0.0f}, 0.35f}};
    cfg.steps = 450;
    cfg.maxPenetrationAllowed = 0.25f;
    cfg.maxEnergyDriftAllowed = 900.0f;
    return cfg;
}

ScenarioConfig BuildSphereFiredFastAtBox() {
    ScenarioConfig cfg;
    cfg.name = "sphere fired fast at box";
    cfg.world = World({0.0f, -9.81f, 0.0f});

    Body target;
    target.shape = ShapeType::Box;
    target.isStatic = true;
    target.position = {0.0f, 0.8f, 0.0f};
    target.halfExtents = {0.45f, 0.45f, 0.45f};
    const std::uint32_t targetId = cfg.world.CreateBody(target);

    Body sphere;
    sphere.shape = ShapeType::Sphere;
    sphere.radius = 0.2f;
    sphere.position = {-6.0f, 0.8f, 0.0f};
    sphere.velocity = {40.0f, 0.0f, 0.0f};
    sphere.mass = 1.0f;
    sphere.restitution = 0.1f;
    const std::uint32_t sphereId = cfg.world.CreateBody(sphere);

    cfg.dynamicIds = {sphereId};
    cfg.penetrationPairs = {{sphereId, targetId}};
    cfg.settleCountIds = {sphereId};
    cfg.expectedFinalPositions = {{sphereId, {-0.9f, 0.8f, 0.0f}, 100.0f}};
    cfg.tunnelProbeBody = sphereId;
    cfg.tunnelTargetBody = targetId;
    cfg.steps = 240;
    cfg.maxPenetrationAllowed = 0.35f;
    cfg.maxEnergyDriftAllowed = 1200.0f;
    return cfg;
}

ScenarioConfig BuildDistanceJointPendulum() {
    ScenarioConfig cfg;
    cfg.name = "distance-joint pendulum";
    cfg.world = World({0.0f, -9.81f, 0.0f});

    Body anchor;
    anchor.shape = ShapeType::Box;
    anchor.isStatic = true;
    anchor.halfExtents = {0.1f, 0.1f, 0.1f};
    anchor.position = {0.0f, 2.5f, 0.0f};
    const std::uint32_t anchorId = cfg.world.CreateBody(anchor);

    Body bob;
    bob.shape = ShapeType::Sphere;
    bob.radius = 0.2f;
    bob.mass = 1.2f;
    bob.position = {1.4f, 1.2f, 0.0f};
    bob.velocity = {0.0f, 0.0f, 0.0f};
    const std::uint32_t bobId = cfg.world.CreateBody(bob);

    cfg.world.CreateDistanceJoint(anchorId, bobId, {0.0f, 2.5f, 0.0f}, bob.position, 0.25f, 0.18f);

    cfg.dynamicIds = {bobId};
    cfg.penetrationPairs = {};
    cfg.settleCountIds = {bobId};
    cfg.expectedFinalPositions = {{bobId, {0.0f, 1.1f, 0.0f}, 1.7f}};
    cfg.steps = 800;
    cfg.maxEnergyDriftAllowed = 150.0f;
    return cfg;
}

ScenarioConfig BuildHingeConnectedTwoBoxSystem() {
    ScenarioConfig cfg;
    cfg.name = "hinge-connected two-box system";
    cfg.world = World({0.0f, -9.81f, 0.0f});
    cfg.world.CreateBody(MakePlane());

    Body base;
    base.shape = ShapeType::Box;
    base.isStatic = true;
    base.position = {0.0f, 1.8f, 0.0f};
    base.halfExtents = {0.2f, 0.2f, 0.2f};
    const std::uint32_t baseId = cfg.world.CreateBody(base);

    Body linkA;
    linkA.shape = ShapeType::Box;
    linkA.position = {0.6f, 1.8f, 0.0f};
    linkA.halfExtents = {0.35f, 0.12f, 0.12f};
    linkA.mass = 1.4f;
    const std::uint32_t linkAId = cfg.world.CreateBody(linkA);

    Body linkB;
    linkB.shape = ShapeType::Box;
    linkB.position = {1.25f, 1.8f, 0.0f};
    linkB.halfExtents = {0.35f, 0.12f, 0.12f};
    linkB.mass = 1.0f;
    const std::uint32_t linkBId = cfg.world.CreateBody(linkB);

    cfg.world.CreateHingeJoint(baseId, linkAId, {0.25f, 1.8f, 0.0f}, {0.0f, 0.0f, 1.0f});
    cfg.world.CreateHingeJoint(linkAId, linkBId, {0.95f, 1.8f, 0.0f}, {0.0f, 0.0f, 1.0f});

    cfg.dynamicIds = {linkAId, linkBId};
    cfg.penetrationPairs = {};
    cfg.settleCountIds = {linkAId, linkBId};
    cfg.expectedFinalPositions = {
        {linkAId, {0.1f, 1.2f, 0.0f}, 1.4f},
        {linkBId, {0.6f, 0.8f, 0.0f}, 1.8f},
    };
    cfg.steps = 900;
    cfg.maxEnergyDriftAllowed = 200.0f;
    return cfg;
}

void PrintScenario(const ScenarioResult& result) {
    std::cout << (result.pass ? "PASS" : "FAIL") << " | " << result.name << "\n"
              << "  max penetration: " << std::fixed << std::setprecision(5) << result.maxPenetration << "\n"
              << "  total energy drift: " << result.energyDrift << "\n"
              << "  sleeping bodies after settle: " << result.sleepingBodiesAfterSettle << "\n"
              << "  exploded: " << (result.exploded ? "yes" : "no")
              << " | NaN: " << (result.hasNan ? "yes" : "no")
              << " | tunneled: " << (result.tunneled ? "yes" : "no") << "\n"
              << "  final positions within tolerance: " << (result.finalPositionsWithinTolerance ? "yes" : "no") << "\n";
}

} // namespace

int main() {
    std::vector<ScenarioConfig> scenarios;
    scenarios.push_back(BuildSphereDropOnPlane());
    scenarios.push_back(BuildBoxStackOnPlane());
    scenarios.push_back(BuildBoxOnBoxRestContact());
    scenarios.push_back(BuildCapsuleRestingOnPlane());
    scenarios.push_back(BuildSphereFiredFastAtPlane());
    scenarios.push_back(BuildSphereFiredFastAtBox());
    scenarios.push_back(BuildDistanceJointPendulum());
    scenarios.push_back(BuildHingeConnectedTwoBoxSystem());

    bool allPass = true;
    for (ScenarioConfig& scenario : scenarios) {
        const ScenarioResult result = RunScenario(std::move(scenario));
        PrintScenario(result);
        allPass = allPass && result.pass;
    }

    std::cout << "\nRegression scene suite: " << (allPass ? "PASS" : "FAIL") << "\n";
    return allPass ? 0 : 1;
}
