#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>

#include "minphys3d/core/world.hpp"

using namespace minphys3d;

namespace {

bool HasManifoldBetween(const World& world, std::uint32_t a, std::uint32_t b) {
    const std::uint32_t lo = std::min(a, b);
    const std::uint32_t hi = std::max(a, b);
    for (const Manifold& m : world.DebugManifolds()) {
        const std::uint32_t mlo = std::min(m.a, m.b);
        const std::uint32_t mhi = std::max(m.a, m.b);
        if (mlo == lo && mhi == hi && !m.contacts.empty()) {
            return true;
        }
    }
    return false;
}

Quat AxisAngle(const Vec3& axis, float radians) {
    Vec3 n = axis;
    if (LengthSquared(n) <= kEpsilon * kEpsilon) {
        return {1.0f, 0.0f, 0.0f, 0.0f};
    }
    n = Normalize(n);
    const float half = 0.5f * radians;
    const float s = std::sin(half);
    return Normalize(Quat{std::cos(half), n.x * s, n.y * s, n.z * s});
}

} // namespace

int main() {
    {
        World world({0.0f, -9.81f, 0.0f});

        Body plane;
        plane.shape = ShapeType::Plane;
        world.CreateBody(plane);

        Body cyl;
        cyl.shape = ShapeType::Cylinder;
        cyl.radius = 0.2f;
        cyl.halfHeight = 0.2f;
        cyl.mass = 1.0f;
        cyl.restitution = 0.0f;
        cyl.position = {0.0f, 0.55f, 0.0f};
        const std::uint32_t cylId = world.CreateBody(cyl);

        for (int i = 0; i < 200; ++i) {
            world.Step(1.0f / 120.0f, 10);
        }
        const Body& settled = world.GetBody(cylId);
        assert(std::isfinite(settled.position.y));
        assert(settled.position.y > cyl.radius * 0.5f);
    }

    {
        World world({0.0f, 0.0f, 0.0f});

        Body box;
        box.shape = ShapeType::Box;
        box.isStatic = true;
        box.halfExtents = {0.5f, 0.15f, 0.5f};
        box.position = {0.0f, 0.0f, 0.0f};
        const std::uint32_t boxId = world.CreateBody(box);

        Body cyl;
        cyl.shape = ShapeType::Cylinder;
        cyl.radius = 0.18f;
        cyl.halfHeight = 0.22f;
        cyl.mass = 1.0f;
        cyl.restitution = 0.0f;
        cyl.position = {0.0f, 0.28f, 0.0f};
        const std::uint32_t cylId = world.CreateBody(cyl);

        world.Step(1.0f / 120.0f, 12);
        assert(HasManifoldBetween(world, boxId, cylId));
        constexpr std::uint8_t kCylinderBoxManifold = 15u;
        const std::uint32_t lo = std::min(boxId, cylId);
        const std::uint32_t hi = std::max(boxId, cylId);
        bool foundAnalytic = false;
        for (const Manifold& m : world.DebugManifolds()) {
            const std::uint32_t mlo = std::min(m.a, m.b);
            const std::uint32_t mhi = std::max(m.a, m.b);
            if (mlo == lo && mhi == hi && m.manifoldType == kCylinderBoxManifold) {
                foundAnalytic = true;
                assert(m.contacts.size() >= 2u);
                break;
            }
        }
        assert(foundAnalytic);
    }

    {
        World world({0.0f, 0.0f, 0.0f});

        Body cap;
        cap.shape = ShapeType::Capsule;
        cap.isStatic = true;
        cap.radius = 0.12f;
        cap.halfHeight = 0.35f;
        cap.position = {0.0f, 0.0f, 0.0f};
        const std::uint32_t capId = world.CreateBody(cap);

        Body cyl;
        cyl.shape = ShapeType::Cylinder;
        cyl.radius = 0.16f;
        cyl.halfHeight = 0.2f;
        cyl.mass = 1.0f;
        cyl.restitution = 0.0f;
        cyl.position = {0.18f, 0.0f, 0.0f};
        const std::uint32_t cylId = world.CreateBody(cyl);

        for (int i = 0; i < 4; ++i) {
            world.Step(1.0f / 120.0f, 12);
        }
        assert(HasManifoldBetween(world, capId, cylId));
    }

    {
        World world({0.0f, 0.0f, 0.0f});

        Body a;
        a.shape = ShapeType::Cylinder;
        a.isStatic = true;
        a.radius = 0.15f;
        a.halfHeight = 0.25f;
        a.position = {0.0f, 0.0f, 0.0f};
        const std::uint32_t aId = world.CreateBody(a);

        Body b;
        b.shape = ShapeType::Cylinder;
        b.radius = 0.14f;
        b.halfHeight = 0.22f;
        b.mass = 1.0f;
        b.restitution = 0.0f;
        b.position = {0.22f, 0.05f, 0.0f};
        const std::uint32_t bId = world.CreateBody(b);

        world.Step(1.0f / 120.0f, 12);
        assert(HasManifoldBetween(world, aId, bId));
    }

    {
        World world({0.0f, 0.0f, 0.0f});

        Body box;
        box.shape = ShapeType::Box;
        box.isStatic = true;
        box.halfExtents = {0.4f, 0.12f, 0.4f};
        box.position = {0.0f, 0.0f, 0.0f};
        const std::uint32_t boxId = world.CreateBody(box);

        Body hc;
        hc.shape = ShapeType::HalfCylinder;
        hc.radius = 0.2f;
        hc.halfHeight = 0.18f;
        hc.mass = 1.0f;
        hc.restitution = 0.0f;
        hc.orientation = AxisAngle({1.0f, 0.0f, 0.0f}, 3.14159265f * 0.5f);
        hc.position = {0.0f, 0.22f, 0.15f};
        const std::uint32_t hcId = world.CreateBody(hc);

        world.Step(1.0f / 120.0f, 12);
        assert(HasManifoldBetween(world, boxId, hcId));
        constexpr std::uint8_t kHalfCylinderBoxManifold = 19u;
        const std::uint32_t lo = std::min(boxId, hcId);
        const std::uint32_t hi = std::max(boxId, hcId);
        bool foundHcBox = false;
        for (const Manifold& m : world.DebugManifolds()) {
            if (std::min(m.a, m.b) == lo && std::max(m.a, m.b) == hi && m.manifoldType == kHalfCylinderBoxManifold) {
                foundHcBox = true;
                assert(m.contacts.size() >= 2u);
                break;
            }
        }
        assert(foundHcBox);
    }

    {
        World world({0.0f, 0.0f, 0.0f});

        Body sph;
        sph.shape = ShapeType::Sphere;
        sph.isStatic = true;
        sph.radius = 0.2f;
        sph.position = {0.0f, 0.0f, 0.0f};
        const std::uint32_t sphId = world.CreateBody(sph);

        Body hc;
        hc.shape = ShapeType::HalfCylinder;
        hc.radius = 0.22f;
        hc.halfHeight = 0.15f;
        hc.mass = 1.0f;
        hc.restitution = 0.0f;
        hc.position = {0.28f, 0.0f, 0.0f};
        const std::uint32_t hcId = world.CreateBody(hc);

        world.Step(1.0f / 120.0f, 12);
        assert(HasManifoldBetween(world, sphId, hcId));
        constexpr std::uint8_t kSphereHalfCylinderManifold = 18u;
        const std::uint32_t lo = std::min(sphId, hcId);
        const std::uint32_t hi = std::max(sphId, hcId);
        bool foundShc = false;
        for (const Manifold& m : world.DebugManifolds()) {
            if (std::min(m.a, m.b) == lo && std::max(m.a, m.b) == hi && m.manifoldType == kSphereHalfCylinderManifold) {
                foundShc = true;
                assert(m.contacts.size() >= 1u);
                break;
            }
        }
        assert(foundShc);
    }

    {
        World world({0.0f, 0.0f, 0.0f});

        Body cyl;
        cyl.shape = ShapeType::Cylinder;
        cyl.isStatic = true;
        cyl.radius = 0.25f;
        cyl.halfHeight = 0.35f;
        cyl.position = {0.0f, 0.0f, 0.0f};
        const std::uint32_t cylId = world.CreateBody(cyl);

        Body sph;
        sph.shape = ShapeType::Sphere;
        sph.radius = 0.12f;
        sph.mass = 1.0f;
        sph.restitution = 0.0f;
        sph.position = {0.32f, 0.0f, 0.0f};
        const std::uint32_t sphId = world.CreateBody(sph);

        world.Step(1.0f / 120.0f, 12);
        assert(HasManifoldBetween(world, cylId, sphId));
        constexpr std::uint8_t kSphereCylinderManifold = 14u;
        const std::uint32_t lo = std::min(cylId, sphId);
        const std::uint32_t hi = std::max(cylId, sphId);
        bool found = false;
        for (const Manifold& m : world.DebugManifolds()) {
            if (std::min(m.a, m.b) == lo && std::max(m.a, m.b) == hi && m.manifoldType == kSphereCylinderManifold) {
                found = true;
                assert(m.contacts.size() >= 1u);
                break;
            }
        }
        assert(found);
    }

    {
        World world({0.0f, 0.0f, 0.0f});

        Body cyl;
        cyl.shape = ShapeType::Cylinder;
        cyl.isStatic = true;
        cyl.radius = 0.18f;
        cyl.halfHeight = 0.25f;
        cyl.position = {0.0f, 0.0f, 0.0f};
        const std::uint32_t cylId = world.CreateBody(cyl);

        Body hc;
        hc.shape = ShapeType::HalfCylinder;
        hc.radius = 0.16f;
        hc.halfHeight = 0.2f;
        hc.mass = 1.0f;
        hc.restitution = 0.0f;
        hc.position = {0.26f, 0.0f, 0.08f};
        const std::uint32_t hcId = world.CreateBody(hc);

        world.Step(1.0f / 120.0f, 12);
        assert(HasManifoldBetween(world, cylId, hcId));
    }

    {
        World world({0.0f, 0.0f, 0.0f});

        Body cyl;
        cyl.shape = ShapeType::Cylinder;
        cyl.isStatic = true;
        cyl.radius = 0.4f;
        cyl.halfHeight = 0.38f;
        cyl.position = {0.0f, 0.0f, 0.0f};
        const std::uint32_t cylId = world.CreateBody(cyl);

        Body hc;
        hc.shape = ShapeType::HalfCylinder;
        hc.radius = 0.35f;
        hc.halfHeight = 0.34f;
        hc.mass = 1.0f;
        hc.restitution = 0.0f;
        hc.position = {0.42f, 0.0f, 0.03f};
        const std::uint32_t hcId = world.CreateBody(hc);

        world.Step(1.0f / 120.0f, 12);
        assert(HasManifoldBetween(world, cylId, hcId));
        constexpr std::uint8_t kHalfCylinderCylinderManifold = 20u;
        const std::uint32_t lo = std::min(cylId, hcId);
        const std::uint32_t hi = std::max(cylId, hcId);
        bool foundPatch = false;
        for (const Manifold& m : world.DebugManifolds()) {
            if (std::min(m.a, m.b) == lo && std::max(m.a, m.b) == hi && m.manifoldType == kHalfCylinderCylinderManifold) {
                foundPatch = true;
                assert(m.contacts.size() >= 2u);
                break;
            }
        }
        assert(foundPatch);
    }

    {
        World world({0.0f, 0.0f, 0.0f});

        Body a;
        a.shape = ShapeType::HalfCylinder;
        a.isStatic = true;
        a.radius = 0.14f;
        a.halfHeight = 0.2f;
        a.position = {0.0f, 0.0f, 0.0f};
        const std::uint32_t aId = world.CreateBody(a);

        Body b;
        b.shape = ShapeType::HalfCylinder;
        b.radius = 0.13f;
        b.halfHeight = 0.18f;
        b.mass = 1.0f;
        b.restitution = 0.0f;
        b.orientation = AxisAngle({0.0f, 1.0f, 0.0f}, 0.35f);
        b.position = {0.2f, 0.02f, 0.06f};
        const std::uint32_t bId = world.CreateBody(b);

        world.Step(1.0f / 120.0f, 12);
        assert(HasManifoldBetween(world, aId, bId));
    }

    {
        World world({0.0f, 0.0f, 0.0f});

        Body a;
        a.shape = ShapeType::HalfCylinder;
        a.isStatic = true;
        a.radius = 0.48f;
        a.halfHeight = 0.42f;
        a.position = {0.0f, 0.0f, 0.0f};
        const std::uint32_t aId = world.CreateBody(a);

        Body b;
        b.shape = ShapeType::HalfCylinder;
        b.radius = 0.46f;
        b.halfHeight = 0.4f;
        b.mass = 1.0f;
        b.restitution = 0.0f;
        b.position = {0.52f, 0.0f, 0.04f};
        const std::uint32_t bId = world.CreateBody(b);

        world.Step(1.0f / 120.0f, 12);
        assert(HasManifoldBetween(world, aId, bId));
        constexpr std::uint8_t kHalfCylinderHalfCylinderManifold = 21u;
        const std::uint32_t lo = std::min(aId, bId);
        const std::uint32_t hi = std::max(aId, bId);
        bool foundPatch = false;
        for (const Manifold& m : world.DebugManifolds()) {
            if (std::min(m.a, m.b) == lo && std::max(m.a, m.b) == hi && m.manifoldType == kHalfCylinderHalfCylinderManifold) {
                foundPatch = true;
                assert(m.contacts.size() >= 2u);
                break;
            }
        }
        assert(foundPatch);
    }

    {
        World world({0.0f, 0.0f, 0.0f});

        Body hc;
        hc.shape = ShapeType::HalfCylinder;
        hc.isStatic = true;
        hc.radius = 0.2f;
        hc.halfHeight = 0.25f;
        hc.position = {0.0f, 0.0f, 0.0f};
        const std::uint32_t hcId = world.CreateBody(hc);

        Body cap;
        cap.shape = ShapeType::Capsule;
        cap.radius = 0.1f;
        cap.halfHeight = 0.2f;
        cap.mass = 1.0f;
        cap.restitution = 0.0f;
        cap.position = {0.22f, 0.0f, 0.06f};
        const std::uint32_t capId = world.CreateBody(cap);

        for (int i = 0; i < 4; ++i) {
            world.Step(1.0f / 120.0f, 12);
        }
        assert(HasManifoldBetween(world, hcId, capId));
    }

    return 0;
}
