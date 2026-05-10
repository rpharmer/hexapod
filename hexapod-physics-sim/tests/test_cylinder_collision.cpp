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

Quat AxisAngle(const Vec3& axis, Real radians) {
    Vec3 n = axis;
    if (LengthSquared(n) <= kEpsilon * kEpsilon) {
        return {1.0, 0.0, 0.0, 0.0};
    }
    n = Normalize(n);
    const Real half = 0.5 * radians;
    const Real s = std::sin(half);
    return Normalize(Quat{std::cos(half), n.x * s, n.y * s, n.z * s});
}

} // namespace

int main() {
    {
        World world({0.0, -9.81, 0.0});

        Body plane;
        plane.shape = ShapeType::Plane;
        world.CreateBody(plane);

        Body cyl;
        cyl.shape = ShapeType::Cylinder;
        cyl.radius = 0.2;
        cyl.halfHeight = 0.2;
        cyl.mass = 1.0;
        cyl.restitution = 0.0;
        cyl.position = {0.0, 0.55, 0.0};
        const std::uint32_t cylId = world.CreateBody(cyl);

        for (int i = 0; i < 200; ++i) {
            world.Step(1.0 / 120.0, 10);
        }
        const Body& settled = world.GetBody(cylId);
        assert(std::isfinite(settled.position.y));
        assert(settled.position.y > cyl.radius * 0.5);
    }

    {
        World world({0.0, 0.0, 0.0});

        Body box;
        box.shape = ShapeType::Box;
        box.isStatic = true;
        box.halfExtents = {0.5, 0.15, 0.5};
        box.position = {0.0, 0.0, 0.0};
        const std::uint32_t boxId = world.CreateBody(box);

        Body cyl;
        cyl.shape = ShapeType::Cylinder;
        cyl.radius = 0.18;
        cyl.halfHeight = 0.22;
        cyl.mass = 1.0;
        cyl.restitution = 0.0;
        cyl.position = {0.0, 0.28, 0.0};
        const std::uint32_t cylId = world.CreateBody(cyl);

        world.Step(1.0 / 120.0, 12);
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
        World world({0.0, 0.0, 0.0});

        Body cap;
        cap.shape = ShapeType::Capsule;
        cap.isStatic = true;
        cap.radius = 0.12;
        cap.halfHeight = 0.35;
        cap.position = {0.0, 0.0, 0.0};
        const std::uint32_t capId = world.CreateBody(cap);

        Body cyl;
        cyl.shape = ShapeType::Cylinder;
        cyl.radius = 0.16;
        cyl.halfHeight = 0.2;
        cyl.mass = 1.0;
        cyl.restitution = 0.0;
        cyl.position = {0.18, 0.0, 0.0};
        const std::uint32_t cylId = world.CreateBody(cyl);

        for (int i = 0; i < 4; ++i) {
            world.Step(1.0 / 120.0, 12);
        }
        assert(HasManifoldBetween(world, capId, cylId));
    }

    {
        World world({0.0, 0.0, 0.0});

        Body a;
        a.shape = ShapeType::Cylinder;
        a.isStatic = true;
        a.radius = 0.15;
        a.halfHeight = 0.25;
        a.position = {0.0, 0.0, 0.0};
        const std::uint32_t aId = world.CreateBody(a);

        Body b;
        b.shape = ShapeType::Cylinder;
        b.radius = 0.14;
        b.halfHeight = 0.22;
        b.mass = 1.0;
        b.restitution = 0.0;
        b.position = {0.22, 0.05, 0.0};
        const std::uint32_t bId = world.CreateBody(b);

        world.Step(1.0 / 120.0, 12);
        assert(HasManifoldBetween(world, aId, bId));
    }

    {
        World world({0.0, 0.0, 0.0});

        Body box;
        box.shape = ShapeType::Box;
        box.isStatic = true;
        box.halfExtents = {0.4, 0.12, 0.4};
        box.position = {0.0, 0.0, 0.0};
        const std::uint32_t boxId = world.CreateBody(box);

        Body hc;
        hc.shape = ShapeType::HalfCylinder;
        hc.radius = 0.2;
        hc.halfHeight = 0.18;
        hc.mass = 1.0;
        hc.restitution = 0.0;
        hc.orientation = AxisAngle({1.0, 0.0, 0.0}, 3.14159265 * 0.5);
        hc.position = {0.0, 0.22, 0.15};
        const std::uint32_t hcId = world.CreateBody(hc);

        world.Step(1.0 / 120.0, 12);
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
        World world({0.0, 0.0, 0.0});

        Body sph;
        sph.shape = ShapeType::Sphere;
        sph.isStatic = true;
        sph.radius = 0.2;
        sph.position = {0.0, 0.0, 0.0};
        const std::uint32_t sphId = world.CreateBody(sph);

        Body hc;
        hc.shape = ShapeType::HalfCylinder;
        hc.radius = 0.22;
        hc.halfHeight = 0.15;
        hc.mass = 1.0;
        hc.restitution = 0.0;
        hc.position = {0.28, 0.0, 0.0};
        const std::uint32_t hcId = world.CreateBody(hc);

        world.Step(1.0 / 120.0, 12);
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
        World world({0.0, 0.0, 0.0});

        Body cyl;
        cyl.shape = ShapeType::Cylinder;
        cyl.isStatic = true;
        cyl.radius = 0.25;
        cyl.halfHeight = 0.35;
        cyl.position = {0.0, 0.0, 0.0};
        const std::uint32_t cylId = world.CreateBody(cyl);

        Body sph;
        sph.shape = ShapeType::Sphere;
        sph.radius = 0.12;
        sph.mass = 1.0;
        sph.restitution = 0.0;
        sph.position = {0.32, 0.0, 0.0};
        const std::uint32_t sphId = world.CreateBody(sph);

        world.Step(1.0 / 120.0, 12);
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
        World world({0.0, 0.0, 0.0});

        Body cyl;
        cyl.shape = ShapeType::Cylinder;
        cyl.isStatic = true;
        cyl.radius = 0.18;
        cyl.halfHeight = 0.25;
        cyl.position = {0.0, 0.0, 0.0};
        const std::uint32_t cylId = world.CreateBody(cyl);

        Body hc;
        hc.shape = ShapeType::HalfCylinder;
        hc.radius = 0.16;
        hc.halfHeight = 0.2;
        hc.mass = 1.0;
        hc.restitution = 0.0;
        hc.position = {0.26, 0.0, 0.08};
        const std::uint32_t hcId = world.CreateBody(hc);

        world.Step(1.0 / 120.0, 12);
        assert(HasManifoldBetween(world, cylId, hcId));
    }

    {
        World world({0.0, 0.0, 0.0});

        Body cyl;
        cyl.shape = ShapeType::Cylinder;
        cyl.isStatic = true;
        cyl.radius = 0.4;
        cyl.halfHeight = 0.38;
        cyl.position = {0.0, 0.0, 0.0};
        const std::uint32_t cylId = world.CreateBody(cyl);

        Body hc;
        hc.shape = ShapeType::HalfCylinder;
        hc.radius = 0.35;
        hc.halfHeight = 0.34;
        hc.mass = 1.0;
        hc.restitution = 0.0;
        hc.position = {0.42, 0.0, 0.03};
        const std::uint32_t hcId = world.CreateBody(hc);

        world.Step(1.0 / 120.0, 12);
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
        World world({0.0, 0.0, 0.0});

        Body a;
        a.shape = ShapeType::HalfCylinder;
        a.isStatic = true;
        a.radius = 0.14;
        a.halfHeight = 0.2;
        a.position = {0.0, 0.0, 0.0};
        const std::uint32_t aId = world.CreateBody(a);

        Body b;
        b.shape = ShapeType::HalfCylinder;
        b.radius = 0.13;
        b.halfHeight = 0.18;
        b.mass = 1.0;
        b.restitution = 0.0;
        b.orientation = AxisAngle({0.0, 1.0, 0.0}, 0.35);
        b.position = {0.2, 0.02, 0.06};
        const std::uint32_t bId = world.CreateBody(b);

        world.Step(1.0 / 120.0, 12);
        assert(HasManifoldBetween(world, aId, bId));
    }

    {
        World world({0.0, 0.0, 0.0});

        Body a;
        a.shape = ShapeType::HalfCylinder;
        a.isStatic = true;
        a.radius = 0.48;
        a.halfHeight = 0.42;
        a.position = {0.0, 0.0, 0.0};
        const std::uint32_t aId = world.CreateBody(a);

        Body b;
        b.shape = ShapeType::HalfCylinder;
        b.radius = 0.46;
        b.halfHeight = 0.4;
        b.mass = 1.0;
        b.restitution = 0.0;
        b.position = {0.52, 0.0, 0.04};
        const std::uint32_t bId = world.CreateBody(b);

        world.Step(1.0 / 120.0, 12);
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
        World world({0.0, 0.0, 0.0});

        Body hc;
        hc.shape = ShapeType::HalfCylinder;
        hc.isStatic = true;
        hc.radius = 0.2;
        hc.halfHeight = 0.25;
        hc.position = {0.0, 0.0, 0.0};
        const std::uint32_t hcId = world.CreateBody(hc);

        Body cap;
        cap.shape = ShapeType::Capsule;
        cap.radius = 0.1;
        cap.halfHeight = 0.2;
        cap.mass = 1.0;
        cap.restitution = 0.0;
        cap.position = {0.22, 0.0, 0.06};
        const std::uint32_t capId = world.CreateBody(cap);

        for (int i = 0; i < 4; ++i) {
            world.Step(1.0 / 120.0, 12);
        }
        assert(HasManifoldBetween(world, hcId, capId));
    }

    return 0;
}
