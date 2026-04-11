#include <cassert>
#include <cmath>

#include "minphys3d/core/world.hpp"

using namespace minphys3d;

namespace {

CompoundChild MakeSphereChild(float radius, const Vec3& localPosition) {
    CompoundChild child;
    child.shape = ShapeType::Sphere;
    child.radius = radius;
    child.localPosition = localPosition;
    return child;
}

CompoundChild MakeBoxChild(const Vec3& halfExtents, const Vec3& localPosition) {
    CompoundChild child;
    child.shape = ShapeType::Box;
    child.halfExtents = halfExtents;
    child.localPosition = localPosition;
    return child;
}

} // namespace

int main() {
    {
        World world({0.0f, -9.81f, 0.0f});

        Body plane;
        plane.shape = ShapeType::Plane;
        world.CreateBody(plane);

        Body compound;
        compound.shape = ShapeType::Compound;
        compound.mass = 2.0f;
        compound.position = {0.0f, 1.2f, 0.0f};
        compound.compoundChildren.push_back(MakeBoxChild({0.25f, 0.10f, 0.25f}, {0.0f, 0.0f, 0.0f}));
        compound.compoundChildren.push_back(MakeSphereChild(0.12f, {0.0f, -0.22f, 0.0f}));
        const std::uint32_t compoundId = world.CreateBody(compound);

        for (int i = 0; i < 240; ++i) {
            world.Step(1.0f / 120.0f, 12);
        }

        const Body& settled = world.GetBody(compoundId);
        assert(std::isfinite(settled.position.y));
        assert(settled.position.y > 0.15f);
    }

    {
        World world({0.0f, 0.0f, 0.0f});

        Body staticCompound;
        staticCompound.shape = ShapeType::Compound;
        staticCompound.isStatic = true;
        staticCompound.compoundChildren.push_back(MakeSphereChild(0.25f, {-1.0f, 0.0f, 0.0f}));
        staticCompound.compoundChildren.push_back(MakeSphereChild(0.25f, {1.0f, 0.0f, 0.0f}));
        world.CreateBody(staticCompound);

        Body sphere;
        sphere.shape = ShapeType::Sphere;
        sphere.radius = 0.2f;
        sphere.mass = 1.0f;
        sphere.restitution = 0.0f;
        sphere.position = {1.8f, 0.0f, 0.0f};
        sphere.velocity = {-2.0f, 0.0f, 0.0f};
        const std::uint32_t sphereId = world.CreateBody(sphere);

        for (int i = 0; i < 180; ++i) {
            world.Step(1.0f / 120.0f, 10);
        }

        const Body& out = world.GetBody(sphereId);
        assert(out.position.x > -0.9f);
    }

    {
        World world({0.0f, 0.0f, 0.0f});

        auto makeCompoundBody = [](const Vec3& position, bool isStatic) {
            Body body;
            body.shape = ShapeType::Compound;
            body.mass = 1.0f;
            body.isStatic = isStatic;
            body.position = position;
            body.compoundChildren.push_back(MakeBoxChild({0.20f, 0.20f, 0.20f}, {-0.25f, 0.0f, 0.0f}));
            body.compoundChildren.push_back(MakeBoxChild({0.20f, 0.20f, 0.20f}, {0.25f, 0.0f, 0.0f}));
            return body;
        };

        world.CreateBody(makeCompoundBody({0.0f, 0.0f, 0.0f}, true));

        Body moving = makeCompoundBody({1.4f, 0.0f, 0.0f}, false);
        moving.velocity = {-1.0f, 0.0f, 0.0f};
        const std::uint32_t movingId = world.CreateBody(moving);

        for (int i = 0; i < 240; ++i) {
            world.Step(1.0f / 120.0f, 12);
        }

        const Body& out = world.GetBody(movingId);
        assert(out.position.x > 0.0f);
    }

    return 0;
}
