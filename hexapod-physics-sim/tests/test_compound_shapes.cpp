#include <cassert>
#include <cmath>

#include "minphys3d/core/world.hpp"

using namespace minphys3d;

namespace {

CompoundChild MakeSphereChild(Real radius, const Vec3& localPosition) {
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
        World world({0.0, -9.81, 0.0});

        Body plane;
        plane.shape = ShapeType::Plane;
        world.CreateBody(plane);

        Body compound;
        compound.shape = ShapeType::Compound;
        compound.mass = 2.0;
        compound.position = {0.0, 1.2, 0.0};
        compound.compoundChildren.push_back(MakeBoxChild({0.25, 0.10, 0.25}, {0.0, 0.0, 0.0}));
        compound.compoundChildren.push_back(MakeSphereChild(0.12, {0.0, -0.22, 0.0}));
        const std::uint32_t compoundId = world.CreateBody(compound);

        for (int i = 0; i < 240; ++i) {
            world.Step(1.0 / 120.0, 12);
        }

        const Body& settled = world.GetBody(compoundId);
        assert(std::isfinite(settled.position.y));
        assert(settled.position.y > 0.15);
    }

    {
        World world({0.0, 0.0, 0.0});

        Body staticCompound;
        staticCompound.shape = ShapeType::Compound;
        staticCompound.isStatic = true;
        staticCompound.compoundChildren.push_back(MakeSphereChild(0.25, {-1.0, 0.0, 0.0}));
        staticCompound.compoundChildren.push_back(MakeSphereChild(0.25, {1.0, 0.0, 0.0}));
        world.CreateBody(staticCompound);

        Body sphere;
        sphere.shape = ShapeType::Sphere;
        sphere.radius = 0.2;
        sphere.mass = 1.0;
        sphere.restitution = 0.0;
        sphere.position = {1.8, 0.0, 0.0};
        sphere.velocity = {-2.0, 0.0, 0.0};
        const std::uint32_t sphereId = world.CreateBody(sphere);

        for (int i = 0; i < 180; ++i) {
            world.Step(1.0 / 120.0, 10);
        }

        const Body& out = world.GetBody(sphereId);
        assert(out.position.x > -0.9);
    }

    {
        World world({0.0, 0.0, 0.0});

        auto makeCompoundBody = [](const Vec3& position, bool isStatic) {
            Body body;
            body.shape = ShapeType::Compound;
            body.mass = 1.0;
            body.isStatic = isStatic;
            body.position = position;
            body.compoundChildren.push_back(MakeBoxChild({0.20, 0.20, 0.20}, {-0.25, 0.0, 0.0}));
            body.compoundChildren.push_back(MakeBoxChild({0.20, 0.20, 0.20}, {0.25, 0.0, 0.0}));
            return body;
        };

        world.CreateBody(makeCompoundBody({0.0, 0.0, 0.0}, true));

        Body moving = makeCompoundBody({1.4, 0.0, 0.0}, false);
        moving.velocity = {-1.0, 0.0, 0.0};
        const std::uint32_t movingId = world.CreateBody(moving);

        for (int i = 0; i < 240; ++i) {
            world.Step(1.0 / 120.0, 12);
        }

        const Body& out = world.GetBody(movingId);
        assert(out.position.x > 0.0);
    }

    return 0;
}
