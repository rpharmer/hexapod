#include <cassert>

#include "minphys3d/core/world.hpp"

namespace {

using namespace minphys3d;

Body MakeDynamicSphere(const Vec3& position) {
    Body body{};
    body.shape = ShapeType::Sphere;
    body.radius = 0.15;
    body.mass = 1.0;
    body.position = position;
    body.orientation = Quat{1.0, 0.0, 0.0, 0.0};
    body.restitution = 0.0;
    body.staticFriction = 0.8;
    body.dynamicFriction = 0.6;
    return body;
}

} // namespace

int main() {
    World world({0.0, 0.0, 0.0});

    const std::uint32_t bodyA = world.CreateBody(MakeDynamicSphere({-0.25, 0.5, 0.0}));
    const std::uint32_t bodyB = world.CreateBody(MakeDynamicSphere({0.25, 0.5, 0.0}));
    const std::uint32_t jointId = world.CreateDistanceJoint(bodyA, bodyB, {-0.25, 0.5, 0.0}, {0.25, 0.5, 0.0});
    assert(jointId == 0u);

    world.Step(1.0 / 60.0);

    const World::TopologySnapshot topology = world.SnapshotTopology();
    assert(topology.bodyCount == 2u);
    assert(topology.dynamicBodyCount == 2u);
    assert(topology.awakeBodyCount == 2u);
    assert(topology.contactCount == 0u);
    assert(topology.manifoldCount == 0u);
    assert(topology.islandCount == 1u);
    assert(topology.maxIslandBodyCount == 2u);
    assert(topology.maxIslandManifoldCount == 0u);
    assert(topology.maxIslandJointCount == 1u);
    assert(topology.maxIslandServoCount == 0u);

    return 0;
}
