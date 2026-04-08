#include "demo/scenes.hpp"

#include <cstdint>
#include <iostream>
#include <memory>

#include "demo/frame_sink.hpp"
#include "minphys3d/core/body.hpp"
#include "minphys3d/core/world.hpp"

namespace minphys3d::demo {

int RunDefaultScene(SinkKind sink_kind) {
    World world({0.0f, -9.81f, 0.0f});
#ifndef NDEBUG
    world.SetBlockSolveDebugLogging(true);
#endif

    Body plane;
    plane.shape = ShapeType::Plane;
    plane.planeNormal = {0.0f, 1.0f, 0.0f};
    plane.planeOffset = 0.0f;
    plane.restitution = 0.05f;
    plane.staticFriction = 0.9f;
    plane.dynamicFriction = 0.65f;
    const std::uint32_t planeId = world.CreateBody(plane);

    Body boxA;
    boxA.shape = ShapeType::Box;
    boxA.position = {0.0f, 0.9f, 0.0f};
    boxA.halfExtents = {0.6f, 0.25f, 0.5f};
    boxA.mass = 3.0f;
    boxA.restitution = 0.05f;
    boxA.staticFriction = 0.8f;
    boxA.dynamicFriction = 0.55f;
    const std::uint32_t boxAId = world.CreateBody(boxA);

    Body boxB;
    boxB.shape = ShapeType::Box;
    boxB.position = {0.1f, 2.1f, 0.0f};
    boxB.velocity = {0.0f, -0.2f, 0.0f};
    boxB.halfExtents = {0.5f, 0.25f, 0.4f};
    boxB.mass = 2.0f;
    boxB.restitution = 0.03f;
    boxB.staticFriction = 0.78f;
    boxB.dynamicFriction = 0.5f;
    boxB.orientation = Normalize(Quat{0.9914f, 0.0f, 0.0f, 0.1305f});
    const std::uint32_t boxBId = world.CreateBody(boxB);

    Body sphere;
    sphere.shape = ShapeType::Sphere;
    sphere.position = {1.25f, 2.8f, 0.0f};
    sphere.velocity = {-3.2f, -0.2f, 0.1f};
    sphere.radius = 0.35f;
    sphere.mass = 1.0f;
    sphere.restitution = 0.25f;
    const std::uint32_t sphereId = world.CreateBody(sphere);

    world.CreateDistanceJoint(
        boxBId,
        sphereId,
        world.GetBody(boxBId).position + Vec3{0.0f, 0.25f, 0.0f},
        world.GetBody(sphereId).position,
        0.2f,
        0.15f);

    world.CreateHingeJoint(
        boxAId,
        boxBId,
        world.GetBody(boxAId).position + Vec3{0.0f, 0.25f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        true,
        -0.5f,
        0.5f,
        true,
        0.5f,
        0.2f);

    std::unique_ptr<FrameSink> sink =
        sink_kind == SinkKind::Udp ? MakeUdpSink() : MakeDummySink();

    constexpr float dt = 1.0f / 60.0f;
    for (int frame = 0; frame < 120; ++frame) {
        world.Step(dt, 16);

        sink->begin_frame(frame, static_cast<float>(frame + 1) * dt);
        sink->emit_body(planeId, world.GetBody(planeId));
        sink->emit_body(boxAId, world.GetBody(boxAId));
        sink->emit_body(boxBId, world.GetBody(boxBId));
        sink->emit_body(sphereId, world.GetBody(sphereId));
        sink->end_frame();

        const Body& a = world.GetBody(boxAId);
        const Body& b = world.GetBody(boxBId);
        const Body& s = world.GetBody(sphereId);
        std::cout << "frame=" << frame
                  << " A=(" << a.position.x << ", " << a.position.y << ", " << a.position.z << ")"
                  << " B=(" << b.position.x << ", " << b.position.y << ", " << b.position.z << ")"
                  << " S=(" << s.position.x << ", " << s.position.y << ", " << s.position.z << ")\n";
    }

    return 0;
}

} // namespace minphys3d::demo
