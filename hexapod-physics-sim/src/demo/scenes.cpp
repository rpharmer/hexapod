#include "demo/scenes.hpp"

#include <chrono>
#include <cstdio>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <memory>
#include <system_error>
#include <thread>
#include <utility>

#include "demo/frame_sink.hpp"
#include "minphys3d/core/body.hpp"
#include "minphys3d/core/world.hpp"

namespace minphys3d::demo {
namespace {

const char* SinkName(SinkKind sink_kind) {
    switch (sink_kind) {
        case SinkKind::Dummy:
            return "dummy";
        case SinkKind::Udp:
            return "udp";
    }
    return "unknown";
}

std::filesystem::path BuildDebugLogPath() {
    std::filesystem::path log_dir = "logs";
    const std::filesystem::path cwd_name = std::filesystem::current_path().filename();
    if (cwd_name != "hexapod-physics-sim") {
        log_dir = std::filesystem::path("hexapod-physics-sim") / log_dir;
    }
    std::error_code ec;
    std::filesystem::create_directories(log_dir, ec);
    if (ec) {
        return "hexapod-physics-sim.log";
    }
    return log_dir / "latest.log";
}

class RunLog {
public:
    explicit RunLog(std::filesystem::path path) : path_(std::move(path)) {
        file_ = std::fopen(path_.string().c_str(), "w");
    }

    ~RunLog() {
        if (file_ != nullptr) {
            std::fclose(file_);
        }
    }

    RunLog(const RunLog&) = delete;
    RunLog& operator=(const RunLog&) = delete;

    std::FILE* stream() const { return file_; }
    bool available() const { return file_ != nullptr; }
    const std::filesystem::path& path() const { return path_; }

private:
    std::filesystem::path path_;
    std::FILE* file_ = nullptr;
};

} // namespace

int RunDefaultScene(SinkKind sink_kind) {
    RunLog run_log(BuildDebugLogPath());
    World world({0.0f, -9.81f, 0.0f});
#ifndef NDEBUG
    world.SetDebugLogStream(run_log.stream());
    world.SetBlockSolveDebugLogging(true);
#endif

    std::cout << "[hexapod-physics-sim] starting demo scene"
              << " sink=" << SinkName(sink_kind);
    if (run_log.available()) {
        std::cout << " log=" << run_log.path().string();
    } else {
        std::cout << " log=stderr (failed to open log file)";
    }
    std::cout << "\n";

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
        if (run_log.available()) {
            std::fprintf(run_log.stream(),
                         "frame=%d A=(%.6f, %.6f, %.6f) B=(%.6f, %.6f, %.6f) S=(%.6f, %.6f, %.6f)\n",
                         frame,
                         a.position.x,
                         a.position.y,
                         a.position.z,
                         b.position.x,
                         b.position.y,
                         b.position.z,
                         s.position.x,
                         s.position.y,
                         s.position.z);
        }
    }

    std::cout << "[hexapod-physics-sim] completed demo scene frames=120 sink=" << SinkName(sink_kind);
    if (run_log.available()) {
        std::cout << " log=" << run_log.path().string();
    }
    std::cout << "\n";

    return 0;
}

int RunRealTimeDefaultScene(SinkKind sink_kind) {
    RunLog run_log(BuildDebugLogPath());
    World world({0.0f, -9.81f, 0.0f});
#ifndef NDEBUG
    world.SetDebugLogStream(run_log.stream());
    world.SetBlockSolveDebugLogging(true);
#endif

    std::cout << "[hexapod-physics-sim] starting demo scene"
              << " sink=" << SinkName(sink_kind);
    if (run_log.available()) {
        std::cout << " log=" << run_log.path().string();
    } else {
        std::cout << " log=stderr (failed to open log file)";
    }
    std::cout << "\n";

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
    for (int frame = 0; frame < 1200; ++frame) {
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
        if (run_log.available()) {
            std::fprintf(run_log.stream(),
                         "frame=%d A=(%.6f, %.6f, %.6f) B=(%.6f, %.6f, %.6f) S=(%.6f, %.6f, %.6f)\n",
                         frame,
                         a.position.x,
                         a.position.y,
                         a.position.z,
                         b.position.x,
                         b.position.y,
                         b.position.z,
                         s.position.x,
                         s.position.y,
                         s.position.z);
        }
        
        std::this_thread::sleep_for(std::chrono::duration<float>(dt));
    }

    std::cout << "[hexapod-physics-sim] completed demo scene frames=120 sink=" << SinkName(sink_kind);
    if (run_log.available()) {
        std::cout << " log=" << run_log.path().string();
    }
    std::cout << "\n";

    return 0;
}

} // namespace minphys3d::demo
