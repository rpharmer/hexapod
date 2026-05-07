#include <algorithm>
#include <cmath>
#include <iostream>

#include "demo/frame_sink.cpp"
#include "demo/scenes.cpp"

namespace {

using namespace minphys3d;
using namespace minphys3d::demo;

struct Metrics {
    float peakChassisAngularSpeed = 0.0f;
};

Metrics RunFreefallNoSupport() {
    World world({0.0f, 0.0f, 0.0f});
    JointSolverConfig joint_cfg = world.GetJointSolverConfig();
    joint_cfg.servoPositionPasses = 0;
    joint_cfg.hingeAnchorBiasFactor = 0.25f;
    joint_cfg.hingeAnchorDampingFactor = 0.3f;
    world.SetJointSolverConfig(joint_cfg);

    const HexapodSceneObjects scene = BuildHexapodScene(world);
    RelaxBuiltInHexapodServos(world, scene);

    // Sink the plane so no support contacts form and the free-floating
    // articulation path stays active for the full production iteration count.
    world.GetBody(scene.plane).planeOffset = -1000.0f;

    Metrics metrics;
    constexpr float kDt = 1.0f / 240.0f;
    constexpr int kSolverIterations = 40;
    for (int step = 0; step < 240; ++step) {
        world.Step(kDt, kSolverIterations);
        const Body& chassis = world.GetBody(scene.body);
        metrics.peakChassisAngularSpeed =
            std::max(metrics.peakChassisAngularSpeed, Length(chassis.angularVelocity));
    }
    return metrics;
}

} // namespace

int main() {
    const Metrics metrics = RunFreefallNoSupport();

    constexpr float kMaxAngularSpeed = 5.0f;

    if (metrics.peakChassisAngularSpeed > kMaxAngularSpeed) {
        std::cerr << "freefall_no_support peak_angular=" << metrics.peakChassisAngularSpeed
                  << " cap=" << kMaxAngularSpeed << "\n";
        return 1;
    }
    return 0;
}
