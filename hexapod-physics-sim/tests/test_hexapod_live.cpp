#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>

#include "demo/frame_sink.cpp"
#include "demo/scenes.cpp"

namespace {

using namespace minphys3d;
using namespace minphys3d::demo;

float WrapAngle(float angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

struct Metrics {
    float peakLinear = 0.0f;
    float peakAngular = 0.0f;
    float peakError = 0.0f;
    float finalSpeed = 0.0f;
    Vec3 finalPosition{};
};

Metrics RunLiveHexapod() {
    World world({0.0f, -9.81f, 0.0f});
    JointSolverConfig joint_cfg = world.GetJointSolverConfig();
    joint_cfg.servoPositionPasses = 0;
    joint_cfg.hingeAnchorBiasFactor = 0.25f;
    joint_cfg.hingeAnchorDampingFactor = 0.3f;
    world.SetJointSolverConfig(joint_cfg);

    const HexapodSceneObjects scene = BuildHexapodScene(world);
    RelaxBuiltInHexapodServos(world, scene);

    Metrics metrics;
    constexpr float kDt = 1.0f / 240.0f;
    for (int step = 0; step < 240; ++step) {
        world.Step(kDt, 24);
        const Body& chassis = world.GetBody(scene.body);
        metrics.peakLinear = std::max(metrics.peakLinear, Length(chassis.velocity));
        metrics.peakAngular = std::max(metrics.peakAngular, Length(chassis.angularVelocity));
        for (const LegLinkIds& leg : scene.legs) {
            metrics.peakError = std::max(
                metrics.peakError,
                std::abs(WrapAngle(world.GetServoJointAngle(leg.bodyToCoxaJoint) - world.GetServoJoint(leg.bodyToCoxaJoint).targetAngle)));
            metrics.peakError = std::max(
                metrics.peakError,
                std::abs(WrapAngle(world.GetServoJointAngle(leg.coxaToFemurJoint) - world.GetServoJoint(leg.coxaToFemurJoint).targetAngle)));
            metrics.peakError = std::max(
                metrics.peakError,
                std::abs(WrapAngle(world.GetServoJointAngle(leg.femurToTibiaJoint) - world.GetServoJoint(leg.femurToTibiaJoint).targetAngle)));
        }
    }

    const Body& chassis = world.GetBody(scene.body);
    metrics.finalSpeed = Length(chassis.velocity);
    metrics.finalPosition = chassis.position;
    return metrics;
}

} // namespace

int main() {
    const Metrics metrics = RunLiveHexapod();

    constexpr float kMaxLinear = 1.5f;
    constexpr float kMaxAngular = 0.1f;
    constexpr float kMaxError = 2.0f;
    constexpr float kMaxFinalSpeed = 0.05f;
    constexpr float kMinFinalY = 0.030f;

    if (metrics.peakLinear > kMaxLinear) {
        std::cerr << "hex_live peak_linear=" << metrics.peakLinear << " cap=" << kMaxLinear << "\n";
        return 1;
    }
    if (metrics.peakAngular > kMaxAngular) {
        std::cerr << "hex_live peak_angular=" << metrics.peakAngular << " cap=" << kMaxAngular << "\n";
        return 1;
    }
    if (metrics.peakError > kMaxError) {
        std::cerr << "hex_live peak_error=" << metrics.peakError << " cap=" << kMaxError << "\n";
        return 1;
    }
    if (metrics.finalSpeed > kMaxFinalSpeed) {
        std::cerr << "hex_live final_speed=" << metrics.finalSpeed << " cap=" << kMaxFinalSpeed << "\n";
        return 1;
    }
    if (metrics.finalPosition.y < kMinFinalY) {
        std::cerr << "hex_live final_y=" << metrics.finalPosition.y << " floor=" << kMinFinalY << "\n";
        return 1;
    }

    return 0;
}
