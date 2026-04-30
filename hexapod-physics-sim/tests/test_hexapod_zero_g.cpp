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
    Vec3 finalPosition{};
};

Metrics RunZeroGravityHexapod() {
    World world({0.0f, 0.0f, 0.0f});
    JointSolverConfig joint_cfg = world.GetJointSolverConfig();
    joint_cfg.servoPositionPasses = 0;
    world.SetJointSolverConfig(joint_cfg);

    const HexapodSceneObjects scene = BuildHexapodScene(world);
    RelaxZeroGravityHexapodServos(world, scene);

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

    metrics.finalPosition = world.GetBody(scene.body).position;
    return metrics;
}

} // namespace

int main() {
    const Metrics metrics = RunZeroGravityHexapod();

    constexpr float kMaxLinear = 0.05f;
    constexpr float kMaxAngular = 0.05f;
    constexpr float kMaxError = 1.0e-4f;
    constexpr float kMinFinalY = 0.11f;
    constexpr float kMaxFinalY = 0.175f;

    if (metrics.peakLinear > kMaxLinear) {
        std::cerr << "hex_zero peak_linear=" << metrics.peakLinear << " cap=" << kMaxLinear << "\n";
        return 1;
    }
    if (metrics.peakAngular > kMaxAngular) {
        std::cerr << "hex_zero peak_angular=" << metrics.peakAngular << " cap=" << kMaxAngular << "\n";
        return 1;
    }
    if (metrics.peakError > kMaxError) {
        std::cerr << "hex_zero peak_error=" << metrics.peakError << " cap=" << kMaxError << "\n";
        return 1;
    }
    if (metrics.finalPosition.y < kMinFinalY || metrics.finalPosition.y > kMaxFinalY) {
        std::cerr << "hex_zero final_y=" << metrics.finalPosition.y << " range=[" << kMinFinalY << "," << kMaxFinalY << "]\n";
        return 1;
    }

    return 0;
}
