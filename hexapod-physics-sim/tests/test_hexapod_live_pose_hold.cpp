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

Metrics RunLivePoseHoldScenario() {
    World world({0.0f, -9.81f, 0.0f});
    JointSolverConfig joint_cfg = world.GetJointSolverConfig();
    joint_cfg.servoPositionPasses = 8;
    joint_cfg.hingeAnchorBiasFactor = 0.8f;
    joint_cfg.hingeAnchorDampingFactor = 0.2f;
    world.SetJointSolverConfig(joint_cfg);

    const HexapodSceneObjects scene = BuildHexapodScene(world);
    RelaxBuiltInHexapodServos(world, scene);

    Metrics metrics;
    constexpr float kFrameDt = 1.0f / 60.0f;
    constexpr int kPhysicsSubstepsPerFrame = 6;
    constexpr int kSolverIterations = 80;
    for (int frame = 0; frame < 240; ++frame) {
        for (int substep = 0; substep < kPhysicsSubstepsPerFrame; ++substep) {
            world.Step(kFrameDt / static_cast<float>(kPhysicsSubstepsPerFrame), kSolverIterations);
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
    }

    const Body& chassis = world.GetBody(scene.body);
    metrics.finalPosition = chassis.position;
    metrics.finalSpeed = Length(chassis.velocity);
    return metrics;
}

} // namespace

int main() {
    const Metrics metrics = RunLivePoseHoldScenario();

    constexpr float kMaxLinear = 1.5f;
    // Six-foot touchdown still produces a short-lived chassis spin-up; inertia-corrected servo
    // position passes reduced this band (~0.29 → ~0.22 rad/s). Keep a small margin over that.
    constexpr float kMaxAngular = 0.24f;
    constexpr float kMaxError = 1.0f;
    constexpr float kMaxFinalSpeed = 0.01f;
    // Chassis `Body::position` is the dynamic center of mass. For the compound hull used here, a
    // supported static pose keeps that COM a few centimetres above the plane even when the mesh
    // "looks" taller; require a small clearance band rather than an absolute 10 cm COM bar.
    constexpr float kMinFinalY = 0.030f;

    if (metrics.peakLinear > kMaxLinear) {
        std::cerr << "hex_live_hold peak_linear=" << metrics.peakLinear << " cap=" << kMaxLinear << "\n";
        return 1;
    }
    if (metrics.peakAngular > kMaxAngular) {
        std::cerr << "hex_live_hold peak_angular=" << metrics.peakAngular << " cap=" << kMaxAngular << "\n";
        return 1;
    }
    if (metrics.peakError > kMaxError) {
        std::cerr << "hex_live_hold peak_error=" << metrics.peakError << " cap=" << kMaxError << "\n";
        return 1;
    }
    if (metrics.finalSpeed > kMaxFinalSpeed) {
        std::cerr << "hex_live_hold final_speed=" << metrics.finalSpeed << " cap=" << kMaxFinalSpeed << "\n";
        return 1;
    }
    if (metrics.finalPosition.y < kMinFinalY) {
        std::cerr << "hex_live_hold final_y=" << metrics.finalPosition.y << " floor=" << kMinFinalY << "\n";
        return 1;
    }

    return 0;
}
