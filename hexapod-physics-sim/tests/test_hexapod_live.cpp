#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>

#include "demo/frame_sink.cpp"
#include "demo/scenes.cpp"

namespace {

using namespace minphys3d;
using namespace minphys3d::demo;

Real WrapAngle(Real angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

struct Metrics {
    Real peakLinear = 0.0;
    Real peakAngular = 0.0;
    Real peakError = 0.0;
    Real finalSpeed = 0.0;
    Vec3 finalPosition{};
};

Metrics RunLiveHexapod() {
    World world({0.0, -9.81, 0.0});
    JointSolverConfig joint_cfg = world.GetJointSolverConfig();
    joint_cfg.servoPositionPasses = 0;
    joint_cfg.hingeAnchorBiasFactor = 0.25;
    joint_cfg.hingeAnchorDampingFactor = 0.3;
    world.SetJointSolverConfig(joint_cfg);

    const HexapodSceneObjects scene = BuildHexapodScene(world);
    RelaxBuiltInHexapodServos(world, scene);

    Metrics metrics;
    constexpr Real kDt = 1.0 / 240.0;
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

    constexpr Real kMaxLinear = 1.5;
    constexpr Real kMaxAngular = 0.1;
    constexpr Real kMaxError = 2.0;
    constexpr Real kMaxFinalSpeed = 0.05;
    constexpr Real kMinFinalY = 0.030;

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
