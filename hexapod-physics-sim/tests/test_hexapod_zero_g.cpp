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
    Real peakJointSpeed = 0.0;
    Vec3 finalPosition{};
};

Metrics RunZeroGravityHexapod() {
    World world({0.0, 0.0, 0.0});
    JointSolverConfig joint_cfg = world.GetJointSolverConfig();
    joint_cfg.servoPositionPasses = 0;
    world.SetJointSolverConfig(joint_cfg);

    const HexapodSceneObjects scene = BuildHexapodScene(world);
    RelaxZeroGravityHexapodServos(world, scene);
    std::array<Real, 18> prevAngles{};
    std::size_t jointWrite = 0;
    for (const LegLinkIds& leg : scene.legs) {
        prevAngles[jointWrite++] = world.GetServoJointAngle(leg.bodyToCoxaJoint);
        prevAngles[jointWrite++] = world.GetServoJointAngle(leg.coxaToFemurJoint);
        prevAngles[jointWrite++] = world.GetServoJointAngle(leg.femurToTibiaJoint);
    }

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
        jointWrite = 0;
        for (const LegLinkIds& leg : scene.legs) {
            const std::array<Real, 3> angles = {
                world.GetServoJointAngle(leg.bodyToCoxaJoint),
                world.GetServoJointAngle(leg.coxaToFemurJoint),
                world.GetServoJointAngle(leg.femurToTibiaJoint)};
            for (Real angle : angles) {
                const Real speed = std::abs(WrapAngle(angle - prevAngles[jointWrite])) / kDt;
                metrics.peakJointSpeed = std::max(metrics.peakJointSpeed, speed);
                prevAngles[jointWrite++] = angle;
            }
        }
    }

    metrics.finalPosition = world.GetBody(scene.body).position;
    return metrics;
}

} // namespace

int main() {
    const Metrics metrics = RunZeroGravityHexapod();

    constexpr Real kMaxLinear = 0.05;
    constexpr Real kMaxAngular = 0.05;
    constexpr Real kMaxError = 1.0e-4;
    constexpr Real kMaxJointSpeed = 8.2;
    constexpr Real kMinFinalY = 0.11;
    constexpr Real kMaxFinalY = 0.175;

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
    if (metrics.peakJointSpeed > kMaxJointSpeed) {
        std::cerr << "hex_zero peak_joint_speed=" << metrics.peakJointSpeed << " cap=" << kMaxJointSpeed << "\n";
        return 1;
    }
    if (metrics.finalPosition.y < kMinFinalY || metrics.finalPosition.y > kMaxFinalY) {
        std::cerr << "hex_zero final_y=" << metrics.finalPosition.y << " range=[" << kMinFinalY << "," << kMaxFinalY << "]\n";
        return 1;
    }

    return 0;
}
