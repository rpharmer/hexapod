#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>

#include "demo/frame_sink.cpp"
#include "demo/scenes.cpp"
#include "minphys3d/demo/hexapod_stability.hpp"
#include "solver_validation_helpers.hpp"

namespace {

using namespace minphys3d;
using namespace minphys3d::demo;
using namespace minphys3d::tests;

struct PoseHoldMetrics {
    float peakLinear = 0.0f;
    float peakAngular = 0.0f;
    float peakJointError = 0.0f;
    float finalHeight = 0.0f;
    bool finite = true;
};

PoseHoldMetrics runPoseHold(float payload_mass_kg) {
    World world({0.0f, -9.81f, 0.0f});
    const HexapodSceneObjects scene = BuildHexapodScene(world);
    RelaxBuiltInHexapodServos(world, scene);
    ApplyHexapodPoseHoldStabilityTuning(world, scene);

    Body& chassis = world.GetBody(scene.body);
    chassis.mass += payload_mass_kg;
    chassis.RecomputeMassProperties();

    PoseHoldMetrics m{};
    constexpr float kDt = 1.0f / 240.0f;
    for (int step = 0; step < 600; ++step) {
        world.Step(kDt, 24);
        const Body& c = world.GetBody(scene.body);
        if (!IsFiniteVec3(c.position) || !IsFiniteVec3(c.velocity) || !IsFiniteVec3(c.angularVelocity)
            || !IsFiniteQuat(c.orientation)) {
            m.finite = false;
            break;
        }
        m.peakLinear = std::max(m.peakLinear, Length(c.velocity));
        m.peakAngular = std::max(m.peakAngular, Length(c.angularVelocity));
        for (const LegLinkIds& leg : scene.legs) {
            m.peakJointError = std::max(
                m.peakJointError,
                std::abs(WrapAngle(world.GetServoJointAngle(leg.bodyToCoxaJoint) - world.GetServoJoint(leg.bodyToCoxaJoint).targetAngle)));
            m.peakJointError = std::max(
                m.peakJointError,
                std::abs(WrapAngle(world.GetServoJointAngle(leg.coxaToFemurJoint) - world.GetServoJoint(leg.coxaToFemurJoint).targetAngle)));
            m.peakJointError = std::max(
                m.peakJointError,
                std::abs(WrapAngle(world.GetServoJointAngle(leg.femurToTibiaJoint) - world.GetServoJoint(leg.femurToTibiaJoint).targetAngle)));
        }
    }
    m.finalHeight = world.GetBody(scene.body).position.y;
    return m;
}

int runCase() {
    const PoseHoldMetrics supported = runPoseHold(0.4f);
    const PoseHoldMetrics overload = runPoseHold(2.6f);

    if (!supported.finite || !overload.finite) {
        std::cerr << "hexapod_payload non-finite state observed\n";
        return 1;
    }
    if (supported.finalHeight < 0.02f) {
        std::cerr << "hexapod_payload supported_final_height=" << supported.finalHeight << " floor=0.02\n";
        return 1;
    }
    if (supported.peakLinear > 6.0f || supported.peakJointError > 3.0f) {
        std::cerr << "hexapod_payload supported peak_linear=" << supported.peakLinear
                  << " peak_joint_error=" << supported.peakJointError << "\n";
        return 1;
    }
    if (overload.peakLinear > 200.0f || overload.peakAngular > 200.0f || overload.peakJointError > 6.3f) {
        std::cerr << "hexapod_payload overload instability peak_linear=" << overload.peakLinear
                  << " peak_angular=" << overload.peakAngular
                  << " peak_joint_error=" << overload.peakJointError << "\n";
        return 1;
    }
    if (overload.finalHeight > supported.finalHeight + 0.03f) {
        std::cerr << "hexapod_payload overload did not degrade as expected overload_h=" << overload.finalHeight
                  << " supported_h=" << supported.finalHeight << "\n";
        return 1;
    }
    return 0;
}

} // namespace

int main() {
    return runCase();
}
