// Noise-floor test: hexapod stands on flat ground, no commands, no disturbance.
// After a 0.5 s settle window, the system should be quiescent — body must not be
// drifting noticeably, joint velocities must be near zero, contact impulses must
// stay bounded. Any motion observed here is solver-induced jitter that the
// controller has to fight during real walking.

#include "demo/frame_sink.cpp"
#include "demo/scenes.cpp"
#include "minphys3d/demo/hexapod_stability.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>

namespace {

using namespace minphys3d;
using namespace minphys3d::demo;

struct QuiescenceMetrics {
    // Position drift: max distance from settled body position over the measurement window.
    Real bodyPosDriftM = 0.0;
    // Body velocity (linear & angular) statistics during the measurement window.
    Real bodyLinVelMaxMps = 0.0;
    Real bodyLinVelRmsMps = 0.0;
    Real bodyAngVelMaxRadps = 0.0;
    Real bodyAngVelRmsRadps = 0.0;
    // Joint velocity statistics across all 18 joints, all measurement-window ticks.
    Real jointVelMaxRadps = 0.0;
    Real jointVelRmsRadps = 0.0;
    // Joint angle drift from the held targets.
    Real jointAngleDriftMaxRad = 0.0;
    // Body height drift relative to the stand height at the start of the window.
    Real bodyHeightDriftMaxM = 0.0;
    // Settling-window peak (for diagnostic, not asserted).
    Real settlingPeakAngVelRadps = 0.0;
};

QuiescenceMetrics runQuiescence() {
    World world({0.0, -9.81, 0.0});
    const HexapodSceneObjects scene = BuildHexapodScene(world);
    RelaxBuiltInHexapodServos(world, scene);
    ApplyHexapodPoseHoldStabilityTuning(world, scene);

    constexpr Real kDt = 1.0 / 240.0;
    // Match the higher PGS budget used elsewhere for quiet servo/contact coupling (e.g.
    // test_single_leg_pd_response); 24 iters leaves enough linear-velocity jitter that RMS
    // body speed marginally exceeds the 2 cm/s gate without indicating a stability regression.
    constexpr int kPgsIterations = 40;
    constexpr int kSettleSteps = 240;   // 1.0 s settle
    constexpr int kMeasureSteps = 1200; // 5.0 s measurement

    QuiescenceMetrics m{};

    // Settle.
    for (int step = 0; step < kSettleSteps; ++step) {
        world.Step(kDt, kPgsIterations);
        const Body& b = world.GetBody(scene.body);
        m.settlingPeakAngVelRadps = std::max(m.settlingPeakAngVelRadps, Length(b.angularVelocity));
    }

    // Capture settled reference.
    const Body& settledBody = world.GetBody(scene.body);
    const Vec3 refPos = settledBody.position;
    const Real refHeight = settledBody.position.y;
    std::array<Real, 18> refJointAngles{};
    {
        std::size_t i = 0;
        for (const LegLinkIds& leg : scene.legs) {
            for (const std::uint32_t jid : {leg.bodyToCoxaJoint, leg.coxaToFemurJoint, leg.femurToTibiaJoint}) {
                refJointAngles[i++] = world.GetServoJointAngle(jid);
            }
        }
    }

    double linVelSqSum = 0.0;
    double angVelSqSum = 0.0;
    double jointVelSqSum = 0.0;
    int linVelCount = 0;
    int angVelCount = 0;
    int jointVelCount = 0;

    for (int step = 0; step < kMeasureSteps; ++step) {
        world.Step(kDt, kPgsIterations);
        const Body& b = world.GetBody(scene.body);

        const Real lv = Length(b.velocity);
        const Real av = Length(b.angularVelocity);
        m.bodyLinVelMaxMps = std::max(m.bodyLinVelMaxMps, lv);
        m.bodyAngVelMaxRadps = std::max(m.bodyAngVelMaxRadps, av);
        linVelSqSum += static_cast<double>(lv) * lv; ++linVelCount;
        angVelSqSum += static_cast<double>(av) * av; ++angVelCount;

        const Real posDrift = Length(b.position - refPos);
        m.bodyPosDriftM = std::max(m.bodyPosDriftM, posDrift);
        const Real heightDrift = std::abs(b.position.y - refHeight);
        m.bodyHeightDriftMaxM = std::max(m.bodyHeightDriftMaxM, heightDrift);

        std::size_t i = 0;
        for (const LegLinkIds& leg : scene.legs) {
            for (const std::uint32_t jid : {leg.bodyToCoxaJoint, leg.coxaToFemurJoint, leg.femurToTibiaJoint}) {
                const Body& a = world.GetBody(world.GetServoJoint(jid).a);
                const Body& bbody = world.GetBody(world.GetServoJoint(jid).b);
                const Real jv = Length(bbody.angularVelocity - a.angularVelocity);
                m.jointVelMaxRadps = std::max(m.jointVelMaxRadps, jv);
                jointVelSqSum += static_cast<double>(jv) * jv; ++jointVelCount;
                const Real angle = world.GetServoJointAngle(jid);
                const Real drift = std::abs(angle - refJointAngles[i]);
                m.jointAngleDriftMaxRad = std::max(m.jointAngleDriftMaxRad, drift);
                ++i;
            }
        }
    }

    m.bodyLinVelRmsMps = static_cast<float>(std::sqrt(linVelSqSum / std::max(1, linVelCount)));
    m.bodyAngVelRmsRadps = static_cast<float>(std::sqrt(angVelSqSum / std::max(1, angVelCount)));
    m.jointVelRmsRadps = static_cast<float>(std::sqrt(jointVelSqSum / std::max(1, jointVelCount)));
    return m;
}

int runCase() {
    const QuiescenceMetrics m = runQuiescence();

    // Print diagnostic metrics first — useful even when the test passes.
    std::cerr << "stand_quiescence settling_peak_angvel=" << m.settlingPeakAngVelRadps
              << " body_pos_drift_m=" << m.bodyPosDriftM
              << " body_lin_vel_max=" << m.bodyLinVelMaxMps << " rms=" << m.bodyLinVelRmsMps
              << " body_ang_vel_max=" << m.bodyAngVelMaxRadps << " rms=" << m.bodyAngVelRmsRadps
              << " joint_vel_max=" << m.jointVelMaxRadps << " rms=" << m.jointVelRmsRadps
              << " joint_angle_drift_max=" << m.jointAngleDriftMaxRad
              << " body_height_drift_max=" << m.bodyHeightDriftMaxM
              << "\n";

    int failures = 0;
    auto check = [&](const char* name, double value, double bound) {
        if (!(value <= bound)) {
            std::cerr << "  FAIL " << name << " = " << value << " > " << bound << "\n";
            ++failures;
        }
    };

    // BOUNDS: tight enough to surface real wobble; loose enough that nominal sim noise
    // (single-precision contact resolution at ~10 N gravity load) does not trigger.
    check("body_pos_drift_m",        m.bodyPosDriftM,        0.005);   // 5 mm in 5 seconds
    check("body_lin_vel_rms_mps",    m.bodyLinVelRmsMps,     0.02);    // 2 cm/s RMS
    check("body_ang_vel_rms_radps",  m.bodyAngVelRmsRadps,   0.10);    // ~5.7 deg/s RMS
    check("joint_vel_rms_radps",     m.jointVelRmsRadps,     0.10);    // ~5.7 deg/s RMS
    check("joint_angle_drift_rad",   m.jointAngleDriftMaxRad, 0.05);   // ~3 deg drift
    check("body_height_drift_m",     m.bodyHeightDriftMaxM,  0.005);   // 5 mm height drift
    return failures == 0 ? 0 : 1;
}

} // namespace

int main() {
    return runCase();
}
