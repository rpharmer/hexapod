// Localise the single-leg PD steady-state oscillation observed in
// test_single_leg_pd_response. Sweep two axes:
//
//   1. gravity ON vs OFF — does the oscillation depend on a steady disturbance?
//      If OFF still oscillates, it's an intrinsic PD limit cycle, not a gravity
//      response with insufficient integral compensation.
//
//   2. dampingGain ζ ∈ {1.24, 4.0, 10.0} — does increasing declared damping
//      attenuate the observed oscillation? If yes, the integrator is simply
//      under-applying damping (we can compensate with higher ζ). If no, damping
//      is not reaching the joint at all (a real bug).
//
// We use the femur joint of an isolated leg, hold all targets at 0, settle for
// 0.5 s, then measure post-settle joint-velocity RMS and joint-angle ripple.

#include "demo/frame_sink.cpp"
#include "demo/scenes.cpp"
#include "solver_validation_helpers.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>

namespace {

using namespace minphys3d;
using namespace minphys3d::demo;
using namespace minphys3d::tests;

struct OscillationMetrics {
    Real jointVelRmsRadps = 0.0;
    Real jointVelMaxRadps = 0.0;
    Real jointAngleRippleRad = 0.0; // peak deviation from mean over the measurement window.
    Real jointAngleMeanRad = 0.0;
};

OscillationMetrics RunCase(bool with_gravity, Real dampingGain, bool decoupled, Real positionGain = 160.0) {
    World world(with_gravity ? Vec3{0.0, -9.81, 0.0} : Vec3{0.0, 0.0, 0.0});
    if (decoupled) {
        auto cfg = world.GetJointSolverConfig();
        cfg.enableServoStiffnessDampingDecoupling = true;
        world.SetJointSolverConfig(cfg);
    }
    Body base = MakeStaticBase({0.0, 0.10, 0.0});
    const std::uint32_t base_id = world.CreateBody(base);

    constexpr Real kCoxaLen = 0.043;
    constexpr Real kFemurLen = 0.060;
    constexpr Real kTibiaLen = 0.085;
    Body coxa = MakeArmLink({base.position.x + 0.5 * kCoxaLen, base.position.y, 0.0}, kCoxaLen, 0.055);
    const std::uint32_t coxa_id = world.CreateBody(coxa);
    Body femur = MakeArmLink({base.position.x + kCoxaLen + 0.5 * kFemurLen, base.position.y, 0.0},
                             kFemurLen, 0.070);
    const std::uint32_t femur_id = world.CreateBody(femur);
    Body tibia = MakeArmLink(
        {base.position.x + kCoxaLen + kFemurLen + 0.5 * kTibiaLen, base.position.y, 0.0},
        kTibiaLen, 0.063);
    const std::uint32_t tibia_id = world.CreateBody(tibia);

    constexpr Real kMaxTorque = 28.0;
    constexpr Real kMaxSpeed = 10.0;
    world.CreateServoJoint(base_id, coxa_id, base.position, {0.0, 1.0, 0.0}, 0.0,
                           kMaxTorque, positionGain, dampingGain, 0.0, 0.5, 0.0, 1.0, kMaxSpeed);
    const Vec3 femurAnchor = {base.position.x + kCoxaLen, base.position.y, 0.0};
    const std::uint32_t femurJoint = world.CreateServoJoint(
        coxa_id, femur_id, femurAnchor, {0.0, 0.0, 1.0}, 0.0,
        kMaxTorque, positionGain, dampingGain, 0.0, 0.5, 0.0, 1.0, kMaxSpeed);
    const Vec3 tibiaAnchor = {base.position.x + kCoxaLen + kFemurLen, base.position.y, 0.0};
    world.CreateServoJoint(femur_id, tibia_id, tibiaAnchor, {0.0, 0.0, 1.0}, 0.0,
                           kMaxTorque, positionGain, dampingGain, 0.0, 0.5, 0.0, 1.0, kMaxSpeed);

    constexpr Real kDt = 1.0 / 240.0;
    // Settle.
    for (int s = 0; s < 240; ++s) world.Step(kDt, 24);

    // Measure 1 s of post-settle behaviour.
    OscillationMetrics m{};
    constexpr int kMeasureSteps = 240;
    double sumAngle = 0.0;
    double sumSqVel = 0.0;
    std::vector<float> angleSamples;
    angleSamples.reserve(kMeasureSteps);
    for (int s = 0; s < kMeasureSteps; ++s) {
        world.Step(kDt, 24);
        const Body& a = world.GetBody(coxa_id);
        const Body& b = world.GetBody(femur_id);
        const Real jv = b.angularVelocity.z - a.angularVelocity.z;
        const Real jvAbs = std::abs(jv);
        m.jointVelMaxRadps = std::max(m.jointVelMaxRadps, jvAbs);
        sumSqVel += static_cast<double>(jv) * jv;
        const Real angle = world.GetServoJointAngle(femurJoint);
        sumAngle += angle;
        angleSamples.push_back(angle);
    }
    m.jointVelRmsRadps = static_cast<float>(std::sqrt(sumSqVel / kMeasureSteps));
    m.jointAngleMeanRad = static_cast<float>(sumAngle / kMeasureSteps);
    for (const Real a : angleSamples) {
        m.jointAngleRippleRad = std::max(m.jointAngleRippleRad, std::abs(a - m.jointAngleMeanRad));
    }
    return m;
}

int runCase() {
    struct Case { const char* label; bool grav; Real zeta; bool decoupled; };
    const Case cases[] = {
        // Legacy single-row Catto formulation (default).
        {"legacy_grav_zeta1.24",  true,  1.24, false},
        {"legacy_zerog_zeta1.24", false, 1.24, false},
        {"legacy_grav_zeta4",     true,  4.0,  false},
        {"legacy_grav_zeta10",    true,  10.0, false},
        // Decoupled stiffness+damping formulation (opt-in).
        {"decoup_grav_zeta1.24",  true,  1.24, true},
        {"decoup_zerog_zeta1.24", false, 1.24, true},
        {"decoup_grav_zeta4",     true,  4.0,  true},
        {"decoup_grav_zeta10",    true,  10.0, true},
    };
    for (const Case& c : cases) {
        const OscillationMetrics m = RunCase(c.grav, c.zeta, c.decoupled);
        std::cerr << c.label
                  << " mean_angle_rad=" << m.jointAngleMeanRad
                  << " angle_ripple_rad=" << m.jointAngleRippleRad
                  << " joint_vel_rms_radps=" << m.jointVelRmsRadps
                  << " joint_vel_max_radps=" << m.jointVelMaxRadps
                  << "\n";
    }
    // No assertions: this is a diagnostic sweep. Always exit 0 so the trace is
    // collected during regular CI runs and we can read the cells off.
    return 0;
}

} // namespace

int main() {
    return runCase();
}
