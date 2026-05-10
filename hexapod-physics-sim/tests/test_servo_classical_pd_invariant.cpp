// Classical PD invariant: steady-state error under a constant disturbance must be
// (approximately) independent of the damping ratio ζ. For a continuous PD with
// stiffness k and damping c, x_ss = F/k regardless of c. The discrete sim should
// preserve this within a small discretisation factor that depends only on dt·ωₙ
// (not ζ). Concretely: vary ζ over a wide range with positionGain held fixed,
// and verify steady-state position error stays within a tight band of its
// reference value.
//
// Diagnostic test from the oscillation-sweep showed the existing Catto soft-
// constraint formulation couples ζ into the steady-state error:
//   err = (τ_ext / k) × (1 + dt·ωₙ·(2ζ + dt·ωₙ))
// This test fails on that formulation and passes on a stiffness/damping-decoupled
// PD.

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

// Single hinge under constant gravity disturbance. We measure the post-settle
// MEAN angle (steady-state position) — not the peak, so transient ringing doesn't
// pollute the measurement. For a classical PD: mean angle should be
// (gravity_torque / k) regardless of ζ.
Real MeanAngleAtSteadyState(Real zeta, Real positionGain) {
    World world({0.0, -9.81, 0.0});
    // This test is the regression check for the decoupled stiffness+damping PD
    // formulation. Opt the world into it explicitly — the default is the legacy
    // single-row Catto formulation (which would FAIL this test by design).
    {
        auto cfg = world.GetJointSolverConfig();
        cfg.enableServoStiffnessDampingDecoupling = true;
        world.SetJointSolverConfig(cfg);
    }
    Body base = MakeStaticBase({0.0, 0.10, 0.0});
    const std::uint32_t base_id = world.CreateBody(base);

    // A single arm hanging horizontally from a hinge: gravity provides a constant
    // torque around the hinge axis (here Z), regardless of joint angle (to first
    // order, for small angles).
    constexpr Real kArmLen = 0.10;
    constexpr Real kArmMass = 0.20;
    Body arm = MakeArmLink({base.position.x + 0.5 * kArmLen, base.position.y, 0.0}, kArmLen, kArmMass);
    const std::uint32_t arm_id = world.CreateBody(arm);

    constexpr Real kMaxTorque = 28.0;
    constexpr Real kMaxSpeed = 10.0;
    const std::uint32_t joint = world.CreateServoJoint(
        base_id, arm_id, base.position, {0.0, 0.0, 1.0}, 0.0,
        kMaxTorque, positionGain, zeta, 0.0, 0.5, 0.0, 1.0, kMaxSpeed);

    constexpr Real kDt = 1.0 / 240.0;
    // Long settle window — needs to be long enough that all transients have
    // decayed even for the lowest-damping case.
    for (int s = 0; s < 600; ++s) world.Step(kDt, 24);

    // Average over 1 s post-settle.
    double sum = 0.0;
    constexpr int kMeasure = 240;
    for (int s = 0; s < kMeasure; ++s) {
        world.Step(kDt, 24);
        sum += world.GetServoJointAngle(joint);
    }
    return static_cast<float>(sum / kMeasure);
}

int runCase() {
    constexpr Real kPositionGain = 160.0;
    const Real ref = MeanAngleAtSteadyState(1.24, kPositionGain);
    const Real at4 = MeanAngleAtSteadyState(4.0, kPositionGain);
    const Real at10 = MeanAngleAtSteadyState(10.0, kPositionGain);

    std::cerr << "classical_pd_invariant"
              << " mean_angle_zeta1.24=" << ref
              << " mean_angle_zeta4=" << at4
              << " mean_angle_zeta10=" << at10
              << " ratio_4/1.24=" << (at4 / (ref == 0.0 ? 1e-9 : ref))
              << " ratio_10/1.24=" << (at10 / (ref == 0.0 ? 1e-9 : ref))
              << "\n";

    // Bound: the steady-state angle must stay within ±15 % of the reference value
    // across a 8× change in ζ. This is the classical-PD invariant; any greater
    // drift means damping is being coupled into position correction.
    const Real tolerance = 0.15 * std::abs(ref);
    int failures = 0;
    if (!(std::abs(at4 - ref) <= tolerance)) {
        std::cerr << "  FAIL ζ=4 steady-state error " << at4
                  << " differs from ζ=1.24 reference " << ref
                  << " by more than 15 % (tol=" << tolerance << ")\n";
        ++failures;
    }
    if (!(std::abs(at10 - ref) <= tolerance)) {
        std::cerr << "  FAIL ζ=10 steady-state error " << at10
                  << " differs from ζ=1.24 reference " << ref
                  << " by more than 15 % (tol=" << tolerance << ")\n";
        ++failures;
    }
    return failures == 0 ? 0 : 1;
}

} // namespace

int main() {
    return runCase();
}
