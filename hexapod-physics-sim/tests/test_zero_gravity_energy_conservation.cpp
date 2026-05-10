// Zero-gravity energy-conservation test. A closed mechanical system in vacuum (no
// gravity, no contacts, no servos) has constant total kinetic energy. Apply an
// initial impulse, then evolve. Any drift in total KE is solver-induced energy
// leak (or pump). Single-precision should permit ~1e-5 relative drift over 2 s;
// anything >1 % indicates a real solver problem (damping, integrator, constraint
// stabilisation that bleeds energy).

#include "demo/frame_sink.cpp"
#include "demo/scenes.cpp"
#include "solver_validation_helpers.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

namespace {

using namespace minphys3d;
using namespace minphys3d::tests;

// Compute total kinetic energy of a single rigid body. linear KE = 0.5 m v² ;
// rotational KE in body frame = 0.5 ω · I_world ω.
double KineticEnergy(const Body& body) {
    const double v2 = static_cast<double>(LengthSquared(body.velocity));
    double linear = 0.5 * static_cast<double>(body.mass) * v2;
    // Approximate rotational KE using diagonal local inertia (close enough for a
    // box; this scenario uses an axis-aligned box, so cross-terms vanish at t=0).
    Mat3 inertiaLocal{};
    if (!InvertMat3(body.invInertiaLocal, inertiaLocal)) {
        return linear;
    }
    const Vec3& w = body.angularVelocity;
    // Take the body's *world* inertia tensor: R · I_local · R^T
    const Mat3 R = RotationMatrix(body.orientation);
    const Mat3 inertiaWorld = R * inertiaLocal * Transpose(R);
    const Vec3 Iw = inertiaWorld * w;
    const double rotational =
        0.5 * (static_cast<double>(w.x) * Iw.x +
               static_cast<double>(w.y) * Iw.y +
               static_cast<double>(w.z) * Iw.z);
    return linear + rotational;
}

int runCase() {
    World world({0.0, 0.0, 0.0}); // zero gravity, no contacts.

    Body box;
    box.shape = ShapeType::Box;
    box.position = {0.0, 0.0, 0.0};
    box.halfExtents = {0.10, 0.05, 0.04};
    box.mass = 0.5;
    box.restitution = 0.0;
    // Damping should not be applied to a free-flying ballistic body — but the body's
    // damping fields default to 0, so this is a strict zero-loss baseline.
    box.linearDamping = 0.0;
    box.angularDamping = 0.0;
    const std::uint32_t id = world.CreateBody(box);

    // Inject some KE: a translational impulse and an angular impulse around an
    // off-axis vector so the inertia is non-trivial.
    Body& b0 = world.GetBody(id);
    b0.velocity = {0.6, 0.4, -0.3};
    b0.angularVelocity = {1.2, -0.8, 0.5};

    constexpr Real kDt = 1.0 / 240.0;
    constexpr int kSteps = 480; // 2.0 s
    const double initialKE = KineticEnergy(world.GetBody(id));
    double maxDeviation = 0.0;
    double finalKE = initialKE;
    for (int step = 0; step < kSteps; ++step) {
        world.Step(kDt, 16);
        const double ke = KineticEnergy(world.GetBody(id));
        finalKE = ke;
        maxDeviation = std::max(maxDeviation, std::abs(ke - initialKE));
    }
    const double relMaxDeviation = maxDeviation / std::max(initialKE, 1e-9);
    const double relDriftEnd = std::abs(finalKE - initialKE) / std::max(initialKE, 1e-9);

    // Position should be exactly v0 * t with no drift — sanity check the integrator.
    const Body& b = world.GetBody(id);
    const Real t = kDt * kSteps;
    const Vec3 expectedPos = {0.6 * t, 0.4 * t, -0.3 * t};
    const Vec3 posErr = b.position - expectedPos;
    const Real linearTrackingErr = Length(posErr);

    std::cerr << "energy_conservation initialKE=" << initialKE
              << " finalKE=" << finalKE
              << " rel_max_dev=" << relMaxDeviation
              << " rel_drift_end=" << relDriftEnd
              << " linear_tracking_err_m=" << linearTrackingErr << "\n";

    int failures = 0;
    // Bound: 1 % is generous. A correct symplectic integrator with no damping
    // and no constraints should be much tighter (< 1e-4).
    if (!(relMaxDeviation < 0.01)) {
        std::cerr << "  FAIL energy drifted by " << relMaxDeviation * 100.0 << " % (>1 %)\n";
        ++failures;
    }
    // Linear tracking: ballistic trajectory in zero gravity must be exact.
    if (!(linearTrackingErr < 1e-4)) {
        std::cerr << "  FAIL ballistic position tracking err " << linearTrackingErr
                  << " m (expected <0.0001 m)\n";
        ++failures;
    }
    return failures == 0 ? 0 : 1;
}

} // namespace

int main() {
    return runCase();
}
