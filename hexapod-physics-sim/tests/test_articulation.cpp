// Unit tests for:
//   1. Spatial math primitives (spatial.hpp)
//   2. Articulation chain detection (BuildArticulationChains)

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "demo/scenes.cpp"       // BuildHexapodScene, SyncBuiltInHexapodServos, etc.
#include "demo/frame_sink.cpp"   // required by scenes.cpp

#include "minphys3d/articulation/types.hpp"
#include "minphys3d/core/body.hpp"
#include "minphys3d/core/world.hpp"
#include "minphys3d/math/spatial.hpp"
#include "minphys3d/math/vec3.hpp"
#include "minphys3d/math/mat3.hpp"

using namespace minphys3d;
using namespace minphys3d::demo;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static void check(bool condition, const char* msg) {
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", msg);
        std::exit(1);
    }
}

static bool approxEq(float a, float b, float tol = 1e-4f) {
    return std::abs(a - b) <= tol;
}

static bool approxEqVec(const Vec3& a, const Vec3& b, float tol = 1e-4f) {
    return approxEq(a.x, b.x, tol) && approxEq(a.y, b.y, tol) && approxEq(a.z, b.z, tol);
}

// ---------------------------------------------------------------------------
// Test 1: Skew matrix correctness  — Skew(v)*u == Cross(v, u)
// ---------------------------------------------------------------------------

static void test_Skew() {
    const Vec3 v{1.0f, 2.0f, 3.0f};
    const Vec3 u{4.0f, 5.0f, 6.0f};
    const Vec3 cross_vu  = Cross(v, u);
    const Vec3 skew_vu   = Skew(v) * u;
    check(approxEqVec(cross_vu, skew_vu), "Skew(v)*u == Cross(v,u)");

    // Also check skew-symmetric: Skew(v)^T == -Skew(v)
    const Mat3 S  = Skew(v);
    const Mat3 St = Transpose(S);
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            check(approxEq(St.m[i][j], -S.m[i][j]), "Skew is skew-symmetric");
}

// ---------------------------------------------------------------------------
// Test 2: SpatialMul — verify dimensions and a known case
// ---------------------------------------------------------------------------

static void test_SpatialMul() {
    // Unit sphere: I_com = 0.4*m*r^2 = 0.4 kg*m^2 for m=1, r=1
    // Reference point = COM  =>  r = 0, mCross = 0
    const float m = 1.0f;
    const float I = 0.4f * m * 1.0f * 1.0f;
    SpatialInertia Ia{};
    Ia.mass       = m;
    Ia.Ioo        = ScaleIdentity(I);
    Ia.mCross     = Mat3{};  // COM at reference

    // Linear velocity only, no angular velocity
    const SpatialVec v_lin{{0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}};
    const SpatialVec h_lin = SpatialMul(Ia, v_lin);
    // h.ang = Ioo * 0 + mCross * v_lin = 0  (mCross is 0 here)
    // h.lin = Transpose(mCross) * 0 + m * v_lin = m * v_lin
    check(approxEqVec(h_lin.ang, {0.0f, 0.0f, 0.0f}), "SpatialMul: h.ang for pure linear v");
    check(approxEqVec(h_lin.lin, {m, 0.0f, 0.0f}),    "SpatialMul: h.lin = m*v");

    // Angular velocity only
    const SpatialVec v_ang{{0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, 0.0f}};
    const SpatialVec h_ang = SpatialMul(Ia, v_ang);
    // h.ang = Ioo * omega = I * omega  (axis z)
    check(approxEqVec(h_ang.ang, {0.0f, 0.0f, I}), "SpatialMul: h.ang = I*omega");
    check(approxEqVec(h_ang.lin, {0.0f, 0.0f, 0.0f}), "SpatialMul: h.lin for pure angular v");
}

// ---------------------------------------------------------------------------
// Test 3: SpatialInertiaFromBody — parallel-axis theorem
// ---------------------------------------------------------------------------

static void test_SpatialInertiaFromBody() {
    // Box: 1 kg, 0.2×0.2×0.2 m
    const float side = 0.2f;
    const float m    = 1.0f;
    const float Ix   = (m / 12.0f) * (side * side + side * side);  // 2/12 * m * side^2
    const Mat3  Iworld = ScaleIdentity(Ix);  // symmetric box

    const Vec3 comWorld{0.0f, 0.1f, 0.0f};  // COM at y=0.1
    const Vec3 refWorld{0.0f, 0.0f, 0.0f};  // reference point at origin

    const SpatialInertia Ia = SpatialInertiaFromBody(m, Iworld, comWorld, refWorld);

    // Check mass
    check(approxEq(Ia.mass, m), "SpatialInertiaFromBody: mass");

    // Check mCross = m * Skew(r), r = com - ref = {0, 0.1, 0}
    const Vec3 r{0.0f, 0.1f, 0.0f};
    const Mat3 expectedmX = m * Skew(r);
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            check(approxEq(Ia.mCross.m[i][j], expectedmX.m[i][j]),
                  "SpatialInertiaFromBody: mCross");

    // Check Ioo (parallel-axis): Ioo = I_com + m*(|r|^2 * I - outer(r,r))
    const float rLen2 = Dot(r, r);
    const Mat3 expectedIoo = Iworld + m * (ScaleIdentity(rLen2) - OuterProduct(r, r));
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            check(approxEq(Ia.Ioo.m[i][j], expectedIoo.m[i][j]),
                  "SpatialInertiaFromBody: Ioo parallel-axis");
}

// ---------------------------------------------------------------------------
// Test 4: PropagateInertia — effective mass and Schur complement
// ---------------------------------------------------------------------------

static void test_PropagateInertia() {
    // Single rigid body (unit sphere, mass=1, inertia I at COM = reference)
    const float m = 1.0f;
    const float Ival = 0.4f;
    SpatialInertia Ic{};
    Ic.mass   = m;
    Ic.Ioo    = ScaleIdentity(Ival);
    Ic.mCross = Mat3{};  // COM at reference

    // Revolute axis along z
    const Vec3 s{0.0f, 0.0f, 1.0f};
    float D = 0.0f;
    const SpatialInertia reduced = PropagateInertia(Ic, s, D);

    // D = s^T * Ic * s = s^T * Ioo * s  (mCross == 0)
    check(approxEq(D, Ival), "PropagateInertia: D == Ival for sphere");

    // After factoring out the 1-DOF around z, the reduced Ioo for the z-axis should be 0
    // (Schur complement removes the z-rotational DOF)
    const float Izz_reduced = Dot(s, reduced.Ioo * s);
    check(approxEq(Izz_reduced, 0.0f), "PropagateInertia: Izz after reduction == 0");

    // Mass should be reduced: m_reduced = m - IsLin^2/D
    // IsLin = Transpose(mCross)*s = 0 (mCross==0), so mass unchanged
    check(approxEq(reduced.mass, m), "PropagateInertia: mass unchanged when mCross==0");
}

// ---------------------------------------------------------------------------
// Test 5: BuildArticulationChains on the hexapod scene
// ---------------------------------------------------------------------------

static void test_BuildArticulationChains_Hexapod() {
    World world({0.0f, -9.81f, 0.0f});
    const HexapodSceneObjects scene = BuildHexapodScene(world);
    // Run one step to trigger BuildArticulationChains()
    world.Step(1.0f / 240.0f, 1);

    const std::vector<ArtChain>& chains = world.GetArticulationChains();

    // Hexapod: 6 legs → 6 chains (one per leg, each [chassis, coxa, femur, tibia]).
    check(chains.size() == 6u, "Hexapod: 6 articulation chains");

    for (std::size_t ci = 0; ci < chains.size(); ++ci) {
        const ArtChain& chain = chains[ci];

        // Each chain has 4 links: chassis (root), coxa, femur, tibia.
        check(chain.links.size() == 4u, "Chain has 4 links (chassis+coxa+femur+tibia)");

        // Root link has no parent joint.
        check(chain.links[0].jointIdx == ArtLink::kNoJoint, "Root link has kNoJoint");
        check(chain.links[0].parent == -1, "Root link parent == -1");

        // Root body is the chassis for all 6 chains.
        check(chain.links[0].bodyIdx == scene.body, "Root link is chassis");

        // Non-root links have valid joint indices.
        for (std::size_t li = 1; li < chain.links.size(); ++li) {
            check(chain.links[li].jointIdx != ArtLink::kNoJoint,
                  "Non-root link has valid jointIdx");
            check(chain.links[li].parent >= 0,
                  "Non-root link has valid parent index");
        }

        // Per-substep caches should be resized to match link count.
        check(chain.Ia.size() == chain.links.size(), "Ia cache size matches links");
        check(chain.Pa.size() == chain.links.size(), "Pa cache size matches links");
        check(chain.D.size()  == chain.links.size(), "D  cache size matches links");
        check(chain.u.size()  == chain.links.size(), "u  cache size matches links");
        check(chain.vel.size()== chain.links.size(), "vel cache size matches links");
    }

    // Verify inArticulationChain flags: all 18 servo joints (3 per leg × 6 legs)
    // should be marked as in-chain.
    std::uint32_t markedCount = 0;
    for (std::uint32_t ji = 0; ji < world.GetServoJointCount(); ++ji) {
        if (world.GetServoJoint(ji).inArticulationChain) {
            ++markedCount;
        }
    }
    check(markedCount == 18u, "All 18 hexapod servo joints marked inArticulationChain");

    // Cross-check: link body indices in each chain match the scene's leg structures.
    // Legs are indexed 0..5 in scene.legs; chains may be in any order, but each chain's
    // non-root bodies should appear exactly once across all leg structures.
    // Build the set of expected {coxa, femur, tibia} triplets.
    std::vector<bool> coxaSeen(scene.legs.size(), false);
    for (const ArtChain& chain : chains) {
        const std::uint32_t coxa_id  = chain.links[1].bodyIdx;
        const std::uint32_t femur_id = chain.links[2].bodyIdx;
        const std::uint32_t tibia_id = chain.links[3].bodyIdx;
        bool foundLeg = false;
        for (std::size_t li = 0; li < scene.legs.size(); ++li) {
            const LegLinkIds& leg = scene.legs[li];
            if (leg.coxa == coxa_id && leg.femur == femur_id && leg.tibia == tibia_id) {
                check(!coxaSeen[li], "Each leg appears in exactly one chain");
                coxaSeen[li] = true;
                foundLeg = true;
                break;
            }
        }
        check(foundLeg, "Chain links match a known leg (coxa/femur/tibia)");
    }
    for (bool seen : coxaSeen) {
        check(seen, "Every leg appears in exactly one chain");
    }
}

// ---------------------------------------------------------------------------
// Test 6: Two-body pendulum chain detection
// ---------------------------------------------------------------------------

static void test_BuildArticulationChains_Pendulum() {
    World world({0.0f, -9.81f, 0.0f});

    // Static anchor body
    Body anchor;
    anchor.shape    = ShapeType::Box;
    anchor.halfExtents = {0.05f, 0.05f, 0.05f};
    anchor.mass     = 1.0f;
    anchor.isStatic = true;
    anchor.position = {0.0f, 1.0f, 0.0f};
    const std::uint32_t anchorId = world.CreateBody(anchor);

    // Pendulum bob
    Body bob;
    bob.shape    = ShapeType::Sphere;
    bob.radius   = 0.05f;
    bob.mass     = 0.5f;
    bob.position = {0.0f, 0.0f, 0.0f};
    const std::uint32_t bobId = world.CreateBody(bob);

    // One servo joint connecting anchor to bob
    const std::uint32_t jointId = world.CreateServoJoint(
        anchorId, bobId,
        /*worldAnchor=*/{0.0f, 1.0f, 0.0f},
        /*worldAxis=*/{1.0f, 0.0f, 0.0f},
        /*targetAngle=*/0.0f,
        /*maxTorque=*/10.0f,
        /*positionGain=*/10.0f,
        /*dampingGain=*/1.0f,
        /*integralGain=*/0.0f,
        /*integralClamp=*/0.5f,
        /*maxCorrectionAngle=*/0.5f,
        /*angleStabilizationScale=*/1.0f,
        /*maxSpeed=*/5.0f,
        /*errorSmoothing=*/0.0f);

    world.Step(1.0f / 240.0f, 1);

    const std::vector<ArtChain>& chains = world.GetArticulationChains();
    check(chains.size() == 1u, "Pendulum: exactly one chain");

    const ArtChain& chain = chains[0];
    check(chain.links.size() == 2u, "Pendulum chain has 2 links");
    check(chain.links[0].bodyIdx == anchorId, "Pendulum root is anchor");
    check(chain.links[1].bodyIdx == bobId,    "Pendulum leaf is bob");
    check(chain.links[1].jointIdx == jointId, "Pendulum leaf's joint is correct");

    check(world.GetServoJoint(jointId).inArticulationChain,
          "Pendulum joint is marked inArticulationChain");
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main() {
    std::printf("test_Skew...\n");
    test_Skew();

    std::printf("test_SpatialMul...\n");
    test_SpatialMul();

    std::printf("test_SpatialInertiaFromBody...\n");
    test_SpatialInertiaFromBody();

    std::printf("test_PropagateInertia...\n");
    test_PropagateInertia();

    std::printf("test_BuildArticulationChains_Pendulum...\n");
    test_BuildArticulationChains_Pendulum();

    std::printf("test_BuildArticulationChains_Hexapod...\n");
    test_BuildArticulationChains_Hexapod();

    std::printf("All articulation tests PASSED.\n");
    return 0;
}
