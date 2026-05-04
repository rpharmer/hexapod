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
#include "minphys3d/core/subsystems.hpp"
#include "minphys3d/core/world.hpp"
#include "minphys3d/demo/hexapod_scene.hpp"
#include "minphys3d/math/spatial.hpp"
#include "minphys3d/math/vec3.hpp"
#include "minphys3d/math/mat3.hpp"

using namespace minphys3d;
using namespace minphys3d::core_internal;
using namespace minphys3d::demo;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static void assertBodyKinematicsFinite(const Body& body);

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

static void test_SpatialMotionCompliance_RigidBody() {
    // Single box at COM reference: compliance should match invMass + ra×n · invI · ra×n
    const float m = 2.0f;
    const float side = 0.2f;
    const float Idiag = (m / 12.0f) * (side * side + side * side);
    const Mat3  Iworld = ScaleIdentity(Idiag);
    const Vec3  com{0.0f, 0.0f, 0.0f};
    const Vec3  ref = com;
    const SpatialInertia Isp = SpatialInertiaFromBody(m, Iworld, com, ref);

    const Vec3 n = Normalize(Vec3{0.6f, 0.8f, 0.0f});
    const Vec3 contact{0.03f, -0.02f, 0.01f};
    const Vec3 ra = contact - com;
    const Vec3 raCrossN = Cross(ra, n);
    const Mat3 invI = ScaleIdentity(1.0f / Idiag);
    const float expected = 1.0f / m + Dot(raCrossN, invI * raCrossN);

    const SpatialVec h{Cross(ra, n), n};
    const float comp = SpatialMotionCompliance(Isp, h);
    check(std::isfinite(comp), "SpatialMotionCompliance finite");
    check(approxEq(comp, expected, 2e-3f), "SpatialMotionCompliance matches rigid-body scalar");
}

static void test_ABI_chain_D_positive() {
    World world({0.0f, -9.81f, 0.0f});
    Body root{};
    root.shape       = ShapeType::Box;
    root.halfExtents = {0.1f, 0.1f, 0.1f};
    root.mass        = 5.0f;
    root.position    = {0.0f, 0.5f, 0.0f};
    const std::uint32_t rootId = world.CreateBody(root);

    Body bob{};
    bob.shape    = ShapeType::Sphere;
    bob.radius   = 0.08f;
    bob.mass     = 0.4f;
    bob.position = {0.2f, 0.5f, 0.0f};
    const std::uint32_t bobId = world.CreateBody(bob);

    world.CreateServoJoint(
        rootId,
        bobId,
        {0.0f, 0.5f, 0.0f},
        {0.0f, 0.0f, 1.0f},
        0.0f,
        5.0f,
        20.0f,
        1.0f,
        0.0f,
        0.5f,
        0.0f,
        1.0f,
        0.0f,
        0.5f);

    world.Step(1.0f / 240.0f, 1);
    const std::vector<ArtChain>& chains = world.GetArticulationChains();
    check(chains.size() == 1u, "ABI test: one chain");
    const ArtChain& ch = chains[0];
    check(ch.links.size() == 2u, "ABI test: two links");
    check(ch.D[1] > 1e-5f, "ABI joint D[1] is positive");
}

static void test_SpatialForceTranslateByOffset() {
    const SpatialVec f{{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
    const Vec3       r{1.0f, 0.0f, 0.0f};
    const SpatialVec g = SpatialForceTranslateByOffset(f, r);
    const Vec3       expectedN = Cross(r, f.lin);
    check(approxEqVec(g.ang, expectedN), "SpatialForceTranslateByOffset: torque from offset force");
    check(approxEqVec(g.lin, f.lin), "SpatialForceTranslateByOffset: force unchanged");
}

static void test_SpatialInertiaSolveMotion_diagonal() {
    const float        m  = 1.0f;
    const float        I0 = 0.4f;
    const Mat3         Iworld = ScaleIdentity(I0);
    const Vec3         com{0.0f, 0.0f, 0.0f};
    const SpatialInertia Isp = SpatialInertiaFromBody(m, Iworld, com, com);
    const SpatialVec     rhs{{1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};
    SpatialVec           x{};
    check(SpatialInertiaSolveMotion(Isp, rhs, x), "SpatialInertiaSolveMotion succeeds");
    check(approxEq(x.ang.x, 1.0f / I0) && approxEq(x.ang.y, 0.0f) && approxEq(x.ang.z, 0.0f),
          "SpatialInertiaSolveMotion: angular part matches diagonal inverse");
    check(approxEqVec(x.lin, Vec3{0.0f, 0.0f, 0.0f}),
          "SpatialInertiaSolveMotion: zero linear RHS block gives zero linear motion");
}

static void test_SpatialInertiaPlucker_kineticEnergyInvariant() {
    const float          m     = 2.0f;
    const Mat3           Icom  = ScaleIdentity(0.3f);
    const Vec3           com{0.1f, -0.05f, 0.02f};
    const Vec3           childRef{-0.2f, 0.4f, 0.1f};
    const SpatialInertia Ic = SpatialInertiaFromBody(m, Icom, com, childRef);
    const Vec3           rChildToParent{0.15f, -0.22f, 0.08f};
    const SpatialInertia Ip = SpatialInertiaPluckerTranslateChildToParent(Ic, rChildToParent);
    const SpatialVec     vc{{0.2f, -0.1f, 0.3f}, {-0.5f, 0.4f, 0.2f}};
    const SpatialVec     vp{vc.ang, vc.lin + Cross(vc.ang, rChildToParent)};
    const float          Tc = DotSpatial(SpatialMul(Ic, vc), vc);
    const float          Tp = DotSpatial(SpatialMul(Ip, vp), vp);
    check(std::isfinite(Tc) && std::isfinite(Tp) && Tc > 0.0f,
          "Plücker KE invariant: child quadratic form finite and positive");
    check(approxEq(Tc, Tp, 1e-4f),
          "Plücker KE invariant: T_child == T_parent under v_parent = X v_child");
}

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
        check(chain.rigidSpatialI.size() == chain.links.size(), "rigidSpatialI cache size matches links");
        check(chain.paJointSnapshot.size() == chain.links.size(), "paJointSnapshot cache size matches links");
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

    // After ABI pass, joint scalar masses and root anchor should be populated.
    for (const ArtChain& chain : chains) {
        check(LengthSquared(chain.links[0].jointAnchorWorld) > 1e-12f,
              "Root link jointAnchorWorld is set (first leg anchor on chassis)");
        for (std::size_t li = 1; li < chain.links.size(); ++li) {
            check(chain.D[li] > 1e-6f, "Hexapod chain joint D is positive");
        }
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

    check(chain.D[1] > 1e-8f, "Pendulum joint D is positive (static root + dynamic bob)");
}

static void test_ArticulationConfig_roundTrip() {
    World world(Vec3{0.0f, 0.0f, 0.0f});
    World::ArticulationConfig cfg{};
    cfg.enableVelocityPreCorrection = true;
    cfg.enableVelocityPreCorrectionKinematicsOnly = true;
    cfg.enableVelocityPreCorrectionFullForwardPass = true;
    cfg.enableChainPositionSolve = true;
    cfg.enableVelocityPreCorrectionChassisCoupling            = true;
    cfg.chassisCouplingGain                                     = 0.5f;
    cfg.chassisCouplingAngularOnly                              = true;
    cfg.enableVelocityPreCorrectionChassisArticulatedTree       = false;
    cfg.enableVelocityPreCorrectionForwardCoriolis              = true;
    cfg.includeServoPdBiasInArticulationU                       = false;
    world.SetArticulationConfig(cfg);
    const World::ArticulationConfig out = world.GetArticulationConfig();
    check(out.enableVelocityPreCorrection, "ArticulationConfig round-trip: velocity pre-correction");
    check(out.enableVelocityPreCorrectionKinematicsOnly,
          "ArticulationConfig round-trip: kinematics-only flag");
    check(out.enableVelocityPreCorrectionFullForwardPass,
          "ArticulationConfig round-trip: full forward pass flag");
    check(out.enableChainPositionSolve, "ArticulationConfig round-trip: chain position solve");
    check(out.enableVelocityPreCorrectionChassisCoupling,
          "ArticulationConfig round-trip: chassis coupling MVP flag");
    check(approxEq(out.chassisCouplingGain, 0.5f), "ArticulationConfig round-trip: chassisCouplingGain");
    check(out.chassisCouplingAngularOnly, "ArticulationConfig round-trip: chassisCouplingAngularOnly");
    check(!out.enableVelocityPreCorrectionChassisArticulatedTree,
          "ArticulationConfig round-trip: chassis tree flag");
    check(out.enableVelocityPreCorrectionForwardCoriolis,
          "ArticulationConfig round-trip: forward Coriolis flag");
    check(!out.includeServoPdBiasInArticulationU,
          "ArticulationConfig round-trip: includeServoPdBiasInArticulationU");
}

static void test_ABI_chain_u_nonzero_withServoBias() {
    World world({0.0f, -9.81f, 0.0f});
    Body anchor{};
    anchor.shape       = ShapeType::Box;
    anchor.halfExtents = {0.05f, 0.05f, 0.05f};
    anchor.mass        = 1.0f;
    anchor.isStatic    = true;
    anchor.position    = {0.0f, 1.0f, 0.0f};
    const std::uint32_t anchorId = world.CreateBody(anchor);

    Body bob{};
    bob.shape    = ShapeType::Sphere;
    bob.radius   = 0.05f;
    bob.mass     = 0.5f;
    bob.position = {0.1f, 0.95f, 0.0f};
    const std::uint32_t bobId = world.CreateBody(bob);

    (void)world.CreateServoJoint(
        anchorId,
        bobId,
        {0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f},
        /*targetAngle=*/0.35f,
        10.0f,
        10.0f,
        1.0f,
        0.0f,
        0.5f,
        1.0f,
        5.0f,
        0.0f);

    World::ArticulationConfig cfg{};
    cfg.enableVelocityPreCorrection               = true;
    cfg.enableVelocityPreCorrectionKinematicsOnly = false;
    world.SetArticulationConfig(cfg);

    world.Step(1.0f / 240.0f, 1);
    const std::vector<ArtChain>& chains = world.GetArticulationChains();
    check(chains.size() == 1u, "ABI u test: one chain");
    const ArtChain& ch = chains[0];
    check(ch.links.size() == 2u, "ABI u test: two links");
    check(std::abs(ch.u[1]) > 1e-4f, "ABI u[1] nonzero when hinge has PD bias (servo error)");
}

static void test_VelocityPreCorrection_fullForward_hexapod_finite() {
    World world(Vec3{0.0f, 0.0f, 0.0f});
    (void)BuildHexapodScene(world);
    World::ArticulationConfig cfg{};
    cfg.enableVelocityPreCorrection                      = true;
    cfg.enableVelocityPreCorrectionKinematicsOnly        = false;
    cfg.enableVelocityPreCorrectionFullForwardPass       = true;
    world.SetArticulationConfig(cfg);
    world.Step(1.0f / 240.0f, 1);
    for (std::uint32_t bi = 0; bi < world.GetBodyCount(); ++bi) {
        assertBodyKinematicsFinite(world.GetBody(bi));
    }
}

static void test_VelocityPreCorrection_fullForward_coriolis_hexapod_finite() {
    World world(Vec3{0.0f, 0.0f, 0.0f});
    (void)BuildHexapodScene(world);
    World::ArticulationConfig cfg{};
    cfg.enableVelocityPreCorrection                      = true;
    cfg.enableVelocityPreCorrectionKinematicsOnly        = false;
    cfg.enableVelocityPreCorrectionFullForwardPass       = true;
    cfg.enableVelocityPreCorrectionForwardCoriolis       = true;
    world.SetArticulationConfig(cfg);
    world.Step(1.0f / 240.0f, 1);
    for (std::uint32_t bi = 0; bi < world.GetBodyCount(); ++bi) {
        assertBodyKinematicsFinite(world.GetBody(bi));
    }
}

static void test_articulation_driveTorque_shifts_chain_u() {
    const Vec3 g{0.0f, -9.81f, 0.0f};
    auto makeWorld = [&](float driveTau, bool pdInU) -> World {
        World world(g);
        Body anchor{};
        anchor.shape       = ShapeType::Box;
        anchor.halfExtents = {0.05f, 0.05f, 0.05f};
        anchor.mass        = 1.0f;
        anchor.isStatic    = true;
        anchor.position    = {0.0f, 1.0f, 0.0f};
        const std::uint32_t anchorId = world.CreateBody(anchor);

        Body bob{};
        bob.shape    = ShapeType::Sphere;
        bob.radius   = 0.05f;
        bob.mass     = 0.5f;
        bob.position = {0.1f, 0.95f, 0.0f};
        const std::uint32_t bobId = world.CreateBody(bob);

        const std::uint32_t jointId = world.CreateServoJoint(
            anchorId,
            bobId,
            {0.0f, 1.0f, 0.0f},
            {0.0f, 0.0f, 1.0f},
            /*targetAngle=*/0.35f,
            10.0f,
            10.0f,
            1.0f,
            0.0f,
            0.5f,
            1.0f,
            5.0f,
            0.0f);

        World::ArticulationConfig cfg{};
        cfg.enableVelocityPreCorrection               = true;
        cfg.enableVelocityPreCorrectionKinematicsOnly = false;
        cfg.includeServoPdBiasInArticulationU         = pdInU;
        world.SetArticulationConfig(cfg);
        world.GetServoJointMutable(jointId).articulationDriveTorque = driveTau;
        world.Step(1.0f / 240.0f, 1);
        return world;
    };

    const World w0 = makeWorld(0.0f, true);
    const World wTau = makeWorld(4.25f, true);
    const float u0   = w0.GetArticulationChains()[0].u[1];
    const float uD   = wTau.GetArticulationChains()[0].u[1];
    check(approxEq(uD - u0, 4.25f, 1e-3f),
          "articulationDriveTorque adds literally to chain u[1] (PD bias in u on)");

    const World wOff = makeWorld(3.0f, false);
    const World wOff0 = makeWorld(0.0f, false);
    check(approxEq(wOff.GetArticulationChains()[0].u[1] - wOff0.GetArticulationChains()[0].u[1], 3.0f, 1e-3f),
          "articulationDriveTorque adds to u when PD bias excluded from u");
}

static void test_star_three_branch_root_Ia_compliance_finite() {
    World world(Vec3{0.0f, 0.0f, 0.0f});

    Body base{};
    base.shape       = ShapeType::Box;
    base.halfExtents = {0.15f, 0.05f, 0.15f};
    base.mass        = 3.0f;
    base.position    = {0.0f, 1.0f, 0.0f};
    const std::uint32_t baseId = world.CreateBody(base);

    auto armBody = [&](const Vec3& pos) {
        Body arm{};
        arm.shape       = ShapeType::Box;
        arm.halfExtents = {0.04f, 0.04f, 0.04f};
        arm.mass        = 0.35f;
        arm.position    = pos;
        return world.CreateBody(arm);
    };

    const std::uint32_t a0 = armBody({0.22f, 1.0f, 0.0f});
    const std::uint32_t a1 = armBody({-0.18f, 1.0f, 0.2f});
    const std::uint32_t a2 = armBody({-0.18f, 1.0f, -0.2f});

    (void)world.CreateServoJoint(
        baseId, a0, {0.15f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}, 0.0f, 8.0f, 25.0f, 1.0f, 0.0f, 0.5f, 0.4f, 1.0f, 6.0f, 0.5f);
    (void)world.CreateServoJoint(
        baseId, a1, {-0.15f, 1.0f, 0.12f}, {0.0f, 1.0f, 0.0f}, 0.1f, 8.0f, 25.0f, 1.0f, 0.0f, 0.5f, 0.4f, 1.0f, 6.0f, 0.5f);
    (void)world.CreateServoJoint(
        baseId, a2, {-0.15f, 1.0f, -0.12f}, {1.0f, 0.0f, 0.0f}, -0.08f, 8.0f, 25.0f, 1.0f, 0.0f, 0.5f, 0.4f, 1.0f, 6.0f, 0.5f);

    World::ArticulationConfig cfg{};
    cfg.enableVelocityPreCorrection               = true;
    cfg.enableVelocityPreCorrectionKinematicsOnly = false;
    world.SetArticulationConfig(cfg);
    world.Step(1.0f / 240.0f, 1);

    const std::vector<ArtChain>& chains = world.GetArticulationChains();
    check(chains.size() == 3u, "star: three serial chains from shared base");
    const SpatialVec hTest{{0.35f, -0.2f, 0.6f}, Normalize(Vec3{0.1f, -0.3f, 0.5f})};
    for (const ArtChain& ch : chains) {
        check(ch.links.size() == 2u, "star: each branch is base + one child");
        check(ch.links[0].bodyIdx == baseId, "star: shared root body");
        check(ch.D[1] > 1e-6f, "star: positive ABI scalar at child joint");
        check(std::isfinite(ch.Ia[0].mass) && ch.Ia[0].mass > 0.5f, "star: root articulated inertia mass sane");
        const float comp = SpatialMotionCompliance(ch.Ia[0], hTest);
        check(std::isfinite(comp) && comp > 0.0f, "star: root SpatialMotionCompliance(Ia[0], h) positive finite");
    }
    for (std::uint32_t bi = 0; bi < world.GetBodyCount(); ++bi) {
        assertBodyKinematicsFinite(world.GetBody(bi));
    }
}

static void test_VelocityPreCorrection_passA_pendulum() {
    World world({0.0f, -9.81f, 0.0f});
    Body anchor{};
    anchor.shape       = ShapeType::Box;
    anchor.halfExtents = {0.05f, 0.05f, 0.05f};
    anchor.mass        = 1.0f;
    anchor.isStatic    = true;
    anchor.position    = {0.0f, 1.0f, 0.0f};
    const std::uint32_t anchorId = world.CreateBody(anchor);

    Body bob{};
    bob.shape    = ShapeType::Sphere;
    bob.radius   = 0.05f;
    bob.mass     = 0.5f;
    bob.position = {0.1f, 0.95f, 0.0f};
    const std::uint32_t bobId = world.CreateBody(bob);

    world.CreateServoJoint(
        anchorId,
        bobId,
        {0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f},
        0.0f,
        10.0f,
        10.0f,
        1.0f,
        0.0f,
        0.5f,
        1.0f,
        5.0f,
        0.0f);

    World::ArticulationConfig cfg{};
    cfg.enableVelocityPreCorrection                   = true;
    cfg.enableVelocityPreCorrectionKinematicsOnly     = true;
    world.SetArticulationConfig(cfg);

    world.Step(1.0f / 240.0f, 1);
    const std::vector<ArtChain>& chains = world.GetArticulationChains();
    check(chains.size() == 1u, "Pass A pendulum: one chain");
    const ArtChain& ch = chains[0];
    check(std::isfinite(ch.vel[1].lin.x) && std::isfinite(ch.vel[1].lin.y) && std::isfinite(ch.vel[1].lin.z),
          "Pass A: child spatial lin velocity is finite");
    check(std::isfinite(ch.vel[1].ang.x) && std::isfinite(ch.vel[1].ang.y) && std::isfinite(ch.vel[1].ang.z),
          "Pass A: child spatial ang velocity is finite");
}

static void assertBodyKinematicsFinite(const Body& body) {
    check(std::isfinite(body.position.x) && std::isfinite(body.position.y) && std::isfinite(body.position.z),
          "body position finite");
    check(std::isfinite(body.velocity.x) && std::isfinite(body.velocity.y) && std::isfinite(body.velocity.z),
          "body velocity finite");
    check(std::isfinite(body.angularVelocity.x) && std::isfinite(body.angularVelocity.y)
            && std::isfinite(body.angularVelocity.z),
          "body angular velocity finite");
}

static void test_VelocityPreCorrection_passC_pa_snapshot_finite_hexapod() {
    World world({0.0f, -9.81f, 0.0f});
    (void)BuildHexapodScene(world);
    World::ArticulationConfig cfg{};
    cfg.enableVelocityPreCorrection               = true;
    cfg.enableVelocityPreCorrectionKinematicsOnly = false;
    world.SetArticulationConfig(cfg);
    world.Step(1.0f / 240.0f, 1);
    const ArtChain& ch = world.GetArticulationChains()[0];
    for (std::size_t li = 1; li < ch.links.size(); ++li) {
        const SpatialVec& p = ch.paJointSnapshot[li];
        check(std::isfinite(p.ang.x) && std::isfinite(p.ang.y) && std::isfinite(p.ang.z)
                && std::isfinite(p.lin.x) && std::isfinite(p.lin.y) && std::isfinite(p.lin.z),
              "Pass B/C: paJointSnapshot is finite at articulated joints");
    }
}

static void test_VelocityPreCorrection_hexapod_finite() {
    World world({0.0f, -9.81f, 0.0f});
    (void)BuildHexapodScene(world);
    World::ArticulationConfig cfg{};
    cfg.enableVelocityPreCorrection               = true;
    cfg.enableVelocityPreCorrectionKinematicsOnly = false;
    world.SetArticulationConfig(cfg);
    world.Step(1.0f / 240.0f, 1);
    for (std::uint32_t bi = 0; bi < world.GetBodyCount(); ++bi) {
        assertBodyKinematicsFinite(world.GetBody(bi));
    }
}

static float absHingeErrorForServoJoint(const World& world, std::uint32_t jointIdx) {
    if (jointIdx == World::kInvalidJointId) {
        return 0.0f;
    }
    const ServoJoint& sj  = world.GetServoJoint(jointIdx);
    const Body&       a   = world.GetBody(sj.a);
    const Body&       b   = world.GetBody(sj.b);
    const float       ang = ComputeServoJointAngle(a, b, sj);
    return std::abs(WrapJointAngle(ang - sj.targetAngle));
}

static void test_hinge_literal_step_monotonicity_hexapod() {
    World world(Vec3{0.0f, 0.0f, 0.0f});
    const HexapodSceneObjects scene = BuildHexapodScene(world);
    JointSolverConfig jc = world.GetJointSolverConfig();
    jc.servoPositionSolveStride = 1u;
    world.SetJointSolverConfig(jc);
    RelaxZeroGravityHexapodServos(world, scene);
    world.Step(1.0f / 240.0f, 1);

    const std::uint32_t kneeJoint = scene.legs[0].femurToTibiaJoint;
    const float         e0k       = absHingeErrorForServoJoint(world, kneeJoint);
    const ServoJoint&   sj0       = world.GetServoJoint(kneeJoint);
    const float         hinge0 =
        ComputeServoJointAngle(world.GetBody(sj0.a), world.GetBody(sj0.b), sj0);
    ServoJoint& sjMut = world.GetServoJointMutable(kneeJoint);
    sjMut.targetAngle = hinge0 + 0.05f;
    const float ePert = absHingeErrorForServoJoint(world, kneeJoint);
    check(ePert > e0k + 1e-5f, "hinge monotonicity: small knee target offset increases that joint error");

    World::ArticulationConfig cfg{};
    cfg.enableChainPositionSolve = true;
    world.SetArticulationConfig(cfg);
    float eTrack = ePert;
    for (int i = 0; i < 96; ++i) {
        world.Step(1.0f / 240.0f, 10);
        const float e = absHingeErrorForServoJoint(world, kneeJoint);
        if (e < eTrack) {
            eTrack = e;
        }
    }
    const float eEnd = absHingeErrorForServoJoint(world, kneeJoint);
    check(std::isfinite(eEnd), "literal Step: knee hinge error finite");
    check(eTrack < ePert - 1e-5f,
          "literal Step sequence (zero-G, stride=1, chain on): knee |hinge error| dips below post-offset");
}

static void test_chassis_coupling_mvp_hexapod_finite() {
    World world(Vec3{0.0f, 0.0f, 0.0f});
    (void)BuildHexapodScene(world);
    World::ArticulationConfig cfg{};
    cfg.enableVelocityPreCorrection               = true;
    cfg.enableVelocityPreCorrectionKinematicsOnly = false;
    cfg.enableVelocityPreCorrectionChassisCoupling = true;
    cfg.chassisCouplingGain                         = 0.35f;
    cfg.chassisCouplingAngularOnly                  = false;
    world.SetArticulationConfig(cfg);
    world.Step(1.0f / 240.0f, 1);
    for (std::uint32_t bi = 0; bi < world.GetBodyCount(); ++bi) {
        assertBodyKinematicsFinite(world.GetBody(bi));
    }
}

static void test_chassis_coupling_tree_hexapod_finite() {
    World world(Vec3{0.0f, 0.0f, 0.0f});
    (void)BuildHexapodScene(world);
    World::ArticulationConfig cfg{};
    cfg.enableVelocityPreCorrection                      = true;
    cfg.enableVelocityPreCorrectionKinematicsOnly      = false;
    cfg.enableVelocityPreCorrectionChassisArticulatedTree = true;
    cfg.chassisCouplingGain                              = 0.25f;
    world.SetArticulationConfig(cfg);
    world.Step(1.0f / 240.0f, 1);
    for (std::uint32_t bi = 0; bi < world.GetBodyCount(); ++bi) {
        assertBodyKinematicsFinite(world.GetBody(bi));
    }
}

static void test_chain_position_solve_hexapod_smoke() {
    World world(Vec3{0.0f, 0.0f, 0.0f});
    const HexapodSceneObjects scene = BuildHexapodScene(world);
    RelaxZeroGravityHexapodServos(world, scene);
    world.Step(1.0f / 240.0f, 1);

    World::ArticulationConfig cfg{};
    cfg.enableChainPositionSolve = true;
    world.SetArticulationConfig(cfg);
    for (int i = 0; i < 20; ++i) {
        world.Step(1.0f / 240.0f, 1);
    }
    for (std::uint32_t bi = 0; bi < world.GetBodyCount(); ++bi) {
        assertBodyKinematicsFinite(world.GetBody(bi));
    }
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

    std::printf("test_SpatialInertiaPlucker_kineticEnergyInvariant...\n");
    test_SpatialInertiaPlucker_kineticEnergyInvariant();

    std::printf("test_SpatialForceTranslateByOffset...\n");
    test_SpatialForceTranslateByOffset();

    std::printf("test_SpatialInertiaSolveMotion_diagonal...\n");
    test_SpatialInertiaSolveMotion_diagonal();

    std::printf("test_SpatialMotionCompliance_RigidBody...\n");
    test_SpatialMotionCompliance_RigidBody();

    std::printf("test_ABI_chain_D_positive...\n");
    test_ABI_chain_D_positive();

    std::printf("test_ArticulationConfig_roundTrip...\n");
    test_ArticulationConfig_roundTrip();

    std::printf("test_ABI_chain_u_nonzero_withServoBias...\n");
    test_ABI_chain_u_nonzero_withServoBias();

    std::printf("test_VelocityPreCorrection_passA_pendulum...\n");
    test_VelocityPreCorrection_passA_pendulum();

    std::printf("test_VelocityPreCorrection_passC_pa_snapshot_finite_hexapod...\n");
    test_VelocityPreCorrection_passC_pa_snapshot_finite_hexapod();

    std::printf("test_VelocityPreCorrection_hexapod_finite...\n");
    test_VelocityPreCorrection_hexapod_finite();

    std::printf("test_VelocityPreCorrection_fullForward_hexapod_finite...\n");
    test_VelocityPreCorrection_fullForward_hexapod_finite();

    std::printf("test_VelocityPreCorrection_fullForward_coriolis_hexapod_finite...\n");
    test_VelocityPreCorrection_fullForward_coriolis_hexapod_finite();

    std::printf("test_articulation_driveTorque_shifts_chain_u...\n");
    test_articulation_driveTorque_shifts_chain_u();

    std::printf("test_star_three_branch_root_Ia_compliance_finite...\n");
    test_star_three_branch_root_Ia_compliance_finite();

    std::printf("test_chain_position_solve_hexapod_smoke...\n");
    test_chain_position_solve_hexapod_smoke();

    std::printf("test_hinge_literal_step_monotonicity_hexapod...\n");
    test_hinge_literal_step_monotonicity_hexapod();

    std::printf("test_chassis_coupling_mvp_hexapod_finite...\n");
    test_chassis_coupling_mvp_hexapod_finite();

    std::printf("test_chassis_coupling_tree_hexapod_finite...\n");
    test_chassis_coupling_tree_hexapod_finite();

    std::printf("test_BuildArticulationChains_Pendulum...\n");
    test_BuildArticulationChains_Pendulum();

    std::printf("test_BuildArticulationChains_Hexapod...\n");
    test_BuildArticulationChains_Hexapod();

    std::printf("All articulation tests PASSED.\n");
    return 0;
}
