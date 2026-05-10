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

static bool approxEq(Real a, Real b, Real tol = 1e-4) {
    return std::abs(a - b) <= tol;
}

static bool approxEqVec(const Vec3& a, const Vec3& b, Real tol = 1e-4) {
    return approxEq(a.x, b.x, tol) && approxEq(a.y, b.y, tol) && approxEq(a.z, b.z, tol);
}

static bool approxEqMat3(const Mat3& a, const Mat3& b, Real tol = 1e-4) {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            if (!approxEq(a.m[i][j], b.m[i][j], tol)) {
                return false;
            }
        }
    }
    return true;
}

// ---------------------------------------------------------------------------
// Test 1: Skew matrix correctness  — Skew(v)*u == Cross(v, u)
// ---------------------------------------------------------------------------

static void test_Skew() {
    const Vec3 v{1.0, 2.0, 3.0};
    const Vec3 u{4.0, 5.0, 6.0};
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
    const Real m = 1.0;
    const Real I = 0.4 * m * 1.0 * 1.0;
    SpatialInertia Ia{};
    Ia.mass       = m;
    Ia.Ioo        = ScaleIdentity(I);
    Ia.mCross     = Mat3{};  // COM at reference

    // Linear velocity only, no angular velocity
    const SpatialVec v_lin{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}};
    const SpatialVec h_lin = SpatialMul(Ia, v_lin);
    // h.ang = Ioo * 0 + mCross * v_lin = 0  (mCross is 0 here)
    // h.lin = Transpose(mCross) * 0 + m * v_lin = m * v_lin
    check(approxEqVec(h_lin.ang, {0.0, 0.0, 0.0}), "SpatialMul: h.ang for pure linear v");
    check(approxEqVec(h_lin.lin, {m, 0.0, 0.0}),    "SpatialMul: h.lin = m*v");

    // Angular velocity only
    const SpatialVec v_ang{{0.0, 0.0, 1.0}, {0.0, 0.0, 0.0}};
    const SpatialVec h_ang = SpatialMul(Ia, v_ang);
    // h.ang = Ioo * omega = I * omega  (axis z)
    check(approxEqVec(h_ang.ang, {0.0, 0.0, I}), "SpatialMul: h.ang = I*omega");
    check(approxEqVec(h_ang.lin, {0.0, 0.0, 0.0}), "SpatialMul: h.lin for pure angular v");
}

// ---------------------------------------------------------------------------
// Test 3: SpatialInertiaFromBody — parallel-axis theorem
// ---------------------------------------------------------------------------

static void test_SpatialInertiaFromBody() {
    // Box: 1 kg, 0.2×0.2×0.2 m
    const Real side = 0.2;
    const Real m    = 1.0;
    const Real Ix   = (m / 12.0) * (side * side + side * side);  // 2/12 * m * side^2
    const Mat3  Iworld = ScaleIdentity(Ix);  // symmetric box

    const Vec3 comWorld{0.0, 0.1, 0.0};  // COM at y=0.1
    const Vec3 refWorld{0.0, 0.0, 0.0};  // reference point at origin

    const SpatialInertia Ia = SpatialInertiaFromBody(m, Iworld, comWorld, refWorld);

    // Check mass
    check(approxEq(Ia.mass, m), "SpatialInertiaFromBody: mass");

    // Check mCross = m * Skew(r), r = com - ref = {0, 0.1, 0}
    const Vec3 r{0.0, 0.1, 0.0};
    const Mat3 expectedmX = m * Skew(r);
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            check(approxEq(Ia.mCross.m[i][j], expectedmX.m[i][j]),
                  "SpatialInertiaFromBody: mCross");

    // Check Ioo (parallel-axis): Ioo = I_com + m*(|r|^2 * I - outer(r,r))
    const Real rLen2 = Dot(r, r);
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
    const Real m = 2.0;
    const Real side = 0.2;
    const Real Idiag = (m / 12.0) * (side * side + side * side);
    const Mat3  Iworld = ScaleIdentity(Idiag);
    const Vec3  com{0.0, 0.0, 0.0};
    const Vec3  ref = com;
    const SpatialInertia Isp = SpatialInertiaFromBody(m, Iworld, com, ref);

    const Vec3 n = Normalize(Vec3{0.6, 0.8, 0.0});
    const Vec3 contact{0.03, -0.02, 0.01};
    const Vec3 ra = contact - com;
    const Vec3 raCrossN = Cross(ra, n);
    const Mat3 invI = ScaleIdentity(1.0 / Idiag);
    const Real expected = 1.0 / m + Dot(raCrossN, invI * raCrossN);

    const SpatialVec h{Cross(ra, n), n};
    const Real comp = SpatialMotionCompliance(Isp, h);
    check(std::isfinite(comp), "SpatialMotionCompliance finite");
    check(approxEq(comp, expected, 2e-3), "SpatialMotionCompliance matches rigid-body scalar");
}

static void test_ABI_chain_D_positive() {
    World world({0.0, -9.81, 0.0});
    Body root{};
    root.shape       = ShapeType::Box;
    root.halfExtents = {0.1, 0.1, 0.1};
    root.mass        = 5.0;
    root.position    = {0.0, 0.5, 0.0};
    const std::uint32_t rootId = world.CreateBody(root);

    Body bob{};
    bob.shape    = ShapeType::Sphere;
    bob.radius   = 0.08;
    bob.mass     = 0.4;
    bob.position = {0.2, 0.5, 0.0};
    const std::uint32_t bobId = world.CreateBody(bob);

    world.CreateServoJoint(
        rootId,
        bobId,
        {0.0, 0.5, 0.0},
        {0.0, 0.0, 1.0},
        0.0,
        5.0,
        20.0,
        1.0,
        0.0,
        0.5,
        0.0,
        1.0,
        0.0,
        0.5);

    world.Step(1.0 / 240.0, 1);
    const std::vector<ArtChain>& chains = world.GetArticulationChains();
    check(chains.size() == 1u, "ABI test: one chain");
    const ArtChain& ch = chains[0];
    check(ch.links.size() == 2u, "ABI test: two links");
    check(ch.D[1] > 1e-5, "ABI joint D[1] is positive");
}

static void test_SpatialForceTranslateByOffset() {
    const SpatialVec f{{0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}};
    const Vec3       r{1.0, 0.0, 0.0};
    const SpatialVec g = SpatialForceTranslateByOffset(f, r);
    const Vec3       expectedN = Cross(r, f.lin);
    check(approxEqVec(g.ang, expectedN), "SpatialForceTranslateByOffset: torque from offset force");
    check(approxEqVec(g.lin, f.lin), "SpatialForceTranslateByOffset: force unchanged");
}

static void test_SpatialInertiaSolveMotion_diagonal() {
    const Real        m  = 1.0;
    const Real        I0 = 0.4;
    const Mat3         Iworld = ScaleIdentity(I0);
    const Vec3         com{0.0, 0.0, 0.0};
    const SpatialInertia Isp = SpatialInertiaFromBody(m, Iworld, com, com);
    const SpatialVec     rhs{{1.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
    SpatialVec           x{};
    check(SpatialInertiaSolveMotion(Isp, rhs, x), "SpatialInertiaSolveMotion succeeds");
    check(approxEq(x.ang.x, 1.0 / I0) && approxEq(x.ang.y, 0.0) && approxEq(x.ang.z, 0.0),
          "SpatialInertiaSolveMotion: angular part matches diagonal inverse");
    check(approxEqVec(x.lin, Vec3{0.0, 0.0, 0.0}),
          "SpatialInertiaSolveMotion: zero linear RHS block gives zero linear motion");
}

static void test_SpatialInertiaSolveMotion_nearSingular_returns_safe_result() {
    SpatialInertia I{};
    I.mass = 1.0e-9;
    I.Ioo = Mat3{{
        {1.0e-10, 0.0, 0.0},
        {0.0, 2.0e-10, 0.0},
        {0.0, 0.0, 1.0e-12},
    }};
    SpatialVec rhs{{1.0, -0.25, 0.5}, {0.0, 0.0, 0.0}};
    SpatialVec x{};
    SpatialSolveDiagnostics diagnostics{};
    const bool ok = spatial_internal::SpatialInertiaSolveMotionDetailed(I, rhs, x, &diagnostics);
    check(diagnostics.regularized, "near-singular solve reports regularization usage");
    check(!diagnostics.failed || !ok, "near-singular solve only reports failure when it rejects");
    if (ok) {
        check(std::isfinite(x.ang.x) && std::isfinite(x.ang.y) && std::isfinite(x.ang.z),
              "near-singular solve keeps angular solution finite");
        check(std::isfinite(x.lin.x) && std::isfinite(x.lin.y) && std::isfinite(x.lin.z),
              "near-singular solve keeps linear solution finite");
    }
}

static void test_SpatialMotionCompliance_singular_is_safe() {
    const SpatialInertia I{};
    const SpatialVec h{{0.0, 1.0, 0.0}, {0.25, 0.0, 0.0}};
    SpatialSolveDiagnostics diagnostics{};
    const Real compliance = spatial_internal::SpatialMotionComplianceDetailed(I, h, &diagnostics);
    check(diagnostics.regularized, "singular spatial compliance reports regularization usage");
    check(std::isinf(compliance) || (std::isfinite(compliance) && compliance > 0.0),
          "singular spatial compliance either rejects or returns a positive finite regularized result");
}

static void test_SpatialInertiaPlucker_kineticEnergyInvariant() {
    const Real          m     = 2.0;
    const Mat3           Icom  = ScaleIdentity(0.3);
    const Vec3           com{0.1, -0.05, 0.02};
    const Vec3           childRef{-0.2, 0.4, 0.1};
    const SpatialInertia Ic = SpatialInertiaFromBody(m, Icom, com, childRef);
    const Vec3           rChildToParent{0.15, -0.22, 0.08};
    const SpatialInertia Ip = SpatialInertiaPluckerTranslateChildToParent(Ic, rChildToParent);
    const SpatialVec     vc{{0.2, -0.1, 0.3}, {-0.5, 0.4, 0.2}};
    const SpatialVec     vp{vc.ang, vc.lin + Cross(vc.ang, rChildToParent)};
    const Real          Tc = DotSpatial(SpatialMul(Ic, vc), vc);
    const Real          Tp = DotSpatial(SpatialMul(Ip, vp), vp);
    check(std::isfinite(Tc) && std::isfinite(Tp) && Tc > 0.0,
          "Plücker KE invariant: child quadratic form finite and positive");
    check(approxEq(Tc, Tp, 1e-4),
          "Plücker KE invariant: T_child == T_parent under v_parent = X v_child");
}

static void test_PropagateInertia() {
    // Single rigid body (unit sphere, mass=1, inertia I at COM = reference)
    const Real m = 1.0;
    const Real Ival = 0.4;
    SpatialInertia Ic{};
    Ic.mass   = m;
    Ic.Ioo    = ScaleIdentity(Ival);
    Ic.mCross = Mat3{};  // COM at reference

    // Revolute axis along z
    const Vec3 s{0.0, 0.0, 1.0};
    Real D = 0.0;
    const SpatialInertia reduced = PropagateInertia(Ic, s, D);

    // D = s^T * Ic * s = s^T * Ioo * s  (mCross == 0)
    check(approxEq(D, Ival), "PropagateInertia: D == Ival for sphere");

    // After factoring out the 1-DOF around z, the reduced Ioo for the z-axis should be 0
    // (Schur complement removes the z-rotational DOF)
    const Real Izz_reduced = Dot(s, reduced.Ioo * s);
    check(approxEq(Izz_reduced, 0.0), "PropagateInertia: Izz after reduction == 0");

    // Mass should be reduced: m_reduced = m - IsLin^2/D
    // IsLin = Transpose(mCross)*s = 0 (mCross==0), so mass unchanged
    check(approxEq(reduced.mass, m), "PropagateInertia: mass unchanged when mCross==0");
}

static void test_SameAxisReducedChildProjection_matches_translated_reference() {
    const Vec3 s{0.0, 0.0, 1.0};
    const SpatialInertia childRigid = SpatialInertiaFromBody(
        1.7,
        Mat3{{
            {0.62, 0.05, 0.01},
            {0.05, 0.48, -0.02},
            {0.01, -0.02, 0.55},
        }},
        Vec3{0.08, -0.03, 0.11},
        Vec3{0.01, -0.02, 0.04});
    Real Dchild = 0.0;
    Vec3  childIsAng{}, childIsLin{};
    const SpatialInertia childReduced = PropagateInertia(
        childRigid, s, Dchild, childIsAng, childIsLin);

    const Vec3 rParentMinusChild{0.23, -0.19, 0.07};
    const SpatialInertia translated =
        SpatialInertiaPluckerTranslateChildToParent(childReduced, rParentMinusChild);
    const Real Dref = Dot(s, translated.Ioo * s);
    const Vec3 IsAngRef = translated.Ioo * s;
    const Vec3 IsLinRef = Transpose(translated.mCross) * s;

    const Vec3 rs = Cross(rParentMinusChild, s);
    const Vec3 childIsLinFast = childReduced.mass * rs;
    const Vec3 childIsAngFast =
        childReduced.mCross * rs - childReduced.mass * Cross(rParentMinusChild, rs);
    const Real Dfast = childReduced.mass * Dot(rs, rs);

    check(approxEq(Dfast, Dref, 1e-4),
          "Same-axis child projection: scalar D matches translated reference");
    check(approxEqVec(childIsAngFast, IsAngRef, 1e-4),
          "Same-axis child projection: IsAng matches translated reference");
    check(approxEqVec(childIsLinFast, IsLinRef, 1e-4),
          "Same-axis child projection: IsLin matches translated reference");
}

static void test_SameAxisParentReduction_matches_generic_PropagateInertia() {
    const Vec3 s{0.0, 0.0, 1.0};
    const Vec3 parentAnchor{0.12, -0.08, 0.03};
    const Vec3 childAnchor{-0.09, 0.14, -0.01};
    const SpatialInertia parentRigid = SpatialInertiaFromBody(
        1.1,
        Mat3{{
            {0.37, 0.02, 0.00},
            {0.02, 0.41, -0.01},
            {0.00, -0.01, 0.29},
        }},
        Vec3{0.19, -0.03, 0.09},
        parentAnchor);
    const SpatialInertia childRigid = SpatialInertiaFromBody(
        0.8,
        Mat3{{
            {0.21, -0.01, 0.02},
            {-0.01, 0.26, 0.03},
            {0.02, 0.03, 0.24},
        }},
        Vec3{-0.01, 0.19, 0.05},
        childAnchor);
    Real Dchild = 0.0;
    const SpatialInertia childReduced = PropagateInertia(childRigid, s, Dchild);
    const Vec3 rParentMinusChild = parentAnchor - childAnchor;
    const SpatialInertia translatedChild =
        SpatialInertiaPluckerTranslateChildToParent(childReduced, rParentMinusChild);
    const SpatialInertia IaParent = parentRigid + translatedChild;

    Real Dref = 0.0;
    Vec3  IsAngRef{}, IsLinRef{};
    const SpatialInertia reducedRef = PropagateInertia(
        IaParent, s, Dref, IsAngRef, IsLinRef);

    const Vec3 rs = Cross(rParentMinusChild, s);
    const Vec3 ownIsAng = parentRigid.Ioo * s;
    const Vec3 ownIsLin = Transpose(parentRigid.mCross) * s;
    const Vec3 childIsLinFast = childReduced.mass * rs;
    const Vec3 childIsAngFast =
        childReduced.mCross * rs - childReduced.mass * Cross(rParentMinusChild, rs);
    const Vec3 IsAngFast = ownIsAng + childIsAngFast;
    const Vec3 IsLinFast = ownIsLin + childIsLinFast;
    const Real Dfast = Dot(s, ownIsAng) + childReduced.mass * Dot(rs, rs);
    const SpatialInertia reducedFast =
        PropagateInertiaFromProjection(IaParent, IsAngFast, IsLinFast, Dfast);

    check(approxEq(Dfast, Dref, 1e-4),
          "Same-axis parent reduction: scalar D matches generic PropagateInertia");
    check(approxEqVec(IsAngFast, IsAngRef, 1e-4),
          "Same-axis parent reduction: IsAng matches generic PropagateInertia");
    check(approxEqVec(IsLinFast, IsLinRef, 1e-4),
          "Same-axis parent reduction: IsLin matches generic PropagateInertia");
    check(approxEqMat3(reducedFast.Ioo, reducedRef.Ioo, 1e-4),
          "Same-axis parent reduction: reduced Ioo matches generic PropagateInertia");
    check(approxEqMat3(reducedFast.mCross, reducedRef.mCross, 1e-4),
          "Same-axis parent reduction: reduced mCross matches generic PropagateInertia");
    check(approxEq(reducedFast.mass, reducedRef.mass, 1e-4),
          "Same-axis parent reduction: reduced mass matches generic PropagateInertia");
}

// ---------------------------------------------------------------------------
// Test 5: BuildArticulationChains on the hexapod scene
// ---------------------------------------------------------------------------

static void test_BuildArticulationChains_Hexapod() {
    World world({0.0, -9.81, 0.0});
    const HexapodSceneObjects scene = BuildHexapodScene(world);
    // Run one step to trigger BuildArticulationChains()
    world.Step(1.0 / 240.0, 1);

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
        check(chain.iaPostJoint.size() == chain.links.size(), "iaPostJoint cache size matches links");
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
        check(LengthSquared(chain.links[0].jointAnchorWorld) > 1e-12,
              "Root link jointAnchorWorld is set (first leg anchor on chassis)");
        const std::uint32_t femurJoint = chain.links[2].jointIdx;
        const std::uint32_t tibiaJoint = chain.links[3].jointIdx;
        check(world.GetServoJoint(tibiaJoint).masterAxisJointIdx == femurJoint,
              "Hexapod tibia joint masterAxisJointIdx points to the femur joint");
        for (std::size_t li = 1; li < chain.links.size(); ++li) {
            check(chain.D[li] > 1e-6, "Hexapod chain joint D is positive");
        }
    }
}

// ---------------------------------------------------------------------------
// Test 6: Two-body pendulum chain detection
// ---------------------------------------------------------------------------

static void test_BuildArticulationChains_Pendulum() {
    World world({0.0, -9.81, 0.0});

    // Static anchor body
    Body anchor;
    anchor.shape    = ShapeType::Box;
    anchor.halfExtents = {0.05, 0.05, 0.05};
    anchor.mass     = 1.0;
    anchor.isStatic = true;
    anchor.position = {0.0, 1.0, 0.0};
    const std::uint32_t anchorId = world.CreateBody(anchor);

    // Pendulum bob
    Body bob;
    bob.shape    = ShapeType::Sphere;
    bob.radius   = 0.05;
    bob.mass     = 0.5;
    bob.position = {0.0, 0.0, 0.0};
    const std::uint32_t bobId = world.CreateBody(bob);

    // One servo joint connecting anchor to bob
    const std::uint32_t jointId = world.CreateServoJoint(
        anchorId, bobId,
        /*worldAnchor=*/{0.0, 1.0, 0.0},
        /*worldAxis=*/{1.0, 0.0, 0.0},
        /*targetAngle=*/0.0,
        /*maxTorque=*/10.0,
        /*positionGain=*/10.0,
        /*dampingGain=*/1.0,
        /*integralGain=*/0.0,
        /*integralClamp=*/0.5,
        /*maxCorrectionAngle=*/0.5,
        /*angleStabilizationScale=*/1.0,
        /*maxSpeed=*/5.0,
        /*errorSmoothing=*/0.0);

    world.Step(1.0 / 240.0, 1);

    const std::vector<ArtChain>& chains = world.GetArticulationChains();
    check(chains.size() == 1u, "Pendulum: exactly one chain");

    const ArtChain& chain = chains[0];
    check(chain.links.size() == 2u, "Pendulum chain has 2 links");
    check(chain.links[0].bodyIdx == anchorId, "Pendulum root is anchor");
    check(chain.links[1].bodyIdx == bobId,    "Pendulum leaf is bob");
    check(chain.links[1].jointIdx == jointId, "Pendulum leaf's joint is correct");

    check(world.GetServoJoint(jointId).inArticulationChain,
          "Pendulum joint is marked inArticulationChain");

    check(chain.D[1] > 1e-8, "Pendulum joint D is positive (static root + dynamic bob)");
}

static void test_ArticulationConfig_roundTrip() {
    World world(Vec3{0.0, 0.0, 0.0});
    World::ArticulationConfig cfg{};
    cfg.enableVelocityPreCorrection = true;
    cfg.enableVelocityPreCorrectionKinematicsOnly = true;
    cfg.enableVelocityPreCorrectionFullForwardPass = true;
    cfg.enableChainPositionSolve = true;
    cfg.enableVelocityPreCorrectionChassisCoupling            = true;
    cfg.chassisCouplingGain                                     = 0.5;
    cfg.chassisCouplingAngularOnly                              = true;
    cfg.enableVelocityPreCorrectionChassisArticulatedTree       = false;
    cfg.enableVelocityPreCorrectionForwardCoriolis              = true;
    cfg.enableSameAxisTwoDofShortcut                            = true;
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
    check(approxEq(out.chassisCouplingGain, 0.5), "ArticulationConfig round-trip: chassisCouplingGain");
    check(out.chassisCouplingAngularOnly, "ArticulationConfig round-trip: chassisCouplingAngularOnly");
    check(!out.enableVelocityPreCorrectionChassisArticulatedTree,
          "ArticulationConfig round-trip: chassis tree flag");
    check(out.enableVelocityPreCorrectionForwardCoriolis,
          "ArticulationConfig round-trip: forward Coriolis flag");
    check(out.enableSameAxisTwoDofShortcut,
          "ArticulationConfig round-trip: same-axis 2-DOF shortcut flag");
    check(!out.includeServoPdBiasInArticulationU,
          "ArticulationConfig round-trip: includeServoPdBiasInArticulationU");
}

static void test_ABI_chain_u_nonzero_withServoBias() {
    auto makeWorld = [](bool pdInU) -> World {
        World world({0.0, -9.81, 0.0});
        Body anchor{};
        anchor.shape       = ShapeType::Box;
        anchor.halfExtents = {0.05, 0.05, 0.05};
        anchor.mass        = 1.0;
        anchor.isStatic    = true;
        anchor.position    = {0.0, 1.0, 0.0};
        const std::uint32_t anchorId = world.CreateBody(anchor);

        Body bob{};
        bob.shape       = ShapeType::Box;
        bob.halfExtents = {0.05, 0.03, 0.03};
        bob.mass        = 0.5;
        bob.position    = {0.1, 1.0, 0.0};
        const std::uint32_t bobId = world.CreateBody(bob);

        (void)world.CreateServoJoint(
            anchorId,
            bobId,
            {0.0, 1.0, 0.0},
            {0.0, 0.0, 1.0},
            /*targetAngle=*/0.0,
            10.0,
            10.0,
            1.0,
            0.0,
            0.0,
            1.0,
            5.0,
            0.0);
        const Real halfAngle = 0.55 * 0.5;
        world.GetBody(bobId).orientation =
            Normalize(Quat{std::cos(halfAngle), 0.0, 0.0, std::sin(halfAngle)});

        World::ArticulationConfig cfg{};
        cfg.enableVelocityPreCorrection               = true;
        cfg.enableVelocityPreCorrectionKinematicsOnly = false;
        cfg.includeServoPdBiasInArticulationU         = pdInU;
        world.SetArticulationConfig(cfg);
        world.Step(1.0 / 240.0, 1);
        return world;
    };

    const World withPdBias = makeWorld(true);
    const World withoutPdBias = makeWorld(false);
    const std::vector<ArtChain>& withPdChains = withPdBias.GetArticulationChains();
    const std::vector<ArtChain>& withoutPdChains = withoutPdBias.GetArticulationChains();
    check(withPdChains.size() == 1u, "ABI u test: one chain with PD bias");
    check(withoutPdChains.size() == 1u, "ABI u test: one chain without PD bias");
    check(withPdChains[0].links.size() == 2u, "ABI u test: two links with PD bias");
    check(withoutPdChains[0].links.size() == 2u, "ABI u test: two links without PD bias");
    const Real uWithPd = withPdChains[0].u[1];
    const Real uWithoutPd = withoutPdChains[0].u[1];
    check(std::abs(uWithPd - uWithoutPd) > 1e-4,
          "ABI u[1] changes when includeServoPdBiasInArticulationU toggles");
}

static void test_VelocityPreCorrection_fullForward_hexapod_finite() {
    World world(Vec3{0.0, 0.0, 0.0});
    (void)BuildHexapodScene(world);
    World::ArticulationConfig cfg{};
    cfg.enableVelocityPreCorrection                      = true;
    cfg.enableVelocityPreCorrectionKinematicsOnly        = false;
    cfg.enableVelocityPreCorrectionFullForwardPass       = true;
    world.SetArticulationConfig(cfg);
    world.Step(1.0 / 240.0, 1);
    for (std::uint32_t bi = 0; bi < world.GetBodyCount(); ++bi) {
        assertBodyKinematicsFinite(world.GetBody(bi));
    }
}

static void test_VelocityPreCorrection_fullForward_coriolis_hexapod_finite() {
    World world(Vec3{0.0, 0.0, 0.0});
    (void)BuildHexapodScene(world);
    World::ArticulationConfig cfg{};
    cfg.enableVelocityPreCorrection                      = true;
    cfg.enableVelocityPreCorrectionKinematicsOnly        = false;
    cfg.enableVelocityPreCorrectionFullForwardPass       = true;
    cfg.enableVelocityPreCorrectionForwardCoriolis       = true;
    world.SetArticulationConfig(cfg);
    world.Step(1.0 / 240.0, 1);
    for (std::uint32_t bi = 0; bi < world.GetBodyCount(); ++bi) {
        assertBodyKinematicsFinite(world.GetBody(bi));
    }
}

static void test_articulation_driveTorque_shifts_chain_u() {
    const Vec3 g{0.0, -9.81, 0.0};
    auto makeWorld = [&](Real driveTau, bool pdInU) -> World {
        World world(g);
        Body anchor{};
        anchor.shape       = ShapeType::Box;
        anchor.halfExtents = {0.05, 0.05, 0.05};
        anchor.mass        = 1.0;
        anchor.isStatic    = true;
        anchor.position    = {0.0, 1.0, 0.0};
        const std::uint32_t anchorId = world.CreateBody(anchor);

        Body bob{};
        bob.shape    = ShapeType::Sphere;
        bob.radius   = 0.05;
        bob.mass     = 0.5;
        bob.position = {0.1, 0.95, 0.0};
        const std::uint32_t bobId = world.CreateBody(bob);

        const std::uint32_t jointId = world.CreateServoJoint(
            anchorId,
            bobId,
            {0.0, 1.0, 0.0},
            {0.0, 0.0, 1.0},
            /*targetAngle=*/0.35,
            10.0,
            10.0,
            1.0,
            0.0,
            0.5,
            1.0,
            5.0,
            0.0);

        World::ArticulationConfig cfg{};
        cfg.enableVelocityPreCorrection               = true;
        cfg.enableVelocityPreCorrectionKinematicsOnly = false;
        cfg.includeServoPdBiasInArticulationU         = pdInU;
        world.SetArticulationConfig(cfg);
        world.GetServoJointMutable(jointId).articulationDriveTorque = driveTau;
        world.Step(1.0 / 240.0, 1);
        return world;
    };

    const World w0 = makeWorld(0.0, true);
    const World wTau = makeWorld(4.25, true);
    const Real u0   = w0.GetArticulationChains()[0].u[1];
    const Real uD   = wTau.GetArticulationChains()[0].u[1];
    check(approxEq(uD - u0, 4.25, 1e-3),
          "articulationDriveTorque adds literally to chain u[1] (PD bias in u on)");

    const World wOff = makeWorld(3.0, false);
    const World wOff0 = makeWorld(0.0, false);
    check(approxEq(wOff.GetArticulationChains()[0].u[1] - wOff0.GetArticulationChains()[0].u[1], 3.0, 1e-3),
          "articulationDriveTorque adds to u when PD bias excluded from u");
}

static void test_star_three_branch_root_Ia_compliance_finite() {
    World world(Vec3{0.0, 0.0, 0.0});

    Body base{};
    base.shape       = ShapeType::Box;
    base.halfExtents = {0.15, 0.05, 0.15};
    base.mass        = 3.0;
    base.position    = {0.0, 1.0, 0.0};
    const std::uint32_t baseId = world.CreateBody(base);

    auto armBody = [&](const Vec3& pos) {
        Body arm{};
        arm.shape       = ShapeType::Box;
        arm.halfExtents = {0.04, 0.04, 0.04};
        arm.mass        = 0.35;
        arm.position    = pos;
        return world.CreateBody(arm);
    };

    const std::uint32_t a0 = armBody({0.22, 1.0, 0.0});
    const std::uint32_t a1 = armBody({-0.18, 1.0, 0.2});
    const std::uint32_t a2 = armBody({-0.18, 1.0, -0.2});

    (void)world.CreateServoJoint(
        baseId, a0, {0.15, 1.0, 0.0}, {0.0, 0.0, 1.0}, 0.0, 8.0, 25.0, 1.0, 0.0, 0.5, 0.4, 1.0, 6.0, 0.5);
    (void)world.CreateServoJoint(
        baseId, a1, {-0.15, 1.0, 0.12}, {0.0, 1.0, 0.0}, 0.1, 8.0, 25.0, 1.0, 0.0, 0.5, 0.4, 1.0, 6.0, 0.5);
    (void)world.CreateServoJoint(
        baseId, a2, {-0.15, 1.0, -0.12}, {1.0, 0.0, 0.0}, -0.08, 8.0, 25.0, 1.0, 0.0, 0.5, 0.4, 1.0, 6.0, 0.5);

    World::ArticulationConfig cfg{};
    cfg.enableVelocityPreCorrection               = true;
    cfg.enableVelocityPreCorrectionKinematicsOnly = false;
    world.SetArticulationConfig(cfg);
    world.Step(1.0 / 240.0, 1);

    const std::vector<ArtChain>& chains = world.GetArticulationChains();
    check(chains.size() == 3u, "star: three serial chains from shared base");
    const SpatialVec hTest{{0.35, -0.2, 0.6}, Normalize(Vec3{0.1, -0.3, 0.5})};
    for (const ArtChain& ch : chains) {
        check(ch.links.size() == 2u, "star: each branch is base + one child");
        check(ch.links[0].bodyIdx == baseId, "star: shared root body");
        check(ch.D[1] > 1e-6, "star: positive ABI scalar at child joint");
        check(std::isfinite(ch.Ia[0].mass) && ch.Ia[0].mass > 0.5, "star: root articulated inertia mass sane");
        const Real comp = SpatialMotionCompliance(ch.Ia[0], hTest);
        check(std::isfinite(comp) && comp > 0.0, "star: root SpatialMotionCompliance(Ia[0], h) positive finite");
    }
    for (std::uint32_t bi = 0; bi < world.GetBodyCount(); ++bi) {
        assertBodyKinematicsFinite(world.GetBody(bi));
    }
}

static void test_VelocityPreCorrection_passA_pendulum() {
    World world({0.0, -9.81, 0.0});
    Body anchor{};
    anchor.shape       = ShapeType::Box;
    anchor.halfExtents = {0.05, 0.05, 0.05};
    anchor.mass        = 1.0;
    anchor.isStatic    = true;
    anchor.position    = {0.0, 1.0, 0.0};
    const std::uint32_t anchorId = world.CreateBody(anchor);

    Body bob{};
    bob.shape    = ShapeType::Sphere;
    bob.radius   = 0.05;
    bob.mass     = 0.5;
    bob.position = {0.1, 0.95, 0.0};
    const std::uint32_t bobId = world.CreateBody(bob);

    world.CreateServoJoint(
        anchorId,
        bobId,
        {0.0, 1.0, 0.0},
        {0.0, 0.0, 1.0},
        0.0,
        10.0,
        10.0,
        1.0,
        0.0,
        0.5,
        1.0,
        5.0,
        0.0);

    World::ArticulationConfig cfg{};
    cfg.enableVelocityPreCorrection                   = true;
    cfg.enableVelocityPreCorrectionKinematicsOnly     = true;
    world.SetArticulationConfig(cfg);

    world.Step(1.0 / 240.0, 1);
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
    World world({0.0, -9.81, 0.0});
    (void)BuildHexapodScene(world);
    World::ArticulationConfig cfg{};
    cfg.enableVelocityPreCorrection               = true;
    cfg.enableVelocityPreCorrectionKinematicsOnly = false;
    world.SetArticulationConfig(cfg);
    world.Step(1.0 / 240.0, 1);
    const ArtChain& ch = world.GetArticulationChains()[0];
    for (std::size_t li = 1; li < ch.links.size(); ++li) {
        const SpatialVec& p = ch.paJointSnapshot[li];
        check(std::isfinite(p.ang.x) && std::isfinite(p.ang.y) && std::isfinite(p.ang.z)
                && std::isfinite(p.lin.x) && std::isfinite(p.lin.y) && std::isfinite(p.lin.z),
              "Pass B/C: paJointSnapshot is finite at articulated joints");
    }
}

static void test_VelocityPreCorrection_hexapod_finite() {
    World world({0.0, -9.81, 0.0});
    (void)BuildHexapodScene(world);
    World::ArticulationConfig cfg{};
    cfg.enableVelocityPreCorrection               = true;
    cfg.enableVelocityPreCorrectionKinematicsOnly = false;
    world.SetArticulationConfig(cfg);
    world.Step(1.0 / 240.0, 1);
    for (std::uint32_t bi = 0; bi < world.GetBodyCount(); ++bi) {
        assertBodyKinematicsFinite(world.GetBody(bi));
    }
}

static void test_VelocityPreCorrection_legacyPassC_axisCache_hexapod_finite() {
    World world({0.0, -9.81, 0.0});
    (void)BuildHexapodScene(world);
    World::ArticulationConfig cfg{};
    cfg.enableVelocityPreCorrection               = true;
    cfg.enableVelocityPreCorrectionKinematicsOnly = false;
    cfg.enableVelocityPreCorrectionFullForwardPass = false;
    world.SetArticulationConfig(cfg);
    world.Step(1.0 / 240.0, 1);
    for (const ArtChain& ch : world.GetArticulationChains()) {
        for (std::size_t li = 1; li < ch.links.size(); ++li) {
            const Vec3 axis = ch.axisWorld[li];
            check(!(axis.x == 0.0 && axis.y == 0.0 && axis.z == 0.0),
                  "Legacy Pass C axis cache: articulated joint axis was cached");
        }
    }
    for (std::uint32_t bi = 0; bi < world.GetBodyCount(); ++bi) {
        assertBodyKinematicsFinite(world.GetBody(bi));
    }
}

static void test_PassB_sameAxisShortcut_hexapod_positive_without_velocityPreCorrection() {
    World world({0.0, -9.81, 0.0});
    (void)BuildHexapodScene(world);
    World::ArticulationConfig cfg = world.GetArticulationConfig();
    cfg.enableVelocityPreCorrection = false;
    cfg.enableSameAxisTwoDofShortcut = true;
    world.SetArticulationConfig(cfg);
    world.Step(1.0 / 240.0, 1);
    const std::vector<ArtChain>& chains = world.GetArticulationChains();
    check(chains.size() == 6u, "Pass B same-axis smoke: hexapod still builds 6 articulation chains");
    for (const ArtChain& ch : chains) {
        check(ch.links.size() == 4u, "Pass B same-axis smoke: chain has 4 links");
        check(ch.D[2] > 1e-6 && std::isfinite(ch.D[2]),
              "Pass B same-axis smoke: femur joint D is positive without velocity pre-correction");
        check(ch.D[3] > 1e-6 && std::isfinite(ch.D[3]),
              "Pass B same-axis smoke: tibia joint D is positive without velocity pre-correction");
    }
}

static void test_SameAxisTwoDofShortcut_gate_hexapod_finite() {
    auto runWithGate = [](bool enabled) {
        World world({0.0, -9.81, 0.0});
        (void)BuildHexapodScene(world);
        World::ArticulationConfig cfg = world.GetArticulationConfig();
        cfg.enableSameAxisTwoDofShortcut = enabled;
        world.SetArticulationConfig(cfg);
        world.Step(1.0 / 240.0, 1);
        for (const ArtChain& ch : world.GetArticulationChains()) {
            for (std::size_t li = 1; li < ch.links.size(); ++li) {
                check(std::isfinite(ch.D[li]) && ch.D[li] > 1e-6,
                      "Same-axis 2-DOF gate: articulated D remains finite and positive");
            }
        }
        for (std::uint32_t bi = 0; bi < world.GetBodyCount(); ++bi) {
            assertBodyKinematicsFinite(world.GetBody(bi));
        }
    };

    runWithGate(false);
    runWithGate(true);
}

static Real absHingeErrorForServoJoint(const World& world, std::uint32_t jointIdx) {
    if (jointIdx == World::kInvalidJointId) {
        return 0.0;
    }
    const ServoJoint& sj  = world.GetServoJoint(jointIdx);
    const Body&       a   = world.GetBody(sj.a);
    const Body&       b   = world.GetBody(sj.b);
    const Real       ang = ComputeServoJointAngle(a, b, sj);
    return std::abs(WrapJointAngle(ang - sj.targetAngle));
}

static void test_hinge_literal_step_monotonicity_hexapod() {
    World world(Vec3{0.0, 0.0, 0.0});
    const HexapodSceneObjects scene = BuildHexapodScene(world);
    JointSolverConfig jc = world.GetJointSolverConfig();
    jc.servoPositionSolveStride = 1u;
    world.SetJointSolverConfig(jc);
    RelaxZeroGravityHexapodServos(world, scene);
    world.Step(1.0 / 240.0, 1);

    const std::uint32_t kneeJoint = scene.legs[0].femurToTibiaJoint;
    const Real         e0k       = absHingeErrorForServoJoint(world, kneeJoint);
    const ServoJoint&   sj0       = world.GetServoJoint(kneeJoint);
    const Real         hinge0 =
        ComputeServoJointAngle(world.GetBody(sj0.a), world.GetBody(sj0.b), sj0);
    ServoJoint& sjMut = world.GetServoJointMutable(kneeJoint);
    sjMut.targetAngle = hinge0 + 0.05;
    const Real ePert = absHingeErrorForServoJoint(world, kneeJoint);
    check(ePert > e0k + 1e-5, "hinge monotonicity: small knee target offset increases that joint error");

    World::ArticulationConfig cfg{};
    cfg.enableChainPositionSolve = true;
    world.SetArticulationConfig(cfg);
    Real eTrack = ePert;
    for (int i = 0; i < 96; ++i) {
        world.Step(1.0 / 240.0, 10);
        const Real e = absHingeErrorForServoJoint(world, kneeJoint);
        if (e < eTrack) {
            eTrack = e;
        }
    }
    const Real eEnd = absHingeErrorForServoJoint(world, kneeJoint);
    check(std::isfinite(eEnd), "literal Step: knee hinge error finite");
    check(eTrack < ePert - 1e-5,
          "literal Step sequence (zero-G, stride=1, chain on): knee |hinge error| dips below post-offset");
}

static void test_chassis_coupling_mvp_hexapod_finite() {
    World world(Vec3{0.0, 0.0, 0.0});
    (void)BuildHexapodScene(world);
    World::ArticulationConfig cfg{};
    cfg.enableVelocityPreCorrection               = true;
    cfg.enableVelocityPreCorrectionKinematicsOnly = false;
    cfg.enableVelocityPreCorrectionChassisCoupling = true;
    cfg.chassisCouplingGain                         = 0.35;
    cfg.chassisCouplingAngularOnly                  = false;
    world.SetArticulationConfig(cfg);
    world.Step(1.0 / 240.0, 1);
    for (std::uint32_t bi = 0; bi < world.GetBodyCount(); ++bi) {
        assertBodyKinematicsFinite(world.GetBody(bi));
    }
}

static void test_chassis_coupling_tree_hexapod_finite() {
    World world(Vec3{0.0, 0.0, 0.0});
    (void)BuildHexapodScene(world);
    World::ArticulationConfig cfg{};
    cfg.enableVelocityPreCorrection                      = true;
    cfg.enableVelocityPreCorrectionKinematicsOnly      = false;
    cfg.enableVelocityPreCorrectionChassisArticulatedTree = true;
    cfg.chassisCouplingGain                              = 0.25;
    world.SetArticulationConfig(cfg);
    world.Step(1.0 / 240.0, 1);
    for (std::uint32_t bi = 0; bi < world.GetBodyCount(); ++bi) {
        assertBodyKinematicsFinite(world.GetBody(bi));
    }
}

static void test_chain_position_solve_hexapod_smoke() {
    World world(Vec3{0.0, 0.0, 0.0});
    const HexapodSceneObjects scene = BuildHexapodScene(world);
    RelaxZeroGravityHexapodServos(world, scene);
    world.Step(1.0 / 240.0, 1);

    World::ArticulationConfig cfg{};
    cfg.enableChainPositionSolve = true;
    world.SetArticulationConfig(cfg);
    for (int i = 0; i < 20; ++i) {
        world.Step(1.0 / 240.0, 1);
    }
    for (std::uint32_t bi = 0; bi < world.GetBodyCount(); ++bi) {
        assertBodyKinematicsFinite(world.GetBody(bi));
    }
}

static void test_short_chain_articulation_falls_back_cleanly() {
    World world({0.0, -9.81, 0.0});

    Body plane;
    plane.shape = ShapeType::Plane;
    plane.isStatic = true;
    plane.planeNormal = {0.0, 1.0, 0.0};
    plane.planeOffset = 0.0;
    world.CreateBody(plane);

    Body support{};
    support.shape = ShapeType::Box;
    support.halfExtents = {0.10, 0.10, 0.10};
    support.mass = 1.0;
    support.position = {0.0, 0.08, 0.5};
    world.CreateBody(support);

    Body base{};
    base.shape = ShapeType::Box;
    base.halfExtents = {0.18, 0.06, 0.18};
    base.mass = 1.0;
    base.isStatic = true;
    base.position = {0.0, 0.18, 0.0};
    const std::uint32_t baseId = world.CreateBody(base);

    Body arm{};
    arm.shape = ShapeType::Box;
    arm.halfExtents = {0.14, 0.05, 0.06};
    arm.mass = 0.55;
    arm.position = {0.29, 0.24, 0.0};
    const std::uint32_t armId = world.CreateBody(arm);

    world.CreateServoJoint(
        baseId,
        armId,
        {0.15, 0.24, 0.0},
        {0.0, 0.0, 1.0},
        0.45,
        2.0,
        1.0,
        0.4,
        0.0,
        0.5,
        0.2,
        0.2);

    World::ArticulationConfig cfg{};
    cfg.enableVelocityPreCorrection = true;
    cfg.enableVelocityPreCorrectionKinematicsOnly = false;
    cfg.enableChainPositionSolve = true;
    world.SetArticulationConfig(cfg);

    for (int step = 0; step < 120; ++step) {
        world.Step(1.0 / 120.0, 24);
    }

    const auto& chains = world.GetArticulationChains();
    check(chains.size() == 1u, "short-chain fallback: one articulation chain");
    check(chains[0].links.size() == 2u, "short-chain fallback: chain has two links");
    assertBodyKinematicsFinite(world.GetBody(baseId));
    assertBodyKinematicsFinite(world.GetBody(armId));
#if MINPHYS3D_SOLVER_TELEMETRY_ENABLED
    const auto& telemetry = world.GetSolverTelemetry();
    check(telemetry.articulationChains.size() == 1u,
          "short-chain fallback: telemetry captured one articulation sample");
    const auto& sample = telemetry.articulationChains[0];
    check(sample.passCSkippedShortChain,
          "short-chain fallback: Pass C is skipped for chains shorter than four links");
    check(sample.chainPositionSkippedShortChain,
          "short-chain fallback: chain position solve falls back to legacy hinge snap");
#endif
}

static void test_contact_heavy_hexapod_disables_passC() {
    World world({0.0, -9.81, 0.0});
    const HexapodSceneObjects scene = BuildHexapodScene(world);
    RelaxBuiltInHexapodServos(world, scene);

    World::ArticulationConfig cfg{};
    cfg.enableVelocityPreCorrection = true;
    cfg.enableVelocityPreCorrectionKinematicsOnly = false;
    world.SetArticulationConfig(cfg);

    for (int step = 0; step < 120; ++step) {
        world.Step(1.0 / 240.0, 24);
    }

    for (std::uint32_t bi = 0; bi < world.GetBodyCount(); ++bi) {
        assertBodyKinematicsFinite(world.GetBody(bi));
    }
#if MINPHYS3D_SOLVER_TELEMETRY_ENABLED
    const auto& telemetry = world.GetSolverTelemetry();
    bool sawContactSkip = false;
    for (const auto& sample : telemetry.articulationChains) {
        sawContactSkip = sawContactSkip || sample.passCSkippedContacts;
    }
    check(sawContactSkip, "contact-heavy hexapod: at least one articulated chain skips Pass C on contacts");
#endif
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

    std::printf("test_SameAxisReducedChildProjection_matches_translated_reference...\n");
    test_SameAxisReducedChildProjection_matches_translated_reference();

    std::printf("test_SameAxisParentReduction_matches_generic_PropagateInertia...\n");
    test_SameAxisParentReduction_matches_generic_PropagateInertia();

    std::printf("test_SpatialInertiaPlucker_kineticEnergyInvariant...\n");
    test_SpatialInertiaPlucker_kineticEnergyInvariant();

    std::printf("test_SpatialForceTranslateByOffset...\n");
    test_SpatialForceTranslateByOffset();

    std::printf("test_SpatialInertiaSolveMotion_diagonal...\n");
    test_SpatialInertiaSolveMotion_diagonal();

    std::printf("test_SpatialInertiaSolveMotion_nearSingular_returns_safe_result...\n");
    test_SpatialInertiaSolveMotion_nearSingular_returns_safe_result();

    std::printf("test_SpatialMotionCompliance_singular_is_safe...\n");
    test_SpatialMotionCompliance_singular_is_safe();

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

    std::printf("test_VelocityPreCorrection_legacyPassC_axisCache_hexapod_finite...\n");
    test_VelocityPreCorrection_legacyPassC_axisCache_hexapod_finite();

    std::printf("test_PassB_sameAxisShortcut_hexapod_positive_without_velocityPreCorrection...\n");
    test_PassB_sameAxisShortcut_hexapod_positive_without_velocityPreCorrection();

    std::printf("test_SameAxisTwoDofShortcut_gate_hexapod_finite...\n");
    test_SameAxisTwoDofShortcut_gate_hexapod_finite();

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

    std::printf("test_short_chain_articulation_falls_back_cleanly...\n");
    test_short_chain_articulation_falls_back_cleanly();

    std::printf("test_contact_heavy_hexapod_disables_passC...\n");
    test_contact_heavy_hexapod_disables_passC();

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
