#pragma once

#include <cstdint>
#include <limits>
#include <vector>

#include "minphys3d/math/spatial.hpp"
#include "minphys3d/math/vec3.hpp"

namespace minphys3d {

// ---------------------------------------------------------------------------
// ArtLink — one link in an articulated chain
// ---------------------------------------------------------------------------
// Links are stored root-to-leaf within an ArtChain.
// The root link has jointIdx == kNoJoint and parent == -1.
// All other links have a valid jointIdx (index into World::servoJoints_)
// and parent >= 0 pointing to the parent link index within this chain.

struct ArtLink {
    std::uint32_t bodyIdx  = 0;
    // Index of the ServoJoint connecting this link to its parent.
    // kNoJoint for the root link.
    static constexpr std::uint32_t kNoJoint = std::numeric_limits<std::uint32_t>::max();
    std::uint32_t jointIdx = kNoJoint;
    // Index of parent link within ArtChain::links; -1 for root.
    int parent = -1;
    // Joint axis in body-A (parent) local frame — copied from ServoJoint::localAxisA.
    Vec3 jointAxisLocal{};
    // World-space joint anchor position — copied from the joint anchor at chain-build time.
    // Reused each substep as the reference point for SpatialInertiaFromBody.
    Vec3 jointAnchorWorld{};
};

// ---------------------------------------------------------------------------
// ArtChain — a serial kinematic chain (path from root to leaf)
// ---------------------------------------------------------------------------
// The chain represents one path through the robot's kinematic tree, e.g.
// [chassis → coxa → femur → tibia] for one hexapod leg.
//
// The root link (index 0) is treated as a "floating base" with known velocity
// from Body::velocity / Body::angularVelocity.  Its own jointIdx is kNoJoint.
//
// Per-substep caches (Ia, Pa, D, u, vel) are resized and recomputed by
// PrepareArticulatedInertias() each substep.  During Phase 1a they are
// allocated but not yet filled — the caches are zero-initialised by default.

struct ArtChain {
    // Link topology — root at index 0, leaves at the end.
    std::vector<ArtLink> links;

    // ---- per-substep caches (filled by PrepareArticulatedInertias) ----
    // Articulated body inertia per link about that link's joint anchor point.
    // Ia[0] is unused (root); Ia[i] for i>0 is the ABI of the subtree rooted at links[i].
    std::vector<SpatialInertia> Ia;
    // Articulated bias force per link (Coriolis / centrifugal).
    std::vector<SpatialVec> Pa;
    // Effective scalar mass at each joint: D[i] = s_i^T * Ia[i] * s_i.
    // D[0] is unused (root has no parent joint).
    std::vector<float> D;
    // Generalised force at each joint (scalar, revolute DOF).  Filled in
    // `PrepareArticulatedInertias`: u = s^T*Pa + (optional) D*servoBias from
    // `ArticulationConfig::includeServoPdBiasInArticulationU` + `ServoJoint::articulationDriveTorque`.
    // u[0] is unused.
    std::vector<float> u;
    // Spatial velocity of each link (computed root→tip in PrepareArticulatedInertias).
    std::vector<SpatialVec> vel;
    // Rigid-body spatial inertia at each link anchor before ABI merge (Phase 2 Pa / diagnostics).
    std::vector<SpatialInertia> rigidSpatialI;
    // Pa at each link immediately before that link's joint is processed in the tip→root ABI loop
    // (used for Phase 2 joint-acceleration projection).
    std::vector<SpatialVec> paJointSnapshot;
    // Articulated composite inertia at link i's joint anchor **before** factoring
    // that joint's DOF (used by optional full forward acceleration pass).
    std::vector<SpatialInertia> iaPreJoint;
    // Articulated composite inertia at link i's joint anchor **after** factoring that link's
    // own revolute DOF in the tip→root ABI pass. Used by the same-axis 2-DOF specialization
    // to reuse the child reduced subtree at its own joint anchor.
    std::vector<SpatialInertia> iaPostJoint;
    // Spatial accelerations at each link joint anchor (optional forward pass).
    std::vector<SpatialVec> spatialAcc;
    // Per-substep world-space joint axis cache (filled by Pass A of PrepareArticulatedInertias
    // when enableVelocityPreCorrection is true). axisWorld[i] is the normalized axis for the
    // joint connecting link i to its parent (index 0 unused; root has no parent joint).
    // Zero vector = axis was degenerate (TryNormalize failed) — caller must skip that link.
    // When velocityPreCorr is false, Pass A does not run; axisWorld stays all-zero and Pass B
    // falls back to computing the axis directly.
    std::vector<Vec3> axisWorld;

    // When `enableVelocityPreCorrectionChassisArticulatedTree` is on: articulated leg subtree
    // (links 1..N−1) at link-1 joint anchor, captured immediately before merging link 1 into the
    // chassis in the tip→root ABI loop (N >= 4).
    SpatialInertia legSubtreeIaAtCoxaAnchor{};
    SpatialVec    legSubtreePaAtCoxaAnchor{};

    // Convenience: number of links (chain length including root).
    std::size_t size() const { return links.size(); }

    // Resize / zero-initialise all per-substep caches to match links.size().
    void resizeCaches() {
        const std::size_t n = links.size();
        Ia.assign(n, SpatialInertia{});
        Pa.assign(n, SpatialVec{});
        D.assign(n, 0.0);
        u.assign(n, 0.0);
        vel.assign(n, SpatialVec{});
        rigidSpatialI.assign(n, SpatialInertia{});
        paJointSnapshot.assign(n, SpatialVec{});
        iaPreJoint.assign(n, SpatialInertia{});
        iaPostJoint.assign(n, SpatialInertia{});
        spatialAcc.assign(n, SpatialVec{});
        axisWorld.assign(n, Vec3{});
        legSubtreeIaAtCoxaAnchor = SpatialInertia{};
        legSubtreePaAtCoxaAnchor = SpatialVec{};
    }
};

} // namespace minphys3d
