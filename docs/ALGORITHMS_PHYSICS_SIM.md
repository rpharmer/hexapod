# `hexapod-physics-sim` Algorithm Reference

This document describes the core algorithms used by `minphys3d` in `hexapod-physics-sim`.

## 1) Simulation step pipeline

Primary entrypoint: `World::Step(float dt, int solverIterations)` in `hexapod-physics-sim/src/core/world.cpp`.

Per substep, the high-level order is:

1. Integrate forces
2. Update broadphase proxies
3. Generate contacts
4. Build manifolds (including persistent warm-start state)
5. Build islands and articulation chains
6. Warm-start contacts and joints
7. Prepare joint/contact solver caches
8. Solve islands (PGS iterations + optional relaxation pass)
9. Apply split stabilization
10. Resolve continuous collision (`TOI`) events
11. Integrate orientation
12. Solve articulation/joint positions
13. Positional correction
14. Sleep update
15. Clear accumulators and clamp body velocities

This ordering is intentionally split between velocity-level solving and position-level correction for stability.

## 2) Broadphase

Primary implementation: `hexapod-physics-sim/src/core/world_broadphase.cpp`.

Algorithms and policies:

- Dynamic AABB tree over fat proxies (`BroadphaseProxy`).
- Proxy update/reinsert path with moved-proxy tracking.
- Pair generation via tree traversal against moved/all proxies.
- Optional pair-cache reuse for quasi-static frames.
- Rebuild heuristics based on tree quality and moved-proxy ratio thresholds.

Important behavior:

- Broadphase detects potential overlap only.
- Pair eligibility filtering (such as mask/group constraints) is applied at leaf-pair checks and pair-cache revalidation, not internal-node pruning.

## 3) Contact generation and narrowphase

Primary implementation: `hexapod-physics-sim/src/core/world_collision.cpp`.

The contact pipeline combines specialized and generic paths:

- Specialized shape-pair routines for common pairs.
- Convex fallback path using GJK + EPA.
- Compound shape expansion and child-shape contact generation.
- Terrain contact generation from sampled heightfield cells.

Supporting algorithms:

- GJK: `hexapod-physics-sim/src/narrowphase/gjk.cpp`
- EPA: `hexapod-physics-sim/src/narrowphase/epa.cpp`
- Cylinder/half-cylinder specific contact handling: `hexapod-physics-sim/src/core/cylinder_contacts.cpp`

### Terrain contacts

Terrain contact generation samples collision heights from the terrain attachment, computes local normals, scores candidates, then selects a bounded set of contacts per body region before manifolding.

## 4) Manifold construction and persistence

Primary manifold build: `World::BuildManifolds()` in `world_collision.cpp`.

Core mechanics:

- Contacts are grouped into manifolds by key.
- Prior manifold/contact state is matched to support warm-start impulses.
- Persistent point keys are tracked to avoid duplicate reuse.
- Manifold block caches are refreshed, and block-solve pair selection is recorded.

This persistence layer is the bridge between frame-to-frame solver convergence and deterministic ordering.

## 5) Constraint solving and island ordering

Primary solver implementation: `hexapod-physics-sim/src/core/world_solver.cpp`.
Island ordering support: `hexapod-physics-sim/src/solver/island_ordering.cpp`.
Block kernels:

- `hexapod-physics-sim/src/solver/block2_solver.cpp`
- `hexapod-physics-sim/src/solver/block4_solver.cpp`

Key algorithmic components:

- Per-substep cache prep (`PrepareContactSolves`, joint prep) to avoid repeated Jacobian/effective-mass work inside each iteration.
- Island-based iterative solve (`SolveIslands`) with configurable ordering.
- Scalar, 2-contact block, and 4-contact block normal solve routes with numerical gating and fallback.
- Friction solve integrated with manifold-level budget handling.
- Joint and servo constraints solved in iterative passes, including hinge/servo-specific logic and early-out guards.

## 6) Articulation and joint position correction

Primary implementation: `world_solver.cpp`.

Articulation features:

- Build articulation chains from servo-joint topology.
- Optional articulated inertia preparation and propagation.
- Position-level passes (`SolveArticulationPositions`, `SolveJointPositions`) run at configurable stride.

The engine keeps velocity-level and position-level correction distinct, then composes both per substep.

## 7) Continuous collision (TOI)

Primary implementation:

- TOI orchestration in `world_solver.cpp` (`ResolveTOIPipeline`)
- Sweep and impact helpers in `world_private_methods.cpp` (`FindEarliestTOI`, `SweepSphere*`, `ResolveTOIImpact`)

Scope:

- Finds earliest qualifying impact over supported sweep pairs.
- Advances and resolves impact impulses/correction for high-speed interactions.
- Complements, not replaces, discrete contact generation.

## 8) Sleeping/waking

Primary implementation: `hexapod-physics-sim/src/core/sleep_system.cpp`.

Algorithm:

- Build body-island connectivity.
- Check per-island near-still conditions using linear/angular thresholds.
- Increment per-body sleep counters while still.
- Enter sleep when frame threshold is reached; reset counters on activity.

Sleep is island-aware, so wake/sleep behavior follows connectivity rather than isolated body checks.

## 9) Main tuning surfaces

Configuration interfaces:

- `ContactSolverConfig` (contact/block/friction/relaxation controls)
- `JointSolverConfig` (joint/servo and articulated options)
- `BroadphaseConfig` (tree/proxy/pair-cache/rebuild behavior)

These are exposed through world config APIs in `include/minphys3d/core/world.hpp`.