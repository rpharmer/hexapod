# Featherstone remaining phases 1–4 (implementation tracker)

**Location:** [`.cursor/plans/featherstone_phases_1_4.plan.md`](featherstone_phases_1_4.plan.md) (workspace root).

**Status:** Phases 1–4 implemented and verified (`test_articulation`, `test_servo_regression_matrix`, `regression_scene_suite`).

## Phase 1 — `N >= 2` hinge + contact

- [x] Relax `artChainIndexForLeafBody_` and `invDenomHinge` ABI loop in `PrepareArticulatedInertias` (`world_solver.cpp`).
- [x] Update `ArticulationConfig` docs (`world.hpp`).
- [x] Re-baseline `test_servo_regression_matrix.cpp`; re-run `regression_scene_suite`.
- [x] Short-chain coverage: star test (three `N == 2` branches) + existing pendulum / hex paths.

## Phase 2 — Forward Coriolis / bias acceleration (sub-flag)

- [x] `ArticulationConfig::enableVelocityPreCorrectionForwardCoriolis` (requires full forward pass).
- [x] Centripetal-style bias on linear spatial accel across link offset (Pass A parent ω).
- [x] Hex finite test: `test_VelocityPreCorrection_fullForward_coriolis_hexapod_finite` (`test_articulation.cpp`).

## Phase 3 — Literal drive torque in `u`

- [x] `ServoJoint::articulationDriveTorque` (default 0).
- [x] `ArticulationConfig::includeServoPdBiasInArticulationU` (default true).
- [x] Combined in `PrepareArticulatedInertias`; `test_articulation_driveTorque_shifts_chain_u`.

## Phase 4 — Star topology sanity

- [x] One base + three short branches: `test_star_three_branch_root_Ia_compliance_finite`.
- [x] Asserts: finite `Step`, `D[1] > 0`, root `Ia[0].mass` sane, positive finite `SpatialMotionCompliance(Ia[0], h)` per chain.

---

*Implementation follows this checklist; edit this file if scope shifts.*
