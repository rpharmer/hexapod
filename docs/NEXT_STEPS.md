# Next Steps (March 23, 2026)

## Phase 1 (1 week) — Core maintainability

1. Refactor `SimpleHardwareBridge` request/decode duplication.
2. Split `RobotRuntime::controlStep` into pure decision helpers.
3. Add/adjust focused unit tests for freshness gate edge cases.

## Phase 2 (1 week) — Firmware dispatch cleanup

1. Replace forwarding wrappers in `command_dispatch.cpp`.
2. Ensure payload policy behavior remains unchanged.
3. Extend firmware host-test coverage for invalid payload lengths.

## Phase 3 (1 week) — Protocol metadata unification

1. Create shared command metadata source in `hexapod-common`.
2. Migrate host command-name telemetry to shared metadata.
3. Add a consistency test ensuring command IDs and route entries remain aligned.

## Ongoing

- Run scenario sweep in sim mode before hardware validation.
- Track command failure and freshness-stale counters over longer test runs.
- Keep documentation synchronized with implementation changes.

## Phase 4 (design spike) — Flat-surface servo calibration probe

Goal: when the robot is standing on a known flat surface, run a guided routine that intentionally touches each foot to several XY spots and uses contact timing/geometry residuals to validate (and optionally refine) `ServoCalibration` offsets/signs.

1. Add a host-side calibration mode runner (safe, low speed, one leg at a time) that executes a fixed probe pattern per leg while keeping a stable support polygon with the other five legs.
2. Capture tuples per touch: commanded body-frame foot target, measured joint angles, contact bit, and estimated body pose from the estimator.
3. Fit per-joint calibration correction terms by minimizing ground-plane height error at contact events (`z_foot ~= 0` on the known plane), with robust rejection for missed/slipped contacts.
4. Keep updates bounded and operator-reviewed: emit a proposed calibration delta report first, then require explicit apply/commit step.
5. Add safety interlocks: abort on contact-count anomalies, over-current, timeout, or IK infeasible target; automatically return to neutral stance on abort.
6. Add simulation + hardware acceptance checks:
   - sim: inject known offset/sign errors and verify routine converges toward truth within tolerance;
   - hardware: repeatability check across two runs and two surface locations.

Deliverables:
- design note for command/state machine + telemetry schema;
- scenario(s) that exercise nominal probe and abort paths;
- test coverage for estimation/math utilities used by the calibration fitter.
