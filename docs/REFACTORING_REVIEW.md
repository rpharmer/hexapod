# Refactoring Review

This document updates refactoring status based on the current codebase.

## Scope reviewed

- `hexapod-common/` framing/protocol utilities
- `hexapod-server/` control runtime modules
- `hexapod-client/` firmware boot/dispatch/command handlers

## Refactoring progress snapshot

### Completed / in good shape

1. **Firmware monolith decomposition is complete at file level**
   - Boot and lifecycle behavior live in `firmware_boot.cpp`.
   - Command routing is centralized in `command_dispatch.cpp`.
   - Command families are separated into `power_commands.cpp`, `sensing_commands.cpp`, and `motion_commands.cpp`.

2. **Server control responsibilities are partially separated**
   - `RobotControl` runs loops and owns orchestration.
   - `ControlPipeline` now encapsulates gait/body/IK sequencing for each control step.
   - `StatusReporter` isolates periodic diagnostic formatting/log output.

3. **Runtime state representation improved on firmware side**
   - Firmware uses explicit `HexapodState` transitions (`BOOT` → `WAITING_FOR_HOST` → `ACTIVE` → `STOPPING` → `OFF`).

### Still high-value refactoring targets

1. **Replace placeholder controllers with actual policy**
   - `BodyController` still returns default `LegTargets` and ignores inputs.
   - `GaitScheduler` still uses fallback speed magnitude rather than command-derived speed.

2. **Further reduce `RobotControl` coupling**
   - The class still owns thread lifecycle, buffers, and direct loop implementations in one unit.
   - A next split could isolate loop runners or scheduling adapters from lifecycle management.

3. **Strengthen contract boundaries and units**
   - More explicit unit-safe types (angles, angular rates, timestamps) would reduce accidental mixing across estimator/gait/IK boundaries.

4. **Externalize geometry and runtime tunables**
   - Static constants are still embedded in code paths that would benefit from validated config-backed parameters.

## Recommended refactoring sequence

1. Implement body controller + gait speed path (highest behavior impact).
2. Extract loop execution concerns out of `RobotControl` into focused loop-runner components.
3. Introduce stronger type aliases/wrappers for units and timing.
4. Move geometry/tuning constants to validated config with defaults and version checks.
5. Add regression tests around protocol errors and safety transitions to preserve behavior during refactors.
