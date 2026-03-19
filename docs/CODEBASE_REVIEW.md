# Codebase Review: Hexapod

## Review scope (2026-03-19)

This pass reviewed:

- `hexapod-server/` control/runtime orchestration and safety flow.
- `hexapod-client/` firmware lifecycle + command dispatch.
- `hexapod-common/` framing/protocol shared layer.
- Documentation consistency against current implementation.

## Build and verification checks run

Commands executed during this review:

1. `cmake -S hexapod-server -B hexapod-server/build`
2. `cmake --build hexapod-server/build -j4`
3. `cmake -S hexapod-client -B hexapod-client/build -DHEXAPOD_CLIENT_SETUP_SDKS_ONLY=ON`
4. `cmake --build hexapod-client/build --target setup-sdks -j4`
5. `cmake -S hexapod-client -B hexapod-client/build -DHEXAPOD_CLIENT_SETUP_SDKS_ONLY=OFF`
6. `cmake --build hexapod-client/build --target hexapod-client -j4`

Result: all six commands completed successfully in the current environment.

## Current-state findings

### 1) Build posture and module boundaries

- Server and firmware both configure and compile cleanly.
- Shared framing code integrates cleanly into both binaries.
- The current split between runtime orchestration (`RobotControl`/`RobotRuntime`) and per-step control pipeline (`ControlPipeline`) is clear and maintainable.

### 2) Firmware command-loop behavior

- Firmware startup/dispatch structure is coherent and easier to reason about than a single monolithic loop.
- Host pairing gate is explicit: firmware only transitions to `ACTIVE` after `HELLO`.
- Behavior for repeated `HELLO`, unsupported commands, and `KILL` is explicit and deterministic.

### 3) Control behavior completeness gaps

Two known placeholders still block “real gaited walking” quality:

- `BodyController::update()` currently returns default `LegTargets` and does not yet implement stance/swing foot-placement policy.
- `GaitScheduler::update()` still derives cadence from fallback constant `kFallbackSpeedMag` rather than command-derived intent magnitude.

### 4) Safety posture

- Safety checks for tip-over and stale command input are wired and prioritized.
- `SAFE_IDLE` motion inhibition is in place.
- Remaining risk: safety/protocol behavior is primarily validated by manual/runtime testing; there is no automated regression suite covering malformed packets, boundary payload lengths, or timeout-recovery scenarios.

### 5) Documentation and maintainability

- Existing docs are generally aligned with current architecture.
- The most important remaining doc risk is drift as control behavior evolves from placeholder policy to real gait policy; this should be updated in lockstep with controller implementation PRs.

## Recommended near-term actions

1. Implement real body target generation in `BodyController` and command-driven cadence in `GaitScheduler`.
2. Add focused regression tests for:
   - framing decode/CRC/length edge cases,
   - command dispatch error paths,
   - safety transitions (`COMMAND_TIMEOUT`, `TIP_OVER`, `SAFE_IDLE`).
3. Add CI checks to preserve today’s build health for:
   - server configure/build,
   - firmware SDK setup target,
   - firmware full firmware build target.
4. Keep docs updated with each control-policy iteration to avoid architecture/behavior drift.
