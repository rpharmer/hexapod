# Codebase Review: Hexapod

## Review scope

This review pass covered:

- `hexapod-server/` (host control loops, safety, hardware bridge)
- `hexapod-client/` (Servo 2040 firmware boot, dispatch, command handlers)
- `hexapod-common/` (framing/protocol utilities)
- top-level and component README files for documentation alignment

## Build and verification checks run

All commands below were executed in this review window:

1. `cmake -S hexapod-server -B hexapod-server/build`
2. `cmake --build hexapod-server/build -j4`
3. `cmake -S hexapod-client -B hexapod-client/build -DHEXAPOD_CLIENT_SETUP_SDKS_ONLY=ON`
4. `cmake --build hexapod-client/build --target setup-sdks -j4`
5. `cmake -S hexapod-client -B hexapod-client/build -DHEXAPOD_CLIENT_SETUP_SDKS_ONLY=OFF`
6. `cmake --build hexapod-client/build --target hexapod-client -j4`

## Current-state findings

### 1) Build posture

- Server and firmware both build successfully in the current environment.
- Shared framing/protocol code in `hexapod-common/` integrates cleanly with both binaries.

### 2) Architecture and code organization

- Firmware-side decomposition is now clear and maintainable:
  - `firmware_boot.cpp` handles bring-up/teardown,
  - `command_dispatch.cpp` routes commands,
  - command families are split into `power_commands.cpp`, `sensing_commands.cpp`, and `motion_commands.cpp`.
- Server-side loop and control orchestration separation is improved:
  - `RobotControl` owns runtime loops,
  - `ControlPipeline` encapsulates per-step controller chaining,
  - `StatusReporter` handles periodic status output.

### 3) Behavior completeness gaps

- `BodyController::update()` is still a placeholder (returns default `LegTargets`; no stance/swing target generation).
- `GaitScheduler::update()` still computes cadence from fallback speed (`kFallbackSpeedMag`) rather than command-derived magnitude.
- Current control behavior therefore exercises infrastructure flow but not finalized locomotion policy.

### 4) Safety and protocol handling

- Safety fault evaluation is wired (`TIP_OVER`, `COMMAND_TIMEOUT`, `SAFE_IDLE` inhibit path), with priority handling in `SafetySupervisor`.
- Firmware command-loop gating around handshake is robust at a baseline level (pre-active command rejection, repeated `HELLO` handling, heartbeat and kill handling).
- Remaining risk: no automated regression suite for malformed framing, CRC mismatch, payload-length edge cases, and heartbeat-timeout recovery.

### 5) Documentation alignment

- Planning docs exist and are useful, but some README sections were stale relative to the current split firmware/server architecture.
- This review refresh includes README updates so project layout and behavior descriptions match current source organization.

## Recommended near-term actions

1. Implement non-placeholder body controller outputs and command-driven gait speed.
2. Add protocol/safety regression tests around malformed frames, invalid lengths, unsupported commands, and timeout paths.
3. Add lightweight CI coverage for:
   - server configure/build,
   - firmware SDK setup target,
   - firmware full compile target.
4. Keep documentation synchronized with architectural changes (especially when file ownership and loop responsibilities move).
