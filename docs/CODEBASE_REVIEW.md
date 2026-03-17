# Codebase Review: Hexapod

## Review scope

This review was refreshed against the current repository state for:

- `hexapod-server/` (host runtime and control orchestration)
- `hexapod-client/` (Servo 2040 firmware and serial command handling)
- `hexapod-common/` (framing/protocol helpers)
- top-level docs (`README.md`, `docs/`)

## Build verification (current run)

The following commands were executed successfully during this review:

1. Server configure + build
   - `cmake -S hexapod-server -B hexapod-server/build`
   - `cmake --build hexapod-server/build -j4`
2. Client SDK setup/prebuild
   - `cmake -S hexapod-client -B hexapod-client/build -DHEXAPOD_CLIENT_SETUP_SDKS_ONLY=ON`
   - `cmake --build hexapod-client/build --target setup-sdks -j4`
3. Client full firmware build
   - `cmake -S hexapod-client -B hexapod-client/build -DHEXAPOD_CLIENT_SETUP_SDKS_ONLY=OFF`
   - `cmake --build hexapod-client/build --target hexapod-client -j4`

## Findings

### 1) Build health: good

- Both server and firmware builds are green in this environment.
- Shared framing code compiles into both targets without extra patching.

### 2) Runtime architecture: stable skeleton, key motion logic still placeholder

- `RobotControl` remains the central orchestrator for bus, estimator, control, safety, and diagnostics loops.
- `BodyController::update()` still returns default outputs with TODO comments and intentionally unused inputs.
- `GaitScheduler::update()` still uses `kFallbackSpeedMag` instead of command-derived speed.

### 3) Firmware command loop and shutdown semantics: improved but still basic

- Firmware now exits the command loop on `KILL`, and post-loop cleanup is reachable.
- Cleanup behavior now disables servos and powers down outputs before exit, which aligns with safety intent.
- There is still no broader lifecycle/state machine around startup/armed/stopped modes.

### 4) Configuration/protocol handling: generally consistent

- Server-side config parsing and validation are robust for calibration inputs and key/cardinality checks.
- Framing and transport buffer bounds remain aligned.

### 5) Documentation status: improved, minor consolidation opportunities remain

- The top-level README now points to existing protocol/documentation locations.
- Build instructions are usable as written, though the client section could be further condensed to reduce repetition.

## Recommended near-term actions

1. Replace placeholder control logic (`BodyController`, gait speed input path).
2. Add command/heartbeat error-path tests (CRC, malformed payload, timeout, unsupported command).
3. Introduce explicit runtime states for firmware bring-up/armed/stop behavior.
4. Keep server/client builds as baseline CI checks.
