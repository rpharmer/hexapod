# Codebase Review (March 19, 2026)

## Scope

This review covers:

- `hexapod-server` host runtime and control loop orchestration.
- `hexapod-client` firmware command handling and transport.
- `hexapod-common` protocol/framing contracts shared between host and firmware.

## Static analysis + build checks run

1. Built `hexapod-server` with default warning profile (`-Wall -Wextra -Werror`) successfully.
2. Built `hexapod-client` setup target and full firmware target successfully.
3. Attempted deeper GCC analyzer build (`-fanalyzer`), but build failed due analyzer diagnostics in `libstdc++` internals while compiling `logger.cpp` with global `-Werror` enabled.
4. `cppcheck` is not installed in this environment.

## Architecture observations

### What is strong

- **Clear protocol single-source-of-truth** in `hexapod-common` with explicit payload size constants and stable command IDs.
- **Reasonable subsystem split** on server: hardware bridge, estimator, gait/body/IK pipeline, safety supervisor, diagnostics.
- **Defensive framing layer** (`tryDecodePacket`) that actively re-syncs on malformed data.
- **Handshake/liveness safety hooks** on firmware side (host timeout drops to safe state).

### Gaps / risk areas

1. **Data races between control loops are likely under current threading model.**
   - `RobotControl::start()` launches bus/estimator/control/safety/diagnostics in independent threads.
   - `ControlPipeline::runStep()` receives `SafetyState` as input, but `safetyStep()` may update it concurrently.
   - There is no explicit cycle barrier/phase ordering between read-estimate-safety-control-write.

2. **Estimator is currently a pass-through stub.**
   - `SimpleEstimator::update` only copies raw leg states/contacts and timestamp.
   - `body_twist_state` is never estimated, yet safety checks use orientation (`TIP_OVER`) thresholds from estimated state.

3. **Body controller is a no-op placeholder.**
   - `BodyController::update` returns default `LegTargets` and explicitly ignores inputs.
   - This means IK receives effectively zero/default foot targets regardless of gait phase.

4. **Gait scheduler command coupling is incomplete.**
   - Step frequency currently uses fallback speed constant instead of intent command magnitude (existing TODO).

5. **Potential protocol bug: single-joint command payload validation mismatch.**
   - Firmware `handleSetAngleCommand` expects payload size `3`, but payload contains `uint8_t servo + float angle` (5 bytes).
   - Host `set_target_angle()` encodes exactly that 5-byte shape.
   - Result: command likely always NACKs as `INVALID_PAYLOAD_LENGTH`.

6. **Transport framing allows payload-length overflow silently.**
   - `encodePacket` stores frame length in a `uint8_t` and currently comments out max-payload guard.
   - If payload > 252 bytes, length wraps and frame becomes malformed/non-interoperable.

7. **Server startup main loop is mostly hard-coded behavior.**
   - `main()` repeatedly writes a fixed WALK/TRIPOD intent with fixed body height.
   - Useful for bring-up, but not yet shaped as commandable application boundary (CLI/RPC/gamepad/network).

## Technical debt hotspots

- **`hardware_bridge.cpp` is a large multipurpose class** (protocol packing, sequencing, handshake, retries/ACK handling, hardware-state decode, command APIs).
- **Command dispatch in firmware is switch-heavy and duplicated by category**, making protocol additions verbose and error-prone.
- **No tests directory / harness detected** for protocol compatibility, packet round-trip, or safety state transitions.

## Recommended quality gates (near-term)

1. Add host/firmware protocol conformance tests for every command payload shape.
2. Add packet fuzz tests around `tryDecodePacket` error recovery and CRC/ETX corruption.
3. Split strict `-Werror` CI from exploratory static-analysis CI (allow analyzer findings without blocking build on third-party/STL noise).
4. Add deterministic simulation test for control/safety loop sequencing.

