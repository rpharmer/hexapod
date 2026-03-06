# Codebase Review: Hexapod

## Scope reviewed

- `README.md`
- `hexapod-common/include/hexapod-common.hpp`
- `hexapod-common/include/framing.hpp`
- `hexapod-common/framing.cpp`
- `hexapod-server/include/*.hpp`
- `hexapod-server/src/*.cpp`
- `hexapod-client/hexapod-client.cpp`
- `hexapod-client/serialCommsClient.*`

## Executive summary

The repository has a good architecture split (shared protocol, Linux host control stack, embedded firmware), and the framed command protocol is significantly more mature than the older startup-only flow described in parts of the docs. The firmware command handlers now include payload-size and index validation for the main command paths.

Main remaining risks are less about basic command parsing and more about **runtime robustness** and **operational correctness**:

1. host-side runtime is still not exercising the full pipeline end-to-end,
2. build/reproducibility is currently fragile in clean environments,
3. a few implementation details can cause subtle protocol/logic bugs under stress.

## What is working well

1. **Clear module boundaries**
   - `hexapod-common` owns protocol + framing primitives.
   - `hexapod-client` cleanly maps packets to hardware-facing handlers.
   - `hexapod-server` is moving toward a layered control stack (`hardware_bridge`, `estimator`, `robot_control`).

2. **Protocol hardening has improved**
   - Handlers such as `handleHandshake`, `handleSetAngleCommand`, `handleCalibCommand`, and `handleSetJointTargetsCommand` enforce exact payload lengths.
   - Servo/sensor index checks return explicit NACK errors.
   - Unsupported commands return a dedicated error code.

3. **Control-loop structure is in place on host**
   - `RobotControl` has dedicated high-rate loops for bus and estimator paths with periodic timing.
   - Thread lifecycle management (`start`/`stop` + `join`) is present and straightforward.

## Key findings and risks

### 1) Documentation and runtime behavior are out of sync (high)

`README.md` still documents a host flow centered on handshake + calibration + heartbeat, while `hexapod-server/src/hexapod-server.cpp` currently only parses/sorts calibration and prints it. The newer control-stack components are not wired into `main` yet.

**Impact:** new contributors/operators can follow docs and still not run expected behavior.

**Recommendation:** either (a) wire the new control stack in `main`, or (b) explicitly mark current `main` as a staging utility and document intended entrypoints.

### 2) Build is not reproducible in a clean environment (high)

Server build fails immediately without external dependency setup for `CppLinuxSerial` headers.

**Impact:** onboarding friction; CI cannot be trusted unless toolchain/dependencies are codified.

**Recommendation:** add explicit dependency installation instructions and/or vendoring/submodule strategy; add CI build checks for host side.

### 3) Potential protocol framing edge cases under long-running streams (medium)

The framed receive path appends bytes into `rxBuffer` and relies on `tryDecodePacket` to consume/advance. This is the right shape, but because stream corruption is normal on serial links, decode-recovery behavior should be tested heavily (invalid preambles, truncated CRCs, back-to-back malformed packets).

**Impact:** possible memory growth / delayed recovery if decode behavior is imperfect under noisy input.

**Recommendation:** add framing fuzz/unit tests and explicit bounds checks around maximum buffered bytes.

### 4) Mixed maturity level in host control pipeline (medium)

`RobotControl` scaffolding is good, but several paths are stubbed/commented (`controlLoop`, safety/diagnostics integration, hardware read failure handling).

**Impact:** system appears architecturally advanced while behavior remains partially inert.

**Recommendation:** define and implement a minimal operational slice (read -> estimate -> command) with explicit failure handling and telemetry logs.

### 5) Configuration and constants could be made safer (low/medium)

Global protocol constants are currently plain `const uint8_t` in a shared header. Stronger typing (`enum class`) and `constexpr` usage would reduce accidental misuse and improve compile-time behavior.

**Impact:** low immediate risk, but grows with protocol complexity.

**Recommendation:** migrate command/status/error codes toward scoped enums and helper conversion utilities.

## Priority action plan

### P0 (do next)

1. Align `README.md` with the actual executable behavior and current architecture.
2. Make host build reproducible (document/install dependencies, add CI build target).
3. Add framing parser tests for corruption/recovery and bounded-buffer behavior.

### P1

1. Wire `hexapod-server` `main` into `RobotControl` with a small operational loop.
2. Implement error/health reporting around hardware read/write failures.
3. Add integration tests for command round-trips (`HELLO`, calibration sync, joint targets, full hardware state).

### P2

1. Improve protocol typing (`enum class` + helpers).
2. Expand diagnostics and counters (NACK reasons, parser resync count, link health).
3. Revisit motion safety invariants (target limits, relay safety policy, watchdog behavior).

## Quick validation notes from this review

- The host build currently fails in this environment due to missing `CppLinuxSerial/SerialPort.hpp`.
- No behavioral runtime test was performed because host compilation did not complete.
- Review conclusions above are based on static inspection plus the observed build failure.

## Applied fix in this pass

- Added a receive-buffer guard in `tryDecodePacket` to cap unbounded growth when malformed or noisy serial data accumulates without yielding a valid frame. When the cap is exceeded, the decoder now keeps only the trailing bytes needed for potential frame recovery.
- Added shared framing constant `MAX_RX_BUFFER_BYTES` to make this limit explicit and configurable in one place.

### Why this fix

This directly addresses the previously identified medium-risk item around long-running serial corruption and delayed parser recovery by bounding memory usage while preserving resynchronization behavior.
