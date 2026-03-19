# Hexapod Codebase Review (2026-03-19)

## Scope and method

This review covered:

- Host control stack (`hexapod-server/`)
- Firmware stack (`hexapod-client/`)
- Shared framing/protocol layer (`hexapod-common/`)

Checks performed:

1. Architectural read-through of control/runtime/transport components.
2. Manual protocol trace from server command emitters to firmware handlers.
3. Concurrency and fault-handling inspection.
4. Build validation for the host target (`hexapod-server`).

## Executive summary

The codebase has a clear, modular subsystem split (runtime pipeline, safety, estimator, transport), and a good baseline of config/schema validation on the host. However, there are several correctness and reliability risks that should be addressed before field operation, especially around cross-thread data exchange and host↔firmware state semantics.

### Overall assessment

- **Architecture clarity:** Good
- **Safety posture:** Moderate (good intent, some gaps)
- **Protocol robustness:** Moderate
- **Concurrency correctness:** Needs improvement
- **Test coverage:** Low (no automated tests present)

## Key strengths

1. **Clean subsystem separation on host**
   - Runtime loop orchestration (`RobotControl`/`RobotRuntime`) is clearly decomposed by responsibility (bus, estimator, control, safety, diagnostics).
2. **Config validation discipline**
   - TOML parsing validates schema, calibration key set, ranges, and duplicates before startup.
3. **Protocol framing encapsulation**
   - Shared encode/decode logic in `hexapod-common` avoids duplicated packet-shape logic.
4. **Error reporting baseline**
   - Logging is present at key host transport failure points (timeouts, sequence mismatches, NACKs).

## High-priority findings

### 1) Potential data race in `DoubleBuffer<T>` under multi-reader/writer overlap

**Severity:** High  
**Location:** `hexapod-server/include/double_buffer.hpp`

`DoubleBuffer<T>` flips between two slots and returns `T` by value. A writer can wrap around and overwrite the same slot that a slow reader is still copying, because there is no reader quiescence or sequence validation. This is not guaranteed safe for general `T` and can produce undefined behavior.

**Why it matters:** The runtime uses this structure to share control-critical state (`RawHardwareState`, `EstimatedState`, `SafetyState`, etc.) between asynchronous loops.

**Recommendation:**
- Replace with an SPSC ring buffer with sequence counters, or
- Add a generation/seqlock pattern to detect torn reads, or
- Use lock-free primitives only for trivially copyable fixed-width payloads plus version stamps.

### 2) Host/firmware unit mismatch for joint position telemetry in full-state response

**Severity:** High  
**Locations:**
- Firmware: `hexapod-client/sensing_commands.cpp` (`handleGetFullHardwareStateCommand`)
- Host decode: `hexapod-server/src/hardware_bridge.cpp` (`decode_full_hardware_state`)

Firmware currently serializes `firmware().servos.value(s)` into the full-state payload. Elsewhere in firmware, `servos.value(...)` is set with pulse values, implying this is likely pulse-width domain. Host decodes those bytes as `pos_rad` directly.

**Why it matters:** Estimation and safety logic may ingest incorrectly scaled joint states, causing false tilt dynamics, gait instability, or inappropriate faulting behavior.

**Recommendation:**
- Define a single protocol contract for full-state joint units (prefer radians).
- On firmware, serialize tracked radian command/state (`jointTargetPositionsRad`) or measured radian estimates if available.
- Add a protocol-level test asserting unit consistency across host and firmware.

### 3) No host liveness timeout on firmware command loop after activation

**Severity:** High  
**Location:** `hexapod-client/command_dispatch.cpp`

After entering `ACTIVE`, firmware continues indefinitely unless receiving `KILL`. If host disappears without sending shutdown commands, there is no explicit inactivity timeout that transitions to a safer state.

**Why it matters:** A lost USB session can leave servos enabled indefinitely under stale command state.

**Recommendation:**
- Track last heartbeat/command timestamp on firmware.
- Enforce timeout-to-safe behavior (disable servos, drop relay, or transition to `WAITING_FOR_HOST`).
- Return a timeout status code in heartbeat ACK/NACK path for observability.

## Medium-priority findings

### 4) Transport receive path treats short read as hard packet miss

**Severity:** Medium  
**Locations:**
- `hexapod-server/src/serialCommsServer.cpp`
- `hexapod-client/serialCommsClient.cpp`

`recv_packet` fetches one byte at a time; when timeout occurs, it returns `false` immediately. This can turn normal framing latency into frequent command failures at higher load/jitter.

**Recommendation:**
- Use bounded retry window per packet attempt.
- Distinguish timeout vs protocol failure in return type/logging.
- Optionally read larger chunks where platform API allows.

### 5) Safety supervisor ignores raw hardware state inputs

**Severity:** Medium  
**Location:** `hexapod-server/src/safety_supervisor.cpp`

`evaluate()` currently casts away `raw` as unused, so voltage/current/contact anomalies do not influence faulting even though data is available.

**Recommendation:**
- Add configurable checks for undervoltage/overcurrent/contact plausibility.
- Tie fault policy to `torque_cut` behavior by fault class.

## Low-priority findings

### 6) Command and protocol regression tests are missing

**Severity:** Low  
**Locations:** repo-wide

There are no automated tests for:
- Framing decode edge cases (corruption, truncation, resync)
- Host↔firmware command contract conformance
- Safety decision regressions

**Recommendation:**
- Add host-side unit tests for `framing.cpp`, `SafetySupervisor`, and parser validation.
- Add protocol fixture tests that replay known packets and assert responses.

## Suggested remediation roadmap

### Phase 1 (safety/correctness blockers)

1. Fix telemetry unit contract mismatch for full hardware state.
2. Add firmware host-liveness timeout behavior.
3. Replace or harden `DoubleBuffer<T>` implementation.

### Phase 2 (reliability hardening)

4. Improve serial receive timeout/retry semantics.
5. Expand safety checks to include raw electrical/contact signals.

### Phase 3 (quality and maintainability)

6. Add protocol + safety automated tests in CI.
7. Document command/state machine invariants in `docs/FIRMWARE.md` and server README.

## Build/verification notes

- Host build completed successfully with strict warning flags enabled (`-Werror`).
- Firmware target was not built in this environment due external SDK/hardware dependency expectations.
