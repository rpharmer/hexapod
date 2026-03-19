# Hexapod Codebase Review (March 19, 2026)

## Scope and method

This review covered host control (`hexapod-server`), firmware (`hexapod-client`), and shared protocol boundaries (`hexapod-common`) with emphasis on runtime safety, transport correctness, maintainability, and testability.

---

## Executive summary

The codebase has a clear subsystem split, practical runtime safety checks, and a coherent control pipeline. The highest-value next work is concentrated in three areas:

1. **Protocol contract hardening** (eliminate host/firmware command-shape drift).
2. **Safety behavior hardening** (fault latching and handshake/state transitions).
3. **Refactoring for maintainability** (decompose large parser/transport functions, reduce duplication and magic numbers).

If only one sprint is available, prioritize protocol/safety hardening first; this offers the strongest reduction in field failure risk.

---

## What is working well

- **Good architectural separation on host side.** Runtime orchestration (`RobotRuntime`) delegates bus, estimator, control, and safety steps to dedicated units, which is a strong base for future modular testing and loop-level instrumentation.
- **Control pipeline has clean stage ordering.** Gait → body control → IK is explicit and straightforward to reason about.
- **Config parser already enforces strict schema + calibration validation.** The server validates required keys, calibration key structure, duplicate detection, and pulse bounds before runtime.
- **Transport layer includes sequence IDs and ACK/NACK semantics.** This enables robust request/response validation and fault diagnosis.

---

## In-depth refactoring opportunities

## 1) Protocol contract mismatch risk (highest priority)

### Findings
- Host-side `set_target_angle(...)` serializes payload as `servo_id + float`.
- Firmware-side `handleSetAngleCommand(...)` expects payload size exactly `3` bytes (`servo_id + uint16`).

This is a direct contract mismatch and can cause immediate NACKs or undefined behavior if this command path is exercised.

### Why this matters
Protocol drift between host and firmware is the fastest path to “looks alive but not controllable” failures. Since both components evolve in one monorepo, this should be prevented by construction.

### Refactor recommendation
- Introduce a **single protocol payload codec source of truth** (shared encode/decode helpers in `hexapod-common` for every command, not just frame envelope).
- Add **host+firmware compatibility tests** in CI that construct payloads and validate decode expectations for all command IDs.
- Mark currently inconsistent commands as deprecated until aligned.

---

## 2) Handshake/state-transition bug in firmware lifecycle (highest priority)

### Findings
In command loop logic, any `HELLO` packet while not active invokes handshake handling and then unconditionally transitions firmware state to `ACTIVE`, regardless of whether handshake succeeded.

### Why this matters
A version-mismatch or malformed handshake can still move the firmware into active command handling mode. That weakens lifecycle guarantees and can hide handshake failures.

### Refactor recommendation
- Change handshake handler to return a structured result (`accepted/rejected` + reason).
- Transition to `ACTIVE` only on explicit acceptance.
- Add a regression test with invalid protocol version payload to guarantee state remains `WAITING_FOR_HOST`.

---

## 3) Safety fault behavior is non-latching (high priority)

### Findings
Safety evaluation creates a fresh `SafetyState` each cycle and only applies current-sample conditions. Fault history is not latched.

### Why this matters
Transient hard faults can clear immediately on next sample. For robotics systems, many fault classes (tip-over, bus timeout, over-current) should remain latched until deliberate operator recovery.

### Refactor recommendation
- Implement a **fault state machine** (`ACTIVE`, `LATCHED`, `RECOVERING`) with explicit clear conditions.
- Add “trip counters” and timestamps for post-mortem diagnostics.
- Expose a `CLEAR_FAULTS` command gated by SAFE_IDLE + heartbeats present.

---

## 4) Concurrency primitive can be simplified and optimized (medium priority)

### Findings
`DoubleBuffer<T>` currently uses both `std::mutex` and `std::atomic<int>`. With full mutex protection around read and write, the atomic index is unnecessary.

### Why this matters
The current approach is safe but semantically mixed (lock-based + atomics), increasing cognitive load and suggesting lock-free semantics that are not actually provided.

### Refactor recommendation
Choose one model:
- **Simple model:** remove atomic and keep mutex-based double-buffer.
- **Performance model:** implement true lock-free SPSC buffer with documented producer/consumer ownership.

Given current architecture, the simple model is likely best unless profiling proves lock contention.

---

## 5) Config parsing function is too large (medium priority)

### Findings
`tomlParser(...)` in `hexapod-server.cpp` contains schema validation, calibration parsing, sorting, tuning defaults, geometry defaults, and fallback behaviors in one long function.

### Why this matters
This is hard to unit test, hard to diff-review, and error-prone when adding new schema fields.

### Refactor recommendation
Split into focused helpers:
- `parseTransportConfig(...)`
- `parseCalibrationConfig(...)`
- `parseTuningConfig(...)`
- `parseGeometryConfig(...)`
- `validateSchemaHeader(...)`

Then add table-driven tests for each parser segment.

---

## 6) Magic-number duplication for joint/sensor dimensions (medium priority)

### Findings
Several firmware/host files repeatedly use literals like `18`, `6`, and byte counts derived from them.

### Why this matters
Dimensional constants drifting across modules silently break payload sizing and array expectations.

### Refactor recommendation
- Centralize protocol dimensions in `hexapod-common` (e.g., `kProtocolJointCount`, `kProtocolFootSensorCount`).
- Replace all transport payload-size literals with `constexpr` expressions derived from these constants.

---

## 7) Host runtime command generation is hardcoded in `main()` (medium priority)

### Findings
Server `main()` currently hardcodes a stand delay then continuously pushes TRIPOD walk commands.

### Why this matters
This limits usability, complicates testing of alternative gaits/modes, and mixes demo behavior with production runtime entrypoint.

### Refactor recommendation
- Move high-level behavior to a **mode/intent provider** abstraction.
- Support at minimum: static stand, scripted sequence, and external command source.
- Keep `main()` as composition root only.

---

## 8) ACK/NACK command handling duplication in bridge (low-medium priority)

### Findings
Handshake, heartbeat, generic ACK flow, and calibration upload each implement similar response checks with small variations.

### Why this matters
Duplicated protocol response logic increases inconsistency risk and makes it harder to enforce uniform telemetry/error reporting.

### Refactor recommendation
- Consolidate into reusable command transaction helper with policy flags (expects payload, status byte required, acceptable commands, timeout class).

---

## Proposed next steps (sequenced)

## Phase 1 (Safety + correctness hardening, 1 sprint)

1. Fix handshake transition gating on firmware side.
2. Align `SET_TARGET_ANGLE` payload contract host/firmware.
3. Add protocol conformance tests for all commands (payload size + semantics).
4. Implement latched safety faults with explicit clear pathway.

## Phase 2 (Maintainability + testability, 1 sprint)

1. Decompose TOML parser into module-level parsing functions.
2. Consolidate ACK/NACK transaction helpers.
3. Replace protocol magic numbers with shared constants.
4. Simplify `DoubleBuffer` synchronization model.

## Phase 3 (Architecture evolution, ongoing)

1. Introduce intent provider abstraction and remove hardcoded command loop behavior from `main()`.
2. Add simulation/mocked transport harness to run control pipeline in CI.
3. Add observability upgrades (fault counters, handshake metrics, loop jitter histograms).

---

## Suggested acceptance criteria for the next PRs

- Any protocol change includes host+firmware test vectors and backward-compatibility note.
- Any safety change includes explicit fault transition table (trip/hold/clear).
- Any parser change includes table-driven tests for valid/invalid config cases.
- Any loop timing change includes measured before/after jitter metrics.

---

## Risk if deferred

Deferring the top three items (protocol mismatch, handshake gating, fault latching) leaves the project exposed to field failures that are hard to diagnose because they manifest as intermittent transport/safety behavior.

