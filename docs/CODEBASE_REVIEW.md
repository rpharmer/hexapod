# Codebase Review: Hexapod

## Scope reviewed

- `README.md`
- `hexapod-common/include/hexapod-common.hpp`
- `hexapod-server/src/hexapod-server.cpp`
- `hexapod-client/hexapod-client.cpp`

## High-level assessment

The repository has a clear split between host (`hexapod-server`), firmware (`hexapod-client`), and shared protocol (`hexapod-common`). The handshake + calibration bootstrap path is implemented end-to-end and appears consistent with protocol constants.

At the same time, the code is currently best characterized as a solid prototype: startup communication works, but runtime control and resilience concerns (validation, retry policy, typed error handling, and observability) need to be expanded before this is production-ready for repeated field operation.

## Current strengths

1. **Good separation of concerns**
   - Shared constants and packet interfaces live in `hexapod-common`.
   - Transport-specific serial implementations remain on each side.

2. **Basic reliability foundations are present**
   - Sequence numbers are used in request/response flow.
   - Handshake validates protocol version and status.
   - Heartbeat command exists and is exercised by the host.

3. **Hardware control surface is already broad on firmware**
   - Calibration set/get, set target angle, relay control, current/voltage/sensor reads, and heartbeat handlers are present.

## Main gaps and risks

1. **Validation and bounds safety are incomplete**
   - Firmware accepts `SET_TARGET_ANGLE` servo index without checking valid range.
   - Sensor index for `GET_SENSOR` is not range checked.
   - Host does not validate parsed calibration values (e.g., min < max and within safe pulse range) before transmitting.

2. **Error semantics are currently too coarse**
   - `TIMEOUT` is reused for several payload/argument validation failures that are not actual timeouts.
   - This makes diagnosis harder and complicates automatic recovery behavior on the host side.

3. **Host runtime behavior is still startup-centric**
   - Host sends initial calibration and a small fixed heartbeat loop, then exits.
   - There is no long-lived control loop, state machine, or reconnect strategy suitable for robot runtime.

4. **Configuration and deploy ergonomics can improve**
   - README notes some runtime values are hard-coded in source.
   - Client README appears boilerplate and does not document project-specific firmware behavior.

5. **Testability and CI maturity are limited**
   - No obvious protocol unit tests/fuzz tests for framing and command decoding.
   - No host/firmware contract tests to guard protocol evolution.

## Prioritized next steps (near-term)

1. **Add strict input validation in firmware command handlers (P0)**
   - Reject invalid servo IDs and sensor IDs with dedicated error codes.
   - Validate payload lengths exactly (not only minimum size) where protocol requires fixed-width messages.

2. **Harden host calibration pipeline (P0)**
   - Validate calibration records after TOML parse:
     - expected count (18),
     - unique motor labels,
     - min/max ordering,
     - pulse bounds.
   - Fail fast with explicit diagnostics before opening serial.

3. **Introduce explicit protocol error taxonomy (P0)**
   - Add error codes for invalid argument, invalid payload length, out-of-range index, unsupported command, and busy/not-ready.
   - Update both host and client to use these consistently.

4. **Refactor host into an explicit connection state machine (P1)**
   - States: disconnected -> opening -> handshake -> calibration sync -> operational -> degraded/recovering.
   - Include retry backoff and reconnection policy.

5. **Define a runtime command path on host (P1)**
   - Expose APIs/CLI to send `SET_TARGET_ANGLE` and query telemetry.
   - Keep heartbeat as background liveness signal rather than a short startup check.

6. **Add protocol-level tests (P1)**
   - Unit tests for framing/deframing edge cases and corrupted bytes.
   - Golden tests for handshake and calibration packets.

## Future directions (mid/long-term)

1. **Safety and motion layer**
   - Add motion-profile generation (ramp/jerk limits) above raw set-angle commands.
   - Add interlock policies: relay behavior tied to communication health and watchdog expiry.

2. **Versioned capability negotiation**
   - Expand HELLO capability bits into a clear feature negotiation system.
   - Allow host to adapt behavior for older/newer firmware without manual branch logic.

3. **Observability and diagnosability**
   - Structured logs on host (command, seq, RTT, error code).
   - Lightweight diagnostics command on firmware for counters (dropped packets, NACK counts, last error).

4. **Protocol evolution strategy**
   - Add a compatibility matrix in docs.
   - Require contract tests for any command or payload format changes.

5. **Developer workflow and quality gates**
   - Add CI for host build + unit tests.
   - Add static analysis/linting where practical.
   - Replace boilerplate client README with project-specific build, flash, and bring-up playbooks.

## Suggested implementation roadmap

- **Phase 1 (stability):** input validation, richer errors, calibration checks, command handler hardening.
- **Phase 2 (operability):** host state machine, reconnect behavior, runtime command/telemetry CLI.
- **Phase 3 (safety/perf):** motion profiling, watchdog/interlocks, improved diagnostics.
- **Phase 4 (scale):** CI contract tests, capability negotiation, documented compatibility policy.
