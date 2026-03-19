# Refactoring Opportunities (March 19, 2026)

## Priority 0 (bug/security/safety correctness)

1. **Fix `SET_TARGET_ANGLE` payload-size mismatch in firmware.**
   - Change payload check from 3 bytes to `sizeof(uint8_t) + sizeof(float)`.
   - Add an assertion-style helper for payload byte counts to avoid repeated literals.

2. **Restore explicit max-payload guard in `encodePacket`.**
   - Reject payload sizes that cannot be represented in `LEN` (uint8).
   - Return error/boolean or expose a `Result` type rather than constructing invalid frames.

## Priority 1 (maintainability and reliability)

3. **Introduce a typed protocol codec layer shared by host + firmware.**
   - Current code manually serializes/deserializes with repeated `memcpy`, `read_scalar`, offset logic.
   - Replace with `struct` codecs per command (`encode/decode` pair) and shared validation.

4. **Break `SimpleHardwareBridge` into cohesive collaborators.**
   - Suggested split:
     - `TransportSession` (seq, tx/rx, retries)
     - `HandshakeClient`
     - `HardwareStateCodec`
     - `CommandClient`
   - Keep top-level `IHardwareBridge` as façade.

5. **Replace category switch-dispatch in firmware with a table-driven router.**
   - Map command ID → handler function + expected payload policy.
   - Gains: lower duplication, easier extension, testability of unknown command behavior.

6. **Define explicit control-cycle execution contract.**
   - Either:
     - single real-time loop with ordered phases, or
     - multi-thread with barrier/latch synchronization between phases.
   - Current free-running loops can read inconsistent snapshots.

## Priority 2 (clarity/perf)

7. **Consolidate duplicated RX buffer trimming logic.**
   - `serialCommsServer.cpp` and `serialCommsClient.cpp` each have near-identical `trim_rx_buffer`.

8. **Prefer compile-time constants and utility wrappers for byte sizes.**
   - Avoid hardcoded `3`, `1`, etc. for payload checks.
   - Use `constexpr std::size_t kExpected...` everywhere.

9. **Create mode-intent command source abstraction for server main.**
   - Add interface such as `IMotionIntentSource` with implementations for scripted demo, CLI, or teleop.

## Proposed staged refactor plan

### Stage A (1-2 days)

- Fix payload mismatch bug.
- Add max payload guard.
- Add protocol unit tests for all command payload lengths.

### Stage B (3-5 days)

- Add command codec module in `hexapod-common`.
- Migrate firmware handlers to typed decode helpers.
- Migrate server bridge to typed encode helpers.

### Stage C (1 week)

- Introduce deterministic loop sequencing model and tests.
- Replace body-controller placeholder with minimal stance/swing implementation.
- Improve estimator from passthrough to filtered body state estimate.

