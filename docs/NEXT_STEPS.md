# Next Steps

## Immediate (this week)

1. **Protocol correctness patch**
   - Fix `SET_TARGET_ANGLE` payload length check on firmware side.
   - Add regression test validating host encode + firmware decode for this command.

2. **Packet safety hardening**
   - Re-enable/implement max payload guard in `encodePacket`.
   - Add test coverage for payload size boundaries (0 bytes, max valid bytes, overflow case).

3. **CI quality gates**
   - Keep strict compile (`-Werror`) for normal builds.
   - Add optional static-analysis job (`-fanalyzer`, sanitizer builds) that reports warnings without blocking merges initially.

## Near-term (2-4 weeks)

4. **Control-loop determinism**
   - Document and implement one deterministic loop ordering (bus → estimate → safety → control → actuate).
   - Add simulation tests for stale command timeout, bus timeout, and fault latching/recovery transitions.

5. **Protocol test harness**
   - Add host-side packet fixture corpus and fuzz/robustness tests for framing decoder resync logic.

6. **Refactor transport/protocol layers**
   - Introduce shared command codecs and table-driven command dispatch to reduce serialization drift.

## Medium-term (1-2 months)

7. **Functional motion pipeline completion**
   - Replace `BodyController` placeholder with initial gait foot placement policy.
   - Improve estimator to produce non-default body orientation/velocity estimates consumed by safety and control.

8. **Operator interfaces**
   - Replace hard-coded motion intent loop in `main()` with a configurable runtime command source (teleop, script, API).

9. **Operational observability**
   - Add structured health snapshots (bus RTT, NACK rates, loop jitter histograms, fault counters) and log sampling.

## Definition of done for next milestone

- Full protocol command matrix covered by tests.
- Deterministic control-cycle sequencing implemented and documented.
- No known payload-shape mismatches between host and firmware.
- Basic walking pipeline no longer returns default/no-op leg targets.

