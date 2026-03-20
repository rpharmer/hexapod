# Refactoring Review (2026-03-20)

This is the actionable extraction from `docs/CODEBASE_REVIEW.md`.

## Priority 0 — Stabilize and unblock

1. **Complete or revert the partial `RobotRuntime` stream-sample migration.**
   - Align `robot_runtime.cpp` with `robot_runtime.hpp` state declarations.
   - Align buffer element types (`T` vs `StreamSample<T>`) consistently.
   - Remove references to missing config fields (`config_.freshness`) unless fully introduced.

2. **Restore clean server build under current warning policy (`-Werror`).**
   - Remove dead/unused helper(s), including scenario-local unused intent builder.

3. **Establish a default test-enabled build path.**
   - Introduce a documented preset/CI job that always configures with tests enabled.

## Priority 1 — Improve runtime correctness and diagnosability

4. **Add explicit freshness contract between runtime stages.**
   - Add per-stream `sample_id` + timestamp usage conventions.
   - Gate control outputs on bounded input age.

5. **Unify host transport command outcomes and error taxonomy.**
   - Centralize ACK/NACK + timeout mapping.
   - Ensure deterministic error handling per command class.

6. **Add structured runtime telemetry.**
   - Loop jitter, stale input counts, transport RTT, fault transitions.

## Priority 2 — Maintainability and test seam improvements

7. **Introduce strict scenario validation/lint mode.**
   - Unknown-key detection and explicit enum/value failures.

8. **Reduce firmware singleton coupling.**
   - Push `FirmwareContext&` through handlers; keep singleton only at boot boundary.

9. **Dead-code and duplication cleanup.**
   - Consolidate duplicated motion-intent construction paths.

## Acceptance criteria

- `hexapod-server` builds cleanly in default documented workflow.
- Tests are executed (not merely discovered as zero) in at least one standard workflow.
- Stale-input behavior is deterministic and covered by tests.
- Transport errors are mapped consistently and observable via logs/metrics.
