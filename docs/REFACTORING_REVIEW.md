# Refactoring Review (2026-03-20)

This document extracts concrete refactoring work-items from `docs/CODEBASE_REVIEW.md`.

## Priority 0

1. Replace mutable global configuration with immutable injected config objects.
2. Add freshness/sample-id contracts between bus/estimator/control/safety stages.
3. Consolidate host transport ACK/NACK and transaction semantics.

## Priority 1

4. Make test execution the default in at least one documented build profile.
5. Add strict scenario TOML validation and lint checks.
6. Deduplicate motion-intent construction helpers.

## Priority 2

7. Reduce firmware singleton coupling by passing `FirmwareContext&` into handlers.
8. Run naming/typo and small consistency cleanup with no behavior changes.

## Acceptance criteria

- No behavior regressions in nominal stand/walk or scenario replay.
- Existing tests pass and new tests are added for changed invariants.
- Deterministic failure behavior for stale intent and transport faults.
