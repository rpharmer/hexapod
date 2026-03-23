# Next Steps (March 23, 2026)

## Phase 1 (1 week) — Core maintainability

1. Refactor `SimpleHardwareBridge` request/decode duplication.
2. Split `RobotRuntime::controlStep` into pure decision helpers.
3. Add/adjust focused unit tests for freshness gate edge cases.

## Phase 2 (1 week) — Firmware dispatch cleanup

1. Replace forwarding wrappers in `command_dispatch.cpp`.
2. Ensure payload policy behavior remains unchanged.
3. Extend firmware host-test coverage for invalid payload lengths.

## Phase 3 (1 week) — Protocol metadata unification

1. Create shared command metadata source in `hexapod-common`.
2. Migrate host command-name telemetry to shared metadata.
3. Add a consistency test ensuring command IDs and route entries remain aligned.

## Ongoing

- Run scenario sweep in sim mode before hardware validation.
- Track command failure and freshness-stale counters over longer test runs.
- Keep documentation synchronized with implementation changes.
