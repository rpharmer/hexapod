# Next Steps

See `docs/CODEBASE_REVIEW.md` for the full rationale.

## Sprint 1 (must-do)

- Fix firmware handshake activation gating.
- Align `SET_TARGET_ANGLE` payload formats host + firmware.
- Add protocol conformance tests for all commands.
- Implement latched safety faults with explicit clear conditions.

## Sprint 2

- Split `tomlParser(...)` into smaller parser modules + tests.
- Centralize transport dimensions in shared constants.
- Refactor host ACK/NACK transaction handling.
- Simplify `DoubleBuffer` implementation model.

## Ongoing

- Introduce intent-provider abstraction for server runtime behavior.
- Build simulation/mocked transport CI harness.
- Expand runtime telemetry and fault analytics.

