# Next Steps Execution Plan

## Goal

Convert the review findings into a staged, low-risk implementation plan with clear deliverables.

## Phase 1 (1 sprint): Stabilize architecture seams

### Deliverables
- Extract host transport concerns from `hardware_bridge.cpp` into separate modules while preserving existing interfaces.
- Introduce a `FreshnessPolicy` component and route runtime gating through it.
- Add/extend tests for retry outcomes, ACK/NACK handling, and freshness edge cases.

### Exit criteria
- `ctest --preset tests` remains green.
- No CLI or protocol behavior changes.
- Net reduction in file-level complexity for transport path.

## Phase 2 (1 sprint): Config and app composition cleanup

### Deliverables
- Split TOML parser into section-focused parsers + shared validators.
- Add CLI options parsing abstraction and mode runners (`scenario`, `interactive/controller`).
- Add tests for parse failures, defaults, and CLI mode selection.

### Exit criteria
- Parsed config parity with current behavior for baseline configs.
- `hexapod-server.cpp` reduced to composition-root concerns.

## Phase 3 (1 sprint): Firmware command hardening

### Deliverables
- Expand firmware tests beyond router (motion/power/sensing command handlers).
- Introduce reusable payload validation/decode helpers.
- Remove non-essential direct global context usage in command handlers.

### Exit criteria
- Firmware command handlers validated for valid + invalid payload paths.
- Command behavior parity maintained (ACK/NACK/error codes).

## Phase 4 (ongoing): Tooling and docs quality loop

### Deliverables
- Add architecture-boundary guidance and optional static checks.
- Keep `README` and docs map synchronized with actual repository content.
- Track refactor progress through a simple checklist in PR templates.

### Exit criteria
- No stale doc references.
- New contributors can find runtime, protocol, and test guidance in <10 minutes.

## Metrics to track across phases

- Server test runtime and pass rate.
- Number of files >300 LOC in core runtime path.
- Number of firmware command handlers with dedicated tests.
- Mean time to review/merge transport and safety-related PRs.

## Practical first PR candidates

1. Extract `CommandClient` and `HandshakeClient` from `hardware_bridge.cpp` with zero behavior change.
2. Add unit tests for command retry + NACK mapping edge cases.
3. Introduce `FreshnessPolicy` class and move current lambda checks into it.
4. Add firmware tests for `SET_JOINT_TARGETS` and `SET_ANGLE_CALIBRATIONS` invalid payload handling.
