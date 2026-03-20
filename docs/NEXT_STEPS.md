# Next Steps (2026-03-20)

## Sprint 0 (immediate unblock)

1. Fix `hexapod-server` compile failures in `RobotRuntime`/scenario code.
2. Submit a stabilization patch that compiles cleanly with current `-Werror` flags.
3. Re-run baseline build and publish exact command outputs in PR.

## Sprint 1 (reliability hardening)

1. Add explicit freshness contract (timestamp + sample_id policy) across runtime streams.
2. Add tests for stale estimator and stale intent behavior.
3. Add structured control-loop timing and stale-input metrics.

## Sprint 2 (workflow quality)

1. Make test-enabled builds first-class (`cmake` preset + CI lane).
2. Update server README to recommend test-enabled default commands.
3. Enforce test execution on merge path.

## Sprint 3 (protocol and firmware maintainability)

1. Normalize host transaction handling and protocol error mapping.
2. Add scenario strict-validation mode/linter for authoring safety.
3. Reduce firmware singleton coupling and improve handler unit-test seams.

## Ongoing guardrails

- Track build breakage rate and mean time to restore green.
- Track loop jitter and stale-input event rates over scenario runs.
- Track transport error rates by command and NACK reason.
