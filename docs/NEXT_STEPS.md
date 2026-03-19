# Next Steps, Fixes, and Improvements

This roadmap was refreshed against the current repository state and is intended to guide implementation, test coverage, and release-readiness work for the next quarter.

## Status check (March 19, 2026)

Legend: ✅ complete, 🟡 partial/in progress, ❌ not started.

## 1) Immediate priorities (highest leverage)

1. **Implement real foot-target generation in `BodyController`** — ❌
   - `BodyController::update()` remains placeholder/TODO-driven.
   - Replace default-output behavior with stance/swing target generation.
   - Apply motion and safety-aware clamping before IK.
   - Acceptance checks: deterministic stance height and swing trajectory continuity.

2. **Make gait cadence command-driven in `GaitScheduler`** — ❌
   - `GaitScheduler::update()` still relies on `control_config::kFallbackSpeedMag`.
   - Replace fallback speed estimate with command-derived motion magnitude.
   - Smooth mode transitions (`SAFE_IDLE` ↔ `STAND` ↔ `WALK`) to avoid abrupt phase-rate changes.

3. **Add behavior-level control loop checks** — ❌
   - Add automated checks for gait phase progression continuity.
   - Validate timeout/inhibit propagation through behavior transitions.

## 2) Reliability and safety hardening

1. **Protocol/framing regression coverage** — ❌
   - Add tests for CRC mismatch handling.
   - Add tests for invalid/truncated payload handling.
   - Add tests for unsupported command and sequence-mismatch behavior.

2. **Link-health recovery checks** — ❌
   - Add heartbeat-timeout → fault transition coverage.
   - Add post-interruption recovery and re-handshake checks.

3. **Improve fault observability** — 🟡
   - Fault state surfacing exists but still needs richer structured metadata.
   - Add source-tagged + timestamped transition records for diagnostics.

## 3) Architecture and maintainability

1. **Continue `RobotControl` decomposition** — 🟡
   - `ControlPipeline`/`LoopExecutor` extraction exists.
   - Remaining step: narrow `RobotControl` orchestration scope.

2. **Externalize control/geometry tunables** — 🟡
   - Config-driven tunables exist with fallback validation.
   - Remaining step: stronger schema/versioning and compatibility guards.

3. **Increase type safety at module boundaries** — ❌
   - Introduce stronger unit abstractions for angles, rates, and timing values.

## 4) Documentation and workflow

1. **Keep READMEs synchronized with architecture changes** — 🟡
   - Mostly aligned now; enforce this as a required part of architecture-impacting changes.

2. **Add baseline CI checks** — ❌
   - Add CI for:
     - `hexapod-server` configure/build,
     - `hexapod-client` SDK setup target,
     - `hexapod-client` full firmware compile target.

3. **Add change-impact template for control PRs** — ❌
   - Require behavior summary, test evidence, and safety-impact statement.

## Suggested execution timeline (anchored to calendar dates)

- **By April 30, 2026**
  - Implement body-controller targeting policy.
  - Implement command-derived gait cadence.
  - Land first protocol regression tests (CRC/payload/unsupported-command paths).

- **By June 30, 2026**
  - Add heartbeat timeout + recovery regression checks.
  - Continue `RobotControl` decomposition of loop/run mechanics.

- **By September 30, 2026**
  - Externalize remaining tunables with schema/version checks.
  - Improve unit-typed boundaries across estimator/control/IK interfaces.

## Definition of done for roadmap items

Each completed item should include:

- reproducible build/test command output,
- updated documentation for behavior/architecture changes,
- explicit safety impact note (`reduced`, `unchanged`, or `introduced`).
