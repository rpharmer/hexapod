# Next Steps, Fixes, and Improvements

This roadmap is refreshed from the current codebase and a fresh local configure/build pass.

## Status check (March 19, 2026)

Legend: ✅ complete, 🟡 partial/in progress, ❌ not started.

### 1) Immediate priorities (highest leverage)

1. **Implement real foot-target generation in `BodyController`** — ❌
   - `BodyController::update()` is still a placeholder and explicitly marked TODO.
2. **Make gait cadence command-driven in `GaitScheduler`** — ❌
   - `GaitScheduler::update()` still uses `control_config::kFallbackSpeedMag` and is marked TODO.
3. **Add behavior-level control loop checks** — ❌
   - No automated behavior tests are present for phase continuity or timeout/inhibit propagation.

### 2) Reliability and safety hardening

1. **Protocol/framing regression coverage** — ❌
   - No dedicated regression tests found for CRC, malformed payloads, unsupported commands, or sequence mismatch paths.
2. **Link-health recovery checks** — ❌
   - Runtime timeout/recovery logic exists, but no automated recovery test coverage was found.
3. **Improve fault observability** — 🟡
   - Fault states are logged and surfaced, but structured source metadata/timestamped transition records are still limited.

### 3) Architecture and maintainability

1. **Continue `RobotControl` decomposition** — 🟡
   - `ControlPipeline` and `LoopExecutor` extraction exists, but `RobotControl` still owns broad orchestration responsibilities.
2. **Externalize control/geometry tunables** — 🟡
   - Significant tunables are loaded from config with fallback validation; further schema/versioning hardening remains.
3. **Increase type safety at module boundaries** — ❌
   - Core interfaces still rely on primitive `double`/integer values for units at many boundaries.

### 4) Documentation and workflow

1. **Keep READMEs synchronized with architecture changes** — 🟡
   - Current READMEs are mostly aligned; this should remain part of each architecture-affecting change.
2. **Add baseline CI checks** — ❌
   - No CI pipeline/configuration files were found in the repository.
3. **Add change-impact template for control PRs** — ❌
   - No PR template/checklist artifact found for behavior + safety impact requirements.

## 1) Immediate priorities (highest leverage)

1. **Implement real foot-target generation in `BodyController`**
   - Replace default-output behavior with stance/swing target generation.
   - Apply motion/safety-aware clamping before IK.
   - Define minimum acceptance checks (e.g., deterministic stance height and swing trajectory continuity).

2. **Make gait cadence command-driven in `GaitScheduler`**
   - Replace fallback speed estimate with command-derived motion magnitude.
   - Smooth mode transitions (`SAFE_IDLE` ↔ `STAND` ↔ `WALK`) to avoid abrupt phase-rate changes.

3. **Add behavior-level control loop checks**
   - Validate gait phase progression continuity and transition behavior.
   - Validate command timeout handling and safety inhibit propagation.

## 2) Reliability and safety hardening

1. **Protocol/framing regression coverage**
   - CRC mismatch handling.
   - Invalid/truncated payload handling.
   - Unsupported command behavior.
   - Sequence mismatch behavior.

2. **Link-health recovery checks**
   - Heartbeat timeout to fault transition.
   - Recovery/re-handshake behavior after link interruption.

3. **Improve fault observability**
   - Include structured fault source + timestamp fields in status logs.
   - Record transition events for easier post-run diagnostics.

## 3) Architecture and maintainability

1. **Continue `RobotControl` decomposition**
   - Separate thread/loop execution mechanics from control-state orchestration.

2. **Externalize control/geometry tunables**
   - Move hardcoded constants to validated config with defaults and compatibility checks.

3. **Increase type safety at module boundaries**
   - Introduce stronger unit abstractions for angles, rates, and timing values.

## 4) Documentation and workflow

1. **Keep READMEs synchronized with architecture changes**
   - Ensure top-level and component READMEs reflect current file layout and flow ownership.

2. **Add baseline CI checks**
   - `hexapod-server` configure/build.
   - `hexapod-client` SDK setup target.
   - `hexapod-client` full firmware compile target.

3. **Add change-impact template for control PRs**
   - Require: behavior summary, test evidence, and safety impact statement.

## Suggested 30/60/90-day execution

- **Next 30 days**
  - Implement body controller policy and command-derived gait cadence.
  - Add first protocol regression tests (payload/CRC/unsupported command paths).

- **Next 60 days**
  - Add heartbeat timeout/recovery tests.
  - Begin `RobotControl` loop-runner extraction.

- **Next 90 days**
  - Externalize tunables with validation/versioning.
  - Tighten unit-typed boundaries across estimator/control/IK modules.

## Definition of done for roadmap items

Each completed item should include:

- reproducible build/test command output,
- updated documentation for behavior/architecture changes,
- explicit safety impact note (`reduced`, `unchanged`, or `introduced`).
