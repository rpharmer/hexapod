# Next Steps, Fixes, and Improvements

This roadmap is refreshed based on the current code review and successful server/firmware builds.

## 1) Immediate priorities (highest leverage)

1. **Implement real body control policy**
   - Replace `BodyController::update()` placeholder outputs with stance/swing target generation.
   - Add constraints and saturation checks before IK.

2. **Make gait rate command-driven**
   - Replace fallback speed magnitude in `GaitScheduler` with values derived from `MotionIntent`.
   - Validate smooth ramping and mode transitions (`STAND` ↔ `WALK`).

3. **Start splitting firmware monolith**
   - Extract command dispatch and command handler groups from `hexapod-client.cpp`.
   - Keep behavior unchanged while reducing file-level coupling.

## 2) Reliability and safety hardening

1. **Protocol framing regression tests**
   - Cover CRC failures, truncated payloads, invalid lengths, and unsupported commands.

2. **Link-health scenario tests**
   - Test heartbeat timeout and recovery behavior.
   - Test out-of-sequence packets and malformed handshake payloads.

3. **Explicit firmware runtime states**
   - Introduce state tracking for boot, waiting-for-host, active, and stopping.
   - Ensure kill/stop behavior is deterministic and observable.

## 3) Architecture and maintainability

1. **Refactor `RobotControl` into smaller units**
   - Separate loop scheduling from control computation and diagnostics publication.

2. **Externalize geometry/calibration configuration**
   - Move hardcoded geometry defaults to config with schema/version checks.

3. **Tighten module contracts**
   - Use stronger types/aliases for units and timestamps across estimator/control/IK boundaries.

## 4) Documentation and developer workflow

1. **Condense README command sections**
   - Keep one concise server flow and one concise client flow with setup/full-build variants.

2. **Add a short operator runbook**
   - Include startup sequence, handshake expectations, and safe-stop actions.

3. **Define baseline CI checks**
   - At minimum: server build, client setup-sdks target, and full client firmware build.

## Suggested 30/60/90 day plan

- **Next 30 days**
  - Land body-controller and gait-rate logic improvements.
  - Begin `hexapod-client.cpp` split without behavior changes.

- **Next 60 days**
  - Add protocol/safety regression tests.
  - Introduce explicit firmware runtime states.

- **Next 90 days**
  - Complete `RobotControl` decomposition.
  - Ship config externalization for geometry/calibration defaults.

## Definition of done for roadmap items

Each completed item should include:

- reproducible build/test evidence,
- updated documentation,
- explicit safety impact note (risk reduced/unchanged/introduced).
