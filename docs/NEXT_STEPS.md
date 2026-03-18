# Next Steps, Fixes, and Improvements

This roadmap is refreshed from the current repository state and latest local build verification.

## 1) Immediate priorities (highest leverage)

1. **Implement real body control output generation**
   - Replace placeholder `BodyController::update()` behavior with stance/swing foot target logic.
   - Add saturation and safety-aware clamping before IK.

2. **Make gait cadence command-driven**
   - Replace fallback speed magnitude in `GaitScheduler` with command-derived motion intent magnitude.
   - Ensure smooth transitions between `SAFE_IDLE`, `STAND`, and `WALK`.

3. **Define behavior-level acceptance tests for control loop changes**
   - Add deterministic checks for gait phase progression, mode transition behavior, and command timeout response.

## 2) Reliability and safety hardening

1. **Protocol/framing regression tests**
   - Cover CRC mismatches, invalid payload lengths, truncated frames, and unsupported commands.

2. **Link-health and recovery testing**
   - Validate heartbeat timeout faulting and recovery flow.
   - Validate repeated `HELLO` behavior after activation and malformed handshake payload handling.

3. **Safety fault observability improvements**
   - Add structured logging/telemetry fields for fault source and transition timestamps.

## 3) Architecture and maintainability

1. **Continue `RobotControl` decomposition**
   - Separate thread/loop execution concerns from orchestration state ownership.

2. **Externalize geometry and tunables**
   - Move hardcoded geometry/control constants into versioned configuration with validation.

3. **Type-safety pass at module boundaries**
   - Introduce stronger types or wrappers for units (angles, rates, durations, timestamps).

## 4) Documentation and workflow

1. **Add a concise operator runbook**
   - Document startup order, expected handshake behavior, and safe-stop process.

2. **Set up baseline CI checks**
   - Server build.
   - Client `setup-sdks` target.
   - Full client firmware build.

3. **Add change-impact templates for control logic PRs**
   - Require test evidence and safety impact notes for behavior-affecting changes.

## Suggested 30/60/90-day plan

- **Next 30 days**
  - Implement body controller policy and command-driven gait cadence.
  - Add first protocol framing regression tests.

- **Next 60 days**
  - Add link-health recovery and safety transition tests.
  - Start `RobotControl` loop-runner extraction.

- **Next 90 days**
  - Externalize geometry/tuning config with validation.
  - Complete unit-typed API tightening for estimator/control/IK boundaries.

## Definition of done for roadmap items

Each completed item should include:

- reproducible build/test command output,
- documentation updates for behavior changes,
- explicit safety impact note (risk reduced/unchanged/introduced).
