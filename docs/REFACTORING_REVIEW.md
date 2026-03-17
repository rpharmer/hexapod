# Refactoring Review

This document captures a full-pass refactoring scan across the repository's production C++ code (`hexapod-common/`, `hexapod-server/`, and `hexapod-client/`) with suggested next actions.

## Scope reviewed

- Shared protocol and framing utilities in `hexapod-common/`
- Server runtime, control, kinematics, and safety code in `hexapod-server/`
- Pico client firmware and serial transport in `hexapod-client/`

## High-priority refactoring opportunities

1. **Unify timing/rate configuration constants**
   - Multiple control loops and watchdog checks embed period or timeout constants inline.
   - Recommendation: centralize loop rates and timeout values in a dedicated configuration module for easier tuning and testability.

2. **Reduce ownership of hard-coded geometry constants in constructors**
   - Kinematics code initializes robot geometry from embedded literal values.
   - Recommendation: move geometry tables and calibration values into config structures loaded at startup so hardware variants are easier to support.

3. **Extract fault-priority policy in safety logic**
   - Safety checks currently run sequentially and can overwrite previously-set faults.
   - Recommendation: introduce explicit fault-priority handling (e.g., first-fault-wins or severity ordering) to make behavior deterministic and auditable.

4. **Encapsulate repetitive leg iteration logic**
   - IK/control paths repeatedly iterate over `kNumLegs` while applying similar per-leg fallbacks.
   - Recommendation: extract reusable helpers for map/transform/fallback patterns over leg arrays.

## Medium-priority refactoring opportunities

1. **Strengthen typed units for angles and timestamps**
   - Current code uses raw `double` and `uint64_t` in many interfaces.
   - Recommendation: introduce lightweight wrappers/aliases for units (rad, deg, microseconds) to reduce accidental misuse.

2. **Split large orchestration classes by responsibility**
   - `RobotControl` coordinates threads, IO, control, and diagnostics.
   - Recommendation: extract scheduler/thread orchestration from control decision logic for easier testing and reduced coupling.

3. **Normalize logging strategy**
   - Console output is currently inline and ad hoc.
   - Recommendation: route logs through a shared logging abstraction with log levels and optional structured fields.

4. **Consolidate protocol framing validation paths**
   - Shared framing code can benefit from a single validation entrypoint and richer error reporting.
   - Recommendation: return explicit decode/validation status enums and map them to diagnostics consistently.

## Low-priority cleanup opportunities

1. **Formatting and naming consistency**
   - Minor inconsistencies in spacing and local variable naming styles are present.

2. **Explicitly document default/placeholder behavior**
   - Some components intentionally contain placeholder behavior (especially in early gait/control stages).
   - Recommendation: mark these with `TODO(owner/date)` style comments tied to milestone docs.

## Immediate change included with this review

As part of this pass, `SafetySupervisor` was lightly refactored to improve readability:

- promoted safety thresholds/timeouts to internal named constants,
- extracted command staleness logic to a helper,
- removed redundant state field re-initialization and clarified unused input handling.

This was a structural readability refactor and keeps the same public interface.
