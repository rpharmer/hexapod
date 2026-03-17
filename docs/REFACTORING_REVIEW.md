# Refactoring Review

This document is an updated refactoring pass across the repository's production C++ code and a validation of prior refactoring notes against the current files.

## Scope reviewed

- Shared protocol/framing utilities in `hexapod-common/`
- Server runtime/control/kinematics/safety in `hexapod-server/`
- Pico client firmware + serial transport in `hexapod-client/`

## Validation of prior review items (current status)

### Confirmed addressed

1. **Scalar serialization helpers are shared**
   - Client and server scalar read/write paths both use shared `read_scalar` helpers in `framing.hpp`.

2. **RX buffer growth is bounded in packet receive loops**
   - Both client/server packet receive loops trim accumulated RX buffers using the shared transport limit constant.

3. **Server serial receive path avoids per-read front erases**
   - Server receive now uses `readBufferHead` and only clears when fully consumed.

4. **Gait phase offset mapping is table-driven**
   - `GaitScheduler` uses named phase-offset arrays and a gait selector helper.

5. **IK fallback assignment is centralized**
   - `LegIK::solve()` uses one fallback condition and `apply_estimated_leg_fallback()`.

### Corrected from the previous review (stale findings)

1. **Framing vs transport RX limits are no longer inconsistent**
   - Previous note claimed `framing` and transport limits differed (`1024` vs `4096`).
   - Current code defines both `MAX_RX_BUFFER_BYTES` and `MAX_TRANSPORT_RX_BUFFER_BYTES` as `1024` in `framing.hpp`.

2. **`LegIK` stale field note is obsolete**
   - Previous note referenced an unused `seq_tx_` field.
   - `LegIK` no longer contains that field.

3. **Unused `serial_scalar_io.hpp` removed**
   - The standalone scalar serial I/O header had no remaining in-repo consumers and was removed.

## Current high-priority refactoring opportunities

1. **Remove repeated server `send_*` lambda wrappers**
   - `SerialCommsServer::send_*` methods are near-identical and all forward to `write_bytes`.
   - Recommendation: add a small templated/member helper so each `send_*` is one-line and no repeated lambdas are needed.

2. **Fix transport write allocation in server path**
   - `SerialCommsServer::write_bytes()` constructs a new `std::vector<uint8_t>` for every write call.
   - Recommendation: if library API allows, reuse a buffer or batch writes to reduce allocation churn.

3. **Split `RobotControl` orchestration responsibilities**
   - `RobotControl` still owns thread lifecycle, loop timing, coordination, status aggregation, and diagnostics output.
   - Recommendation: extract loop runner/scheduler + status/reporting concerns into dedicated units to improve testability.

4. **Externalize geometry/calibration configuration**
   - `defaultHexapodGeometry()` still embeds dimensions, mount angles, and servo offsets in compile-time literals.
   - Recommendation: load geometry/calibration from config to support hardware variants without rebuild.

## Current medium-priority refactoring opportunities

1. **Tighten typed units around timestamps and angles**
   - Internal loops use `std::chrono` for periods, but interfaces still rely heavily on raw `uint64_t timestamp_us` and `double` angles.
   - Recommendation: add stronger aliases/wrappers at API boundaries.

2. **Normalize logging/diagnostics strategy**
   - Code still mixes `std::cout` and `std::cerr` directly across modules.
   - Recommendation: introduce a shared logging abstraction with levels/component tags.

3. **Make safety policy data-driven as checks expand**
   - `SafetySupervisor` fault priorities and trip policy are encoded directly in methods.
   - Recommendation: move ranking/policy to a table/config once more fault sources are added.

4. **Track placeholder controller milestones explicitly**
   - `BodyController::update()` is intentionally placeholder logic with TODO comments and unused inputs.
   - Recommendation: tie TODOs to milestone IDs/issues and remove placeholders once replacement lands.

## Current low-priority cleanup opportunities

1. **Header hygiene in `framing.hpp`**
   - Includes and commented legacy snippets can be trimmed (`stdio.h`, older commented template helpers).

2. **Comment and formatting consistency**
   - Several files still contain historical comments or inconsistent spacing/newline style.

## Suggested execution order

1. Reduce server transport wrapper duplication + write allocation overhead.
2. Split `RobotControl` loop orchestration/reporting concerns.
3. Externalize geometry/calibration loading and validate startup checks.
4. Introduce typed-unit wrappers incrementally at control API boundaries.
5. Consolidate logging and clean low-priority header/comment hygiene.
