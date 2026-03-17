# Refactoring Review

This document captures an updated refactoring pass based on the current repository state.

## Scope reviewed

- Shared framing/protocol utilities in `hexapod-common/`
- Server runtime/control/kinematics/safety in `hexapod-server/`
- Pico firmware + serial command handling in `hexapod-client/`

## Validation of previously reported items

### Confirmed still true

1. **Shared scalar framing helpers are in use**
   - Client/server encode/decode paths both use common framing helpers from `hexapod-common`.

2. **RX buffer growth is bounded**
   - Both client and server trim transport RX buffers with `MAX_TRANSPORT_RX_BUFFER_BYTES`.

3. **Server serial receive avoids repeated front erases**
   - Server receive path still uses `readBufferHead` and clears only when fully consumed.

4. **Gait phase mapping is table-driven**
   - `GaitScheduler` uses phase-offset arrays selected by gait type.

### Corrected / no longer priority findings

1. **Server send wrappers duplication is no longer an active issue**
   - Current server serial transport exposes a single `send_packet` implementation that writes encoded frames directly.

2. **Framing vs transport buffer mismatch is resolved**
   - Buffer limits remain aligned at `1024` bytes.

## Current high-priority refactoring opportunities

1. **Split firmware monolith (`hexapod-client.cpp`)**
   - The firmware file still combines board initialization, command routing, protocol handling, and device I/O logic.
   - Recommendation: extract command handlers and hardware adapters into focused translation units.

2. **Implement explicit firmware shutdown semantics**
   - Infinite receive loop currently leaves cleanup code unreachable.
   - Recommendation: add a stop condition and make cleanup behavior intentional and testable.

3. **Break down `RobotControl` responsibilities**
   - `RobotControl` continues to own thread lifecycle, loop timing, coordination, and status emission.
   - Recommendation: separate loop scheduling, command arbitration, and reporting.

4. **Externalize geometry/calibration defaults**
   - `defaultHexapodGeometry()` still hardcodes body dimensions, mount angles, and sign conventions.
   - Recommendation: load geometry/calibration from config to support hardware variants without rebuilds.

## Medium-priority opportunities

1. **Replace placeholder control logic**
   - `BodyController::update()` currently returns default targets and explicitly ignores inputs.

2. **Use command-driven gait speed estimation**
   - `GaitScheduler::update()` still relies on fallback speed constants.

3. **Consolidate logging strategy boundaries**
   - Logging infrastructure exists, but direct stream usage and mixed logging styles still appear across modules.

4. **Fix documentation/source-of-truth alignment**
   - The top-level README references a non-existent protocol file and duplicates a firmware quick-verify flow.

## Low-priority cleanup opportunities

1. **Comment hygiene and dead-code trimming**
   - Remove outdated comments and unreachable sections once shutdown behavior is implemented.

2. **Minor include/style consistency pass**
   - Normalize include ordering and formatting in touched files as part of future behavior changes.

## Suggested execution order

1. Refactor firmware monolith + add explicit shutdown path.
2. Split `RobotControl` orchestration/reporting responsibilities.
3. Replace placeholder body controller and fallback gait-speed logic.
4. Externalize geometry/calibration configuration.
5. Clean up docs/style/dead code.
