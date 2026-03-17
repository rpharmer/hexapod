# Refactoring Review

This document captures an updated refactoring scan across the repository's production C++ code (`hexapod-common/`, `hexapod-server/`, and `hexapod-client/`) and verifies prior recommendations against the current codebase.

## Scope reviewed

- Shared protocol and framing utilities in `hexapod-common/`
- Server runtime, control, kinematics, and safety code in `hexapod-server/`
- Pico client firmware and serial transport in `hexapod-client/`

## Validation of previous review items

### Addressed since the previous pass

1. **Scalar serialization helpers are now shared**
   - Client and server transports now route scalar read/write through `hexapod-common/include/serial_scalar_io.hpp`.
   - Remaining work is mostly call-site cleanup (optional), not duplicated byte-pack logic.

2. **RX buffer growth is now bounded in packet receive loops**
   - Both client and server `recv_packet()` paths apply max-buffer trimming while accumulating framed bytes.
   - Server-side trim path also emits an overflow diagnostic.

3. **Server serial receive path no longer does per-read front erases**
   - The server now uses a `readBufferHead` cursor and only clears/reset when consumed, avoiding repeated linear shifts on each scalar receive.

4. **Gait phase offset mapping has been extracted to table-driven constants**
   - `GaitScheduler` now uses named phase-offset arrays and a selector helper instead of per-leg switch logic.

5. **IK fallback assignment has been centralized**
   - `LegIK::solve()` now applies a unified fallback condition with a shared helper for estimated-leg fallback behavior.

## Current high-priority refactoring opportunities

1. **Reduce repeated scalar send wrappers in transports**
   - While low-level serialization is shared, each transport still contains many near-identical `send_*` wrappers with duplicated lambda bodies.
   - Recommendation: introduce a tiny internal helper per transport (e.g., `write_bytes`) and route all `send_*` wrappers through it.

2. **Fix inconsistent RX buffer limit policy between framing and transport layers**
   - `framing.cpp` enforces `MAX_RX_BUFFER_BYTES = 1024`, while client/server transport loops trim to 4096 bytes.
   - Recommendation: centralize buffer limits (single source of truth) and define ownership of truncation policy.

3. **Externalize geometry configuration instead of compile-time constants**
   - `defaultHexapodGeometry()` still embeds robot dimensions, mount angles, and servo offsets as compile-time literals.
   - Recommendation: load geometry/calibration from configuration files to support hardware variants without rebuild.

4. **Split `RobotControl` orchestration responsibilities**
   - `RobotControl` still owns thread lifecycle, loop scheduling, status aggregation, and domain coordination in one class.
   - Recommendation: extract loop runner/scheduler concerns from control-domain composition for easier testability.

## Current medium-priority refactoring opportunities

1. **Tighten typed units for time and angular values**
   - `std::chrono` usage has improved for loop periods, but interfaces still widely exchange raw `uint64_t` timestamps and `double` angles.
   - Recommendation: incrementally introduce stronger aliases/wrappers at API boundaries.

2. **Normalize diagnostics/logging strategy**
   - Logging remains a mix of `std::cout`, `std::cerr`, and `printf` across modules.
   - Recommendation: add a shared logging abstraction with levels and optional component tags.

3. **Clarify safety policy extensibility**
   - Fault prioritization is explicit in `SafetySupervisor`, but policy is still hardcoded in the class.
   - Recommendation: move fault ranking and trip policy into a data-driven policy table/config when additional faults are introduced.

4. **Retire stale fields and placeholders**
   - Example: `LegIK` includes an apparently unused `seq_tx_` member; `BodyController::update()` remains mostly placeholder logic.
   - Recommendation: remove dead fields and add milestone-linked TODO ownership for intentional placeholders.

## Current low-priority cleanup opportunities

1. **File hygiene and formatting consistency**
   - A few files still show inconsistent newline/formatting patterns.

2. **Comment modernization**
   - Some comments are historical or exploratory and can be tightened to current behavior.

## Suggested execution order

1. Unify/centralize RX buffer limit policy (`framing` + transport).
2. Simplify transport `send_*` wrappers to reduce copy-paste surface.
3. Break out `RobotControl` orchestration helpers and add targeted tests.
4. Move geometry to runtime configuration and validate with startup checks.
5. Apply typed-unit wrappers and logging abstraction incrementally.
