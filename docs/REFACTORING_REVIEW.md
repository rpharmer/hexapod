# Refactoring Review

This document captures an updated refactoring pass against the current repository state.

## Scope reviewed

- Shared framing/protocol utilities in `hexapod-common/`
- Server runtime/control/kinematics/safety in `hexapod-server/`
- Pico firmware + serial command handling in `hexapod-client/`

## Status of previously tracked items

### Confirmed still true

1. **Shared framing helpers are consistently used**
   - Client/server packet encode/decode paths continue to rely on `hexapod-common` framing helpers.

2. **Transport RX growth is bounded**
   - RX buffers remain capped by transport framing limits.

3. **Server receive path avoids repeated vector front erases**
   - The read-head approach remains in place.

4. **Gait phase mapping is table-driven**
   - `GaitScheduler` still selects offsets via gait-specific arrays.

### Updated from previous review

1. **Firmware cleanup is now reachable**
   - `hexapod-client.cpp` breaks from the command loop on `KILL` and executes cleanup.
   - Cleanup behavior now disables servos before shutdown.

2. **README protocol reference drift is resolved**
   - The top-level README now points to existing files for protocol details.

## Current high-priority refactoring opportunities

1. **Decompose firmware monolith (`hexapod-client.cpp`)**
   - The file still combines hardware setup, transport loop, command dispatch, and handlers.
   - Suggested split: `firmware_boot.cpp`, `command_dispatch.cpp`, `power_commands.cpp`, `sensing_commands.cpp`, `motion_commands.cpp`.

2. **Decompose `RobotControl` responsibilities**
   - `RobotControl` still owns loop timing, IO coordination, safety interaction, and diagnostics.
   - Suggested split: scheduler/timing, control pipeline, and status reporting modules.

3. **Replace placeholder control policies**
   - `BodyController::update()` is still a pass-through/default output stub.
   - `GaitScheduler::update()` still uses fallback speed magnitude.

4. **Externalize geometry/config defaults**
   - Hardcoded geometry assumptions remain in code rather than versioned config.

## Medium-priority opportunities

1. **Strengthen typed interfaces at module boundaries**
   - Introduce stronger units/types for angles, rates, and timestamps.

2. **Unify logging patterns**
   - Keep diagnostics through logger abstractions and avoid mixed direct stream output.

3. **Targeted tests around protocol and safety transitions**
   - Add malformed frame tests and command-timeout safety behavior tests.

## Suggested execution order

1. Split `hexapod-client.cpp` into focused translation units.
2. Break `RobotControl` into pipeline + scheduling/reporting responsibilities.
3. Replace body-controller and gait-speed placeholders with command-driven logic.
4. Move geometry/calibration defaults into validated configuration.
5. Add focused protocol/safety regression tests.
