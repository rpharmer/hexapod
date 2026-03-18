# Refactoring Review

This review summarizes refactoring progress and remaining high-impact work based on the current repository state.

## Scope reviewed

- `hexapod-common/` framing and protocol helpers
- `hexapod-server/` loop orchestration + control pipeline modules
- `hexapod-client/` firmware startup, command dispatch, and command handlers

## Progress snapshot

### Completed / materially improved

1. **Firmware monolith split is complete at file-responsibility level**
   - Boot/shutdown responsibilities: `firmware_boot.cpp`
   - Packet routing and host-state gatekeeping: `command_dispatch.cpp`
   - Domain handlers:
     - power: `power_commands.cpp`
     - sensing: `sensing_commands.cpp`
     - motion/calibration: `motion_commands.cpp`

2. **Server pipeline modularity improved**
   - `ControlPipeline` now groups gait/body/IK execution and status synthesis.
   - `StatusReporter` isolates diagnostics formatting/output concerns.
   - Control-loop timing utilities are centralized in `loop_timing`.

3. **State modeling is clearer on firmware**
   - Explicit lifecycle states (`BOOT`, `WAITING_FOR_HOST`, `ACTIVE`, `STOPPING`, `OFF`) improve readability and future testability of transitions.

## Remaining high-value refactoring targets

1. **Replace behavior placeholders with policy implementations**
   - `BodyController` still returns defaults and does not consume control inputs.
   - `GaitScheduler` still uses fallback speed magnitude instead of command-derived motion intensity.

2. **Further decompose `RobotControl` runtime ownership**
   - `RobotControl` still owns thread lifecycle, loop bodies, and shared-state plumbing in one class.
   - Next split candidate: extract loop runners/schedulers from lifecycle orchestration.

3. **Tighten boundary contracts and unit semantics**
   - Add stronger types/wrappers for angles, rates, and durations to reduce cross-module unit mistakes.

4. **Move tunables toward validated config**
   - Runtime geometry/control constants should be externalized where practical, with schema/version validation and defaults.

5. **Backstop refactors with regression tests**
   - Add focused tests for framing decode/encode errors, command routing edges, and safety transition behavior before larger structural changes.

## Suggested refactoring sequence

1. Implement body controller and command-driven gait cadence.
2. Add regression coverage for protocol + safety transitions.
3. Split `RobotControl` into orchestration + loop-runner components.
4. Introduce stronger unit-typed interfaces across estimator/gait/IK paths.
5. Externalize/configure tunables with validation and compatibility checks.
