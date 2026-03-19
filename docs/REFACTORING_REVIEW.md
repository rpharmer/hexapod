# Refactoring Review

Review date: 2026-03-19

This document summarizes refactoring progress and the next high-impact refactoring targets based on the current repository state.

## Scope reviewed

- `hexapod-common/`: framing/protocol helpers.
- `hexapod-server/`: runtime orchestration, control pipeline, and safety supervisor.
- `hexapod-client/`: firmware boot lifecycle, dispatch loop, and command family handlers.

## Progress snapshot

### Completed / materially improved

1. **Firmware responsibilities are now cleanly separated by concern**
   - bring-up/lifecycle: `firmware_boot.cpp`
   - packet routing + state gatekeeping: `command_dispatch.cpp`
   - command families:
     - power: `power_commands.cpp`
     - sensing: `sensing_commands.cpp`
     - motion/calibration: `motion_commands.cpp`

2. **Server control flow is modularized**
   - `ControlPipeline` encapsulates per-step gait/body/IK + status synthesis.
   - `RobotRuntime` centralizes state, IO, and loop step entry points.
   - `RobotControl` focuses on lifecycle and multi-loop startup/shutdown.

3. **Safety prioritization logic is explicit and auditable**
   - Fault priority and replacement logic are isolated in `SafetySupervisor`.
   - `TIP_OVER` and `COMMAND_TIMEOUT` handling is present and ordered by severity.

## Remaining high-value refactoring targets

1. **Replace placeholders with policy implementations**
   - `BodyController` still returns defaults and does not consume control intent for target placement.
   - `GaitScheduler` still uses fallback speed magnitude instead of command-derived movement intensity.

2. **Improve separations inside runtime orchestration**
   - `RobotRuntime` remains a broad owner of shared state + bus/safety/control/diagnostics step entry points.
   - Next split candidate: isolate step-specific coordinators (bus/control/safety/diagnostics) to reduce class breadth and simplify test seams.

3. **Strengthen domain typing and contracts**
   - Additional strong typing around units and bounds (angles/rates/durations, per-joint limits) would reduce accidental cross-module misuse.

4. **Externalize and validate tunables/config**
   - More runtime constants can move to validated config with schema/version checks and explicit defaults to reduce hard-coded drift.

5. **Backstop future refactors with automated tests**
   - Add tests around framing edge cases, dispatch routing/unsupported commands, and safety transition behavior before deeper structural splits.

## Suggested refactoring sequence

1. Implement body-controller target policy + command-driven gait cadence.
2. Add targeted regression tests for framing/dispatch/safety transitions.
3. Split runtime step ownership into narrower coordinator components.
4. Introduce stronger unit/contract types across estimator → gait → body → IK boundaries.
5. Move tunables to validated config surfaces and document compatibility expectations.
