# Extending Input Devices and Hardware Backends

This guide explains how to wire in:

1. a new **interactive input device** (controller/teleop source), and
2. a new **hardware bridge backend** (anything other than current `serial` and `sim`).

It is focused on `hexapod-server` internals.

---

## Runtime architecture (where extension points exist)

At startup, `hexapod-server` does the following:

1. Parse `config.txt` into `ParsedToml`.
2. Create a hardware bridge (`IHardwareBridge`) in `makeHardwareBridge(...)`.
3. Build `RobotControl` with that bridge.
4. Run either:
   - **scenario mode** (no controller), or
   - **interactive mode** (optional `IControlDevice` implementation).

Relevant code:

- App boot + bridge selection: `src/app/hexapod-server.cpp`
- Interactive runner + controller creation: `src/control/mode_runners.cpp`
- Controller interface: `include/control/control_device.hpp`
- Hardware bridge interface: `include/hardware/hardware_bridge.hpp`

---

## A) Adding a new input device

## 1) Implement `IControlDevice`

Create a new class in `include/input/` + `src/input/` that implements:

- lifecycle: `start()`, `stop()`
- event source: `EventQueue<ControllerEvent>& getQueue()`
- analog state getters (`getLeftX/Y/Mag/Ang`, `getRightX/Y/Mag/Ang`)
- trigger getters (`getLeftTrigger()`, `getRightTrigger()`)
- deadzone setter (`setRadialDeadzone(...)`)

Use these existing classes as references:

- `EvdevGamepadController` (real Linux input device)
- `VirtualGamepadController` (test/synthetic input)

### Contract notes

- `InteractiveRunner` drains the queue every command refresh cycle.
- Button events must use names expected by the mapper (`A`, `B`, `X`, `Y`, `LB`, `RB`, etc.).
- Axis semantics should remain aligned with current mapper assumptions:
  - Left stick: heading/speed in walk mode
  - Right stick: yaw rate in walk mode, roll/pitch in body-pose mode
  - Triggers: height or yaw, depending on mode

If your physical device uses different button/axis codes, add a mapping layer (similar to `makeXboxGamepadMapping()` / `makeGenericGamepadMapping()`).

## 2) Wire device creation in interactive mode

`InteractiveRunner::run(...)` currently builds an `EvdevGamepadController` when `--controller-device` is supplied.

To add a new device type, update this runner to choose your class based on a CLI flag, config key, or auto-detect strategy.

Typical pattern:

- extend `CliOptions` with input driver selector (for example `--controller-driver evdev|mydriver`)
- branch in `InteractiveRunner::run(...)`
- keep fallback behavior (log warning + fallback command loop) when device start fails

## 3) Update CLI parsing

If adding flags, update:

- `include/control/cli_options.hpp`
- `src/control/cli_options.cpp`

and document the new usage in `hexapod-server/README.md`.

## 4) Add tests

Recommended:

- unit tests for your event normalization/mapping
- unit tests for queue behavior under bursts
- integration-style tests similar to `tests/test_virtual_gamepad_controller.cpp` and `tests/test_interactive_input_mapper.cpp`

---

## B) Adding a new hardware backend

A hardware backend is any implementation of `IHardwareBridge`.

Current backends:

- `SimpleHardwareBridge` (`serial` runtime)
- `SimHardwareBridge` (`sim` runtime)

## 1) Implement `IHardwareBridge`

Create a new class implementing:

- `bool init()`
- `bool read(RobotState& out)`
- `bool write(const JointTargets& in)`

Design expectations:

- `read(...)` should return a coherent `RobotState` snapshot each call.
- `write(...)` should consume all 18 joint targets.
- calls must be safe for real-time loop usage (avoid unbounded blocking).

Tip: follow `SimHardwareBridge` if your transport is not packet/protocol based, and `SimpleHardwareBridge` if it is request/response based.

## 2) Add runtime-mode selection

Backend choice is currently done in `makeHardwareBridge(...)` with `Runtime.Mode` values:

- `serial`
- `sim`

To add another mode (example: `canfd`, `udp`, `spi`):

1. Extend accepted values in `runtime_section_parser.cpp`.
2. Add any new parsed config fields to `ParsedToml` (`include/app/hexapod-server.hpp`) and parser modules.
3. Branch in `makeHardwareBridge(...)` to instantiate your bridge.
4. Preserve existing behavior for `serial` and `sim`.

## 3) Add config schema/docs

Update config documentation (and defaults) so operators know:

- new `Runtime.Mode` value
- required connection parameters
- timeout/retry semantics
- any capability or feature limitations

## 4) Add tests for link and failure behavior

Recommended coverage:

- init success/failure
- read/write success path
- timeouts/disconnects/reconnect behavior
- malformed data handling
- deterministic safety behavior when bridge read/write fails

Useful references:

- transport/link tests under `hexapod-server/tests/` (`test_hardware_bridge_transport.cpp`, `test_bridge_link_manager.cpp`, etc.)
- scenario mode for integration validation in sim-like environments

---

## C) Wiring checklist (quick)

When adding either input or hardware extensions, verify all of these:

- [ ] Interface implementation compiles (`IControlDevice` or `IHardwareBridge`)
- [ ] Construction path is reachable from app startup/interactive runner
- [ ] CLI/config parser supports selecting the new implementation
- [ ] Logging clearly states which driver/backend is active
- [ ] Failure path degrades safely (fallback or stop with explicit error)
- [ ] Unit/integration tests added for normal + fault paths
- [ ] README/docs updated for operators

---

## D) Suggested implementation sequence

1. Add class + minimal tests.
2. Add selection wiring (CLI/config + factory branch).
3. Verify local run in the smallest possible loop.
4. Add fault-path tests.
5. Update docs and examples last.

This order keeps behavior testable and avoids large, hard-to-debug integration jumps.
