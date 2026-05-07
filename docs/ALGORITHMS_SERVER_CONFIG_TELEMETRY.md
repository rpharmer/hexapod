# `hexapod-server` Config and Telemetry Mapping

This document maps control-related configuration fields to runtime consumers and telemetry output surfaces.

## 1) Config ingestion path

Primary parser: `hexapod-server/src/control/control_config.cpp`.

Parser output object: `ControlConfig`.

Representative mapped groups:

- Loop timing (`loop_timing.*`)
- Safety (`safety.*`)
- Freshness policy (`freshness.*`)
- Gait and locomotion command settings (`gait.*`, `locomotion_cmd.*`)
- Command governor (`command_governor.*`)
- Navigation and local planner (`local_map.*`, `local_planner.*`, `nav_bridge.*`)
- Fusion and terrain behavior (`fusion.*`, `foot_terrain.*`)
- Telemetry and replay (`telemetry.*`, `replay_log.*`)

## 2) Runtime consumers (algorithmic)

Primary runtime owner: `hexapod-server/src/control/robot_runtime.cpp`.

### Used directly in runtime orchestration

- Loop timing drives scheduler periods.
- Freshness policy configures `RuntimeFreshnessGate`.
- Safety config constructs `SafetySupervisor`.
- Telemetry/replay config controls publish and persistence cadence.

### Used in control modules

- Gait, locomotion-command, and foot-terrain config are injected into `ControlPipeline`.
- Navigation/local map/planner/nav bridge config are consumed in `NavigationManager` and `NavLocomotionBridge`.

### Caveat: governor configuration wiring

- `ControlConfig` parses `command_governor` values.
- `ControlPipeline` currently default-constructs `CommandGovernor` instead of injecting parsed governor config.
- Result: runtime behavior may not reflect tuned governor values from config unless wiring is updated.

## 3) Telemetry surface mapping

Primary serializer: `hexapod-server/src/control/telemetry_json.cpp`.
Publish path: `RobotRuntime::maybePublishTelemetry()` in `robot_runtime.cpp`.

Control-step telemetry packet includes:

- `status` (`active_mode`, `active_fault`, bus/estimator state)
- `joint_targets`
- locomotion debug fields
- `governor` snapshot (`CommandGovernorState`)
- `navigation` monitor snapshot (when available)
- `fusion` diagnostics and correction payload
- process and section resource metrics

This is the primary observability path for verifying supervisory decisions and command shaping.

## 4) Recommended debug workflow

When diagnosing motion behavior:

1. Verify freshness acceptance/rejection path in control logs and status packet.
2. Check `safety` fault transitions and lifecycle state.
3. Inspect `governor` severity/reason mask and command scale.
4. Compare navigation lifecycle/blocked state with commanded motion.
5. Correlate fusion trust/residuals with governor pressure terms.

## 5) Known documentation mismatches to keep explicit

- README control-pipeline order is stale relative to current `ControlPipeline`.
- Parsed governor tuning is not clearly wired into active governor construction path.
- Supervisor terminology can be ambiguous unless docs distinguish safety supervisor vs freshness gate.
