# Source File Review Matrix (2026-03-26)

This matrix covers every `*.cpp` and `*.hpp` under `hexapod-server/src`, `hexapod-server/include`, and `hexapod-server/tests` (184 files total).

Columns:
- **LOC**: Approximate physical lines.
- **Primary role**: Main responsibility inferred from location/name.
- **Review note / next step**: Concrete follow-up for refactoring, hardening, or testing.

## include/app

| File | LOC | Primary role | Review note / next step |
|---|---:|---|---|
| `hexapod-server/include/app/hexapod-server.hpp` | 115 | App bootstrap/runtime lifecycle | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/include/app/runtime_teardown.hpp` | 15 | App bootstrap/runtime lifecycle | Add observability hooks + invariants for timing/freshness. |
| `hexapod-server/include/app/shutdown_summary.hpp` | 16 | App bootstrap/runtime lifecycle | Keep cohesive; document contracts and extend tests incrementally. |

## include/config

| File | LOC | Primary role | Review note / next step |
|---|---:|---|---|
| `hexapod-server/include/config/calibrations_section_parser.hpp` | 24 | Config parse/validation | Strengthen schema error reporting and table-driven tests. |
| `hexapod-server/include/config/config_validation.hpp` | 84 | Config parse/validation | Strengthen schema error reporting and table-driven tests. |
| `hexapod-server/include/config/geometry_config.hpp` | 80 | Config parse/validation | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/include/config/geometry_section_parser.hpp` | 15 | Config parse/validation | Strengthen schema error reporting and table-driven tests. |
| `hexapod-server/include/config/motor_calibration_validator.hpp` | 23 | Config parse/validation | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/include/config/runtime_section_parser.hpp` | 22 | Config parse/validation | Add observability hooks + invariants for timing/freshness. |
| `hexapod-server/include/config/toml_parser.hpp` | 29 | Config parse/validation | Strengthen schema error reporting and table-driven tests. |
| `hexapod-server/include/config/transport_section_parser.hpp` | 21 | Config parse/validation | Strengthen schema error reporting and table-driven tests. |
| `hexapod-server/include/config/tuning_section_parser.hpp` | 20 | Config parse/validation | Strengthen schema error reporting and table-driven tests. |

## include/control

| File | LOC | Primary role | Review note / next step |
|---|---:|---|---|
| `hexapod-server/include/control/body_controller.hpp` | 35 | Control runtime/pipeline | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/include/control/calibration_probe.hpp` | 125 | Control runtime/pipeline | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/include/control/cli_options.hpp` | 35 | Control runtime/pipeline | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/include/control/contact_manager.hpp` | 37 | Control runtime/pipeline | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/include/control/control_config.hpp` | 195 | Control runtime/pipeline | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/include/control/control_device.hpp` | 32 | Control runtime/pipeline | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/include/control/control_pipeline.hpp` | 34 | Control runtime/pipeline | Add observability hooks + invariants for timing/freshness. |
| `hexapod-server/include/control/estimator.hpp` | 28 | Control runtime/pipeline | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/include/control/foothold_planner.hpp` | 33 | Control runtime/pipeline | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/include/control/freshness_policy.hpp` | 60 | Control runtime/pipeline | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/include/control/gait_policy_planner.hpp` | 102 | Control runtime/pipeline | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/include/control/gait_scheduler.hpp` | 22 | Control runtime/pipeline | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/include/control/geometry_profile_service.hpp` | 17 | Control runtime/pipeline | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/include/control/interactive_calibration_actions.hpp` | 13 | Control runtime/pipeline | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/include/control/interactive_input_mapper.hpp` | 56 | Control runtime/pipeline | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/include/control/joint_oscillation_tracker.hpp` | 40 | Control runtime/pipeline | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/include/control/loop_executor.hpp` | 87 | Control runtime/pipeline | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/include/control/loop_timing.hpp` | 12 | Control runtime/pipeline | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/include/control/mode_runners.hpp` | 29 | Control runtime/pipeline | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/include/control/motion_intent_utils.hpp` | 8 | Control runtime/pipeline | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/include/control/plane_estimation.hpp` | 16 | Control runtime/pipeline | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/include/control/probe_contact_logic.hpp` | 11 | Control runtime/pipeline | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/include/control/robot_control.hpp` | 37 | Control runtime/pipeline | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/include/control/robot_runtime.hpp` | 84 | Control runtime/pipeline | Add observability hooks + invariants for timing/freshness. |
| `hexapod-server/include/control/runtime_diagnostics_reporter.hpp` | 77 | Control runtime/pipeline | Add observability hooks + invariants for timing/freshness. |
| `hexapod-server/include/control/runtime_freshness_gate.hpp` | 49 | Control runtime/pipeline | Add observability hooks + invariants for timing/freshness. |
| `hexapod-server/include/control/runtime_timing_metrics.hpp` | 25 | Control runtime/pipeline | Add observability hooks + invariants for timing/freshness. |
| `hexapod-server/include/control/safety_supervisor.hpp` | 83 | Control runtime/pipeline | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/include/control/servo_dynamics_fit.hpp` | 12 | Control runtime/pipeline | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/include/control/stability_tracker.hpp` | 13 | Control runtime/pipeline | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/include/control/status_reporter.hpp` | 17 | Control runtime/pipeline | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/include/control/telemetry_json.hpp` | 18 | Control runtime/pipeline | Version telemetry schema and add compatibility tests. |
| `hexapod-server/include/control/telemetry_publisher.hpp` | 53 | Control runtime/pipeline | Version telemetry schema and add compatibility tests. |
| `hexapod-server/include/control/touch_residuals.hpp` | 23 | Control runtime/pipeline | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/include/control/visualiser_telemetry.hpp` | 31 | Control runtime/pipeline | Version telemetry schema and add compatibility tests. |

## include/hardware

| File | LOC | Primary role | Review note / next step |
|---|---:|---|---|
| `hexapod-server/include/hardware/bridge_command_api.hpp` | 101 | Hardware bridge/protocol I/O | Clarify protocol-state machine and failure contracts. |
| `hexapod-server/include/hardware/bridge_link_manager.hpp` | 66 | Hardware bridge/protocol I/O | Clarify protocol-state machine and failure contracts. |
| `hexapod-server/include/hardware/command_client.hpp` | 41 | Hardware bridge/protocol I/O | Clarify protocol-state machine and failure contracts. |
| `hexapod-server/include/hardware/handshake_client.hpp` | 29 | Hardware bridge/protocol I/O | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/include/hardware/hardware_bridge.hpp` | 153 | Hardware bridge/protocol I/O | Clarify protocol-state machine and failure contracts. |
| `hexapod-server/include/hardware/hardware_state_codec.hpp` | 14 | Hardware bridge/protocol I/O | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/include/hardware/imu_unit.hpp` | 65 | Hardware bridge/protocol I/O | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/include/hardware/joint_feedback_estimator.hpp` | 21 | Hardware bridge/protocol I/O | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/include/hardware/serialCommsServer.hpp` | 54 | Hardware bridge/protocol I/O | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/include/hardware/sim_hardware_bridge.hpp` | 43 | Hardware bridge/protocol I/O | Clarify protocol-state machine and failure contracts. |
| `hexapod-server/include/hardware/transport_session.hpp` | 56 | Hardware bridge/protocol I/O | Clarify protocol-state machine and failure contracts. |

## include/input

| File | LOC | Primary role | Review note / next step |
|---|---:|---|---|
| `hexapod-server/include/input/controller_event.hpp` | 17 | Input/controller integration | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/include/input/evdev_gamepad_controller.hpp` | 81 | Input/controller integration | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/include/input/event_queue.hpp` | 26 | Input/controller integration | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/include/input/virtual_gamepad_controller.hpp` | 62 | Input/controller integration | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/include/input/xbox_controller.hpp` | 12 | Input/controller integration | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/include/input/xbox_controller_event.hpp` | 4 | Input/controller integration | Keep cohesive; document contracts and extend tests incrementally. |

## include/kinematics

| File | LOC | Primary role | Review note / next step |
|---|---:|---|---|
| `hexapod-server/include/kinematics/leg_fk.hpp` | 19 | Kinematics/math domain | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/include/kinematics/leg_ik.hpp` | 20 | Kinematics/math domain | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/include/kinematics/leg_kinematics_utils.hpp` | 12 | Kinematics/math domain | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/include/kinematics/math_types.hpp` | 189 | Kinematics/math domain | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/include/kinematics/reach_envelope.hpp` | 54 | Kinematics/math domain | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/include/kinematics/types.hpp` | 316 | Kinematics/math domain | Consider splitting into smaller units and add focused tests. |

## include/scenario

| File | LOC | Primary role | Review note / next step |
|---|---:|---|---|
| `hexapod-server/include/scenario/scenario_driver.hpp` | 66 | Scenario execution | Keep cohesive; document contracts and extend tests incrementally. |

## include/utils

| File | LOC | Primary role | Review note / next step |
|---|---:|---|---|
| `hexapod-server/include/utils/double_buffer.hpp` | 36 | Shared utilities/logging | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/include/utils/logger.hpp` | 167 | Shared utilities/logging | Keep cohesive; document contracts and extend tests incrementally. |

## src/app

| File | LOC | Primary role | Review note / next step |
|---|---:|---|---|
| `hexapod-server/src/app/hexapod-server.cpp` | 222 | App bootstrap/runtime lifecycle | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/src/app/runtime_teardown.cpp` | 42 | App bootstrap/runtime lifecycle | Add observability hooks + invariants for timing/freshness. |
| `hexapod-server/src/app/shutdown_summary.cpp` | 28 | App bootstrap/runtime lifecycle | Keep cohesive; document contracts and extend tests incrementally. |

## src/config

| File | LOC | Primary role | Review note / next step |
|---|---:|---|---|
| `hexapod-server/src/config/calibrations_section_parser.cpp` | 24 | Config parse/validation | Strengthen schema error reporting and table-driven tests. |
| `hexapod-server/src/config/config_validation.cpp` | 257 | Config parse/validation | Strengthen schema error reporting and table-driven tests. |
| `hexapod-server/src/config/geometry_config.cpp` | 169 | Config parse/validation | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/src/config/geometry_section_parser.cpp` | 94 | Config parse/validation | Strengthen schema error reporting and table-driven tests. |
| `hexapod-server/src/config/motor_calibration_validator.cpp` | 126 | Config parse/validation | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/src/config/runtime_section_parser.cpp` | 211 | Config parse/validation | Add observability hooks + invariants for timing/freshness. |
| `hexapod-server/src/config/toml_parser.cpp` | 97 | Config parse/validation | Strengthen schema error reporting and table-driven tests. |
| `hexapod-server/src/config/transport_section_parser.cpp` | 48 | Config parse/validation | Strengthen schema error reporting and table-driven tests. |
| `hexapod-server/src/config/tuning_section_parser.cpp` | 195 | Config parse/validation | Strengthen schema error reporting and table-driven tests. |

## src/control

| File | LOC | Primary role | Review note / next step |
|---|---:|---|---|
| `hexapod-server/src/control/calibration/calibration_probe.cpp` | 230 | Control runtime/pipeline | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/src/control/calibration/interactive_calibration_actions.cpp` | 113 | Control runtime/pipeline | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/src/control/calibration/plane_estimation.cpp` | 47 | Control runtime/pipeline | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/src/control/calibration/probe_contact_logic.cpp` | 19 | Control runtime/pipeline | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/src/control/calibration/servo_dynamics_fit.cpp` | 88 | Control runtime/pipeline | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/src/control/calibration/touch_residuals.cpp` | 61 | Control runtime/pipeline | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/src/control/input/cli_options.cpp` | 128 | Control runtime/pipeline | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/src/control/input/interactive_input_mapper.cpp` | 156 | Control runtime/pipeline | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/src/control/input/mode_runners.cpp` | 86 | Control runtime/pipeline | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/src/control/input/motion_intent_utils.cpp` | 23 | Control runtime/pipeline | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/src/control/pipeline/body_controller.cpp` | 140 | Control runtime/pipeline | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/src/control/pipeline/contact_manager.cpp` | 103 | Control runtime/pipeline | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/src/control/pipeline/control_config.cpp` | 105 | Control runtime/pipeline | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/src/control/pipeline/control_pipeline.cpp` | 84 | Control runtime/pipeline | Add observability hooks + invariants for timing/freshness. |
| `hexapod-server/src/control/pipeline/estimator.cpp` | 168 | Control runtime/pipeline | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/src/control/pipeline/foothold_planner.cpp` | 120 | Control runtime/pipeline | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/src/control/pipeline/freshness_policy.cpp` | 116 | Control runtime/pipeline | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/src/control/pipeline/gait_policy_planner.cpp` | 493 | Control runtime/pipeline | Consider splitting into smaller units and add focused tests. |
| `hexapod-server/src/control/pipeline/gait_scheduler.cpp` | 91 | Control runtime/pipeline | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/src/control/pipeline/geometry_profile_service.cpp` | 134 | Control runtime/pipeline | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/src/control/pipeline/robot_control.cpp` | 87 | Control runtime/pipeline | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/src/control/pipeline/stability_tracker.cpp` | 57 | Control runtime/pipeline | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/src/control/runtime/joint_oscillation_tracker.cpp` | 107 | Control runtime/pipeline | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/src/control/runtime/loop_executor.cpp` | 118 | Control runtime/pipeline | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/src/control/runtime/loop_timing.cpp` | 12 | Control runtime/pipeline | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/src/control/runtime/robot_runtime.cpp` | 274 | Control runtime/pipeline | Add observability hooks + invariants for timing/freshness. |
| `hexapod-server/src/control/runtime/runtime_diagnostics_reporter.cpp` | 307 | Control runtime/pipeline | Consider splitting into smaller units and add focused tests. |
| `hexapod-server/src/control/runtime/runtime_freshness_gate.cpp` | 89 | Control runtime/pipeline | Add observability hooks + invariants for timing/freshness. |
| `hexapod-server/src/control/runtime/runtime_timing_metrics.cpp` | 35 | Control runtime/pipeline | Add observability hooks + invariants for timing/freshness. |
| `hexapod-server/src/control/runtime/safety_supervisor.cpp` | 233 | Control runtime/pipeline | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/src/control/runtime/status_reporter.cpp` | 153 | Control runtime/pipeline | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/src/control/telemetry/telemetry_json.cpp` | 80 | Control runtime/pipeline | Version telemetry schema and add compatibility tests. |
| `hexapod-server/src/control/telemetry/telemetry_publisher.cpp` | 264 | Control runtime/pipeline | Version telemetry schema and add compatibility tests. |
| `hexapod-server/src/control/telemetry/visualiser_telemetry.cpp` | 123 | Control runtime/pipeline | Version telemetry schema and add compatibility tests. |

## src/hardware

| File | LOC | Primary role | Review note / next step |
|---|---:|---|---|
| `hexapod-server/src/hardware/bridge_command_api.cpp` | 130 | Hardware bridge/protocol I/O | Clarify protocol-state machine and failure contracts. |
| `hexapod-server/src/hardware/bridge_link_manager.cpp` | 138 | Hardware bridge/protocol I/O | Clarify protocol-state machine and failure contracts. |
| `hexapod-server/src/hardware/command_client.cpp` | 122 | Hardware bridge/protocol I/O | Clarify protocol-state machine and failure contracts. |
| `hexapod-server/src/hardware/handshake_client.cpp` | 90 | Hardware bridge/protocol I/O | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/src/hardware/hardware_bridge.cpp` | 614 | Hardware bridge/protocol I/O | Consider splitting into smaller units and add focused tests. |
| `hexapod-server/src/hardware/hardware_bridge_calibration.cpp` | 72 | Hardware bridge/protocol I/O | Clarify protocol-state machine and failure contracts. |
| `hexapod-server/src/hardware/hardware_bridge_led.cpp` | 44 | Hardware bridge/protocol I/O | Clarify protocol-state machine and failure contracts. |
| `hexapod-server/src/hardware/hardware_bridge_power_sensor.cpp` | 93 | Hardware bridge/protocol I/O | Clarify protocol-state machine and failure contracts. |
| `hexapod-server/src/hardware/hardware_bridge_servo.cpp` | 67 | Hardware bridge/protocol I/O | Clarify protocol-state machine and failure contracts. |
| `hexapod-server/src/hardware/hardware_state_codec.cpp` | 42 | Hardware bridge/protocol I/O | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/src/hardware/imu_unit.cpp` | 242 | Hardware bridge/protocol I/O | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/src/hardware/joint_feedback_estimator.cpp` | 76 | Hardware bridge/protocol I/O | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/src/hardware/serialCommsServer.cpp` | 152 | Hardware bridge/protocol I/O | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/src/hardware/sim_hardware_bridge.cpp` | 75 | Hardware bridge/protocol I/O | Clarify protocol-state machine and failure contracts. |
| `hexapod-server/src/hardware/transport_session.cpp` | 104 | Hardware bridge/protocol I/O | Clarify protocol-state machine and failure contracts. |

## src/input

| File | LOC | Primary role | Review note / next step |
|---|---:|---|---|
| `hexapod-server/src/input/evdev_gamepad_controller.cpp` | 262 | Input/controller integration | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/src/input/virtual_gamepad_controller.cpp` | 204 | Input/controller integration | Keep cohesive; document contracts and extend tests incrementally. |

## src/kinematics

| File | LOC | Primary role | Review note / next step |
|---|---:|---|---|
| `hexapod-server/src/kinematics/leg_fk.cpp` | 99 | Kinematics/math domain | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/src/kinematics/leg_ik.cpp` | 100 | Kinematics/math domain | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/src/kinematics/leg_kinematics_utils.cpp` | 33 | Kinematics/math domain | Keep cohesive; document contracts and extend tests incrementally. |
| `hexapod-server/src/kinematics/types.cpp` | 356 | Kinematics/math domain | Consider splitting into smaller units and add focused tests. |

## src/scenario

| File | LOC | Primary role | Review note / next step |
|---|---:|---|---|
| `hexapod-server/src/scenario/scenario_driver.cpp` | 310 | Scenario execution | Consider splitting into smaller units and add focused tests. |

## src/utils

| File | LOC | Primary role | Review note / next step |
|---|---:|---|---|
| `hexapod-server/src/utils/logger.cpp` | 298 | Shared utilities/logging | Keep cohesive; document contracts and extend tests incrementally. |

## tests

| File | LOC | Primary role | Review note / next step |
|---|---:|---|---|
| `hexapod-server/tests/test_body_controller_velocity.cpp` | 159 | Unit/integration test coverage | Add boundary + failure-mode assertions where missing. |
| `hexapod-server/tests/test_bridge_command_api.cpp` | 124 | Unit/integration test coverage | Add boundary + failure-mode assertions where missing. |
| `hexapod-server/tests/test_bridge_link_manager.cpp` | 131 | Unit/integration test coverage | Add boundary + failure-mode assertions where missing. |
| `hexapod-server/tests/test_calibration_probe_fit.cpp` | 472 | Unit/integration test coverage | Add boundary + failure-mode assertions where missing. |
| `hexapod-server/tests/test_cli_mode_selection.cpp` | 382 | Unit/integration test coverage | Add boundary + failure-mode assertions where missing. |
| `hexapod-server/tests/test_command_flow_integration.cpp` | 295 | Unit/integration test coverage | Add boundary + failure-mode assertions where missing. |
| `hexapod-server/tests/test_contact_manager.cpp` | 74 | Unit/integration test coverage | Add boundary + failure-mode assertions where missing. |
| `hexapod-server/tests/test_control_pipeline_sanity.cpp` | 69 | Unit/integration test coverage | Add boundary + failure-mode assertions where missing. |
| `hexapod-server/tests/test_freshness_policy.cpp` | 157 | Unit/integration test coverage | Add boundary + failure-mode assertions where missing. |
| `hexapod-server/tests/test_gait_dynamics_turn_safety.cpp` | 201 | Unit/integration test coverage | Add boundary + failure-mode assertions where missing. |
| `hexapod-server/tests/test_gait_policy_planner.cpp` | 200 | Unit/integration test coverage | Add boundary + failure-mode assertions where missing. |
| `hexapod-server/tests/test_geometry_profile_service.cpp` | 166 | Unit/integration test coverage | Add boundary + failure-mode assertions where missing. |
| `hexapod-server/tests/test_hardware_bridge_transport_capabilities.cpp` | 249 | Unit/integration test coverage | Add boundary + failure-mode assertions where missing. |
| `hexapod-server/tests/test_hardware_bridge_transport_commands_calibration.cpp` | 122 | Unit/integration test coverage | Add boundary + failure-mode assertions where missing. |
| `hexapod-server/tests/test_hardware_bridge_transport_commands_led.cpp` | 110 | Unit/integration test coverage | Add boundary + failure-mode assertions where missing. |
| `hexapod-server/tests/test_hardware_bridge_transport_commands_power.cpp` | 102 | Unit/integration test coverage | Add boundary + failure-mode assertions where missing. |
| `hexapod-server/tests/test_hardware_bridge_transport_commands_sensor.cpp` | 114 | Unit/integration test coverage | Add boundary + failure-mode assertions where missing. |
| `hexapod-server/tests/test_hardware_bridge_transport_commands_servo.cpp` | 213 | Unit/integration test coverage | Add boundary + failure-mode assertions where missing. |
| `hexapod-server/tests/test_hardware_bridge_transport_failures.cpp` | 334 | Unit/integration test coverage | Add boundary + failure-mode assertions where missing. |
| `hexapod-server/tests/test_hardware_bridge_transport_handshake.cpp` | 66 | Unit/integration test coverage | Add boundary + failure-mode assertions where missing. |
| `hexapod-server/tests/test_hardware_bridge_transport_main.cpp` | 21 | Unit/integration test coverage | Add boundary + failure-mode assertions where missing. |
| `hexapod-server/tests/test_imu_units.cpp` | 61 | Unit/integration test coverage | Add boundary + failure-mode assertions where missing. |
| `hexapod-server/tests/test_interactive_input_mapper.cpp` | 96 | Unit/integration test coverage | Add boundary + failure-mode assertions where missing. |
| `hexapod-server/tests/test_joint_feedback_estimator.cpp` | 68 | Unit/integration test coverage | Add boundary + failure-mode assertions where missing. |
| `hexapod-server/tests/test_loop_executor_timing.cpp` | 311 | Unit/integration test coverage | Add boundary + failure-mode assertions where missing. |
| `hexapod-server/tests/test_motion_intent_through_ik_fk.cpp` | 264 | Unit/integration test coverage | Add boundary + failure-mode assertions where missing. |
| `hexapod-server/tests/test_protocol_command_metadata.cpp` | 126 | Unit/integration test coverage | Add boundary + failure-mode assertions where missing. |
| `hexapod-server/tests/test_robot_runtime_freshness_gate_matrix.cpp` | 181 | Unit/integration test coverage | Add boundary + failure-mode assertions where missing. |
| `hexapod-server/tests/test_robot_runtime_loop.cpp` | 347 | Unit/integration test coverage | Add boundary + failure-mode assertions where missing. |
| `hexapod-server/tests/test_runtime_helpers.cpp` | 263 | Unit/integration test coverage | Add boundary + failure-mode assertions where missing. |
| `hexapod-server/tests/test_runtime_teardown_logging.cpp` | 116 | Unit/integration test coverage | Add boundary + failure-mode assertions where missing. |
| `hexapod-server/tests/test_safety_supervisor_faults.cpp` | 276 | Unit/integration test coverage | Add boundary + failure-mode assertions where missing. |
| `hexapod-server/tests/test_scenario_driver_validation.cpp` | 322 | Unit/integration test coverage | Add boundary + failure-mode assertions where missing. |
| `hexapod-server/tests/test_servo_calibration_unified_conversion.cpp` | 79 | Unit/integration test coverage | Add boundary + failure-mode assertions where missing. |
| `hexapod-server/tests/test_shutdown_summary_logging.cpp` | 114 | Unit/integration test coverage | Add boundary + failure-mode assertions where missing. |
| `hexapod-server/tests/test_simple_estimator_ground_plane.cpp` | 75 | Unit/integration test coverage | Add boundary + failure-mode assertions where missing. |
| `hexapod-server/tests/test_stability_tracker.cpp` | 58 | Unit/integration test coverage | Add boundary + failure-mode assertions where missing. |
| `hexapod-server/tests/test_telemetry_json_serialization.cpp` | 130 | Unit/integration test coverage | Add boundary + failure-mode assertions where missing. |
| `hexapod-server/tests/test_toml_parser_sections.cpp` | 482 | Unit/integration test coverage | Add boundary + failure-mode assertions where missing. |
| `hexapod-server/tests/test_transport_components.cpp` | 199 | Unit/integration test coverage | Add boundary + failure-mode assertions where missing. |
| `hexapod-server/tests/test_virtual_gamepad_controller.cpp` | 151 | Unit/integration test coverage | Add boundary + failure-mode assertions where missing. |
| `hexapod-server/tests/test_visualiser_telemetry.cpp` | 75 | Unit/integration test coverage | Add boundary + failure-mode assertions where missing. |

