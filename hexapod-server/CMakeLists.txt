cmake_minimum_required(VERSION 3.16)
include(FetchContent)

project(hexapod-server LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(HEXAPOD_SERVER_BUILD_TESTS "Build hexapod-server unit tests" OFF)

# Disable third-party tests when fetching dependencies.
set(BUILD_TESTING OFF CACHE BOOL "Disable CTest-based tests" FORCE)
set(BUILD_TESTS OFF CACHE BOOL "Disable project-specific tests" FORCE)

find_package(CppLinuxSerial CONFIG QUIET)

if(TARGET CppLinuxSerial AND NOT TARGET CppLinuxSerial::CppLinuxSerial)
    add_library(CppLinuxSerial::CppLinuxSerial ALIAS CppLinuxSerial)
endif()

if(NOT TARGET CppLinuxSerial::CppLinuxSerial)
    find_path(CPPLINUXSERIAL_INCLUDE_DIR
        NAMES CppLinuxSerial/SerialPort.hpp
    )
    find_library(CPPLINUXSERIAL_LIBRARY
        NAMES CppLinuxSerial cpplinuxserial
    )

    include(FindPackageHandleStandardArgs)
    find_package_handle_standard_args(CppLinuxSerial
        REQUIRED_VARS CPPLINUXSERIAL_INCLUDE_DIR CPPLINUXSERIAL_LIBRARY
    )

    if(NOT CppLinuxSerial_FOUND)
        message(FATAL_ERROR "CppLinuxSerial not found. Install it system-wide (for example via the setup script) or provide CppLinuxSerial_DIR.")
    endif()

    add_library(CppLinuxSerial::CppLinuxSerial UNKNOWN IMPORTED)
    set_target_properties(CppLinuxSerial::CppLinuxSerial PROPERTIES
        IMPORTED_LOCATION "${CPPLINUXSERIAL_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${CPPLINUXSERIAL_INCLUDE_DIR}"
    )
endif()

find_package(toml11 REQUIRED)

# Some CppLinuxSerial packages export a non-existent "/CppLinuxSerial" include
# directory in INTERFACE_INCLUDE_DIRECTORIES, which causes configuration to fail
# before build begins. Strip any include entries that do not exist so the imported
# target remains usable across environments.
get_target_property(CPPLINUXSERIAL_INTERFACE_INCLUDES CppLinuxSerial::CppLinuxSerial INTERFACE_INCLUDE_DIRECTORIES)
if(CPPLINUXSERIAL_INTERFACE_INCLUDES)
    set(CPPLINUXSERIAL_VALID_INCLUDES "")
    foreach(CPPLINUXSERIAL_INCLUDE_DIR IN LISTS CPPLINUXSERIAL_INTERFACE_INCLUDES)
        if(EXISTS "${CPPLINUXSERIAL_INCLUDE_DIR}")
            list(APPEND CPPLINUXSERIAL_VALID_INCLUDES "${CPPLINUXSERIAL_INCLUDE_DIR}")
        endif()
    endforeach()

    if(CPPLINUXSERIAL_VALID_INCLUDES)
        set_property(TARGET CppLinuxSerial::CppLinuxSerial PROPERTY INTERFACE_INCLUDE_DIRECTORIES "${CPPLINUXSERIAL_VALID_INCLUDES}")
    endif()
endif()

set(HEXAPOD_SERVER_MAIN_SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/src/app/hexapod-server.cpp")

# Keep this list explicit so target composition is deterministic and reviewable.
# When adding a new src/*.cpp file, append it to the subsystem block below and
# keep the entry sorted within that block.
set(HEXAPOD_SERVER_SOURCES
    # app
    "${CMAKE_CURRENT_SOURCE_DIR}/src/app/runtime_teardown.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/app/shutdown_summary.cpp"

    # autonomy
    "${CMAKE_CURRENT_SOURCE_DIR}/src/autonomy/contract_enforcer.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/autonomy/mission_executive.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/autonomy/motion_arbiter.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/autonomy/navigation_manager.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/autonomy/progress_monitor.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/autonomy/recovery_manager.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/autonomy/mission_scripting.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/autonomy/modules/global_planner.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/autonomy/modules/global_planner_algorithms.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/autonomy/modules/autonomy_stack.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/autonomy/modules/local_planner.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/autonomy/modules/localization.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/autonomy/modules/locomotion_interface.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/autonomy/modules/motion_arbiter.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/autonomy/modules/navigation_manager.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/autonomy/modules/progress_monitor.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/autonomy/modules/recovery_manager.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/autonomy/modules/traversability_analyzer.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/autonomy/modules/world_model.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/autonomy/module_stubs.cpp"

    # config
    "${CMAKE_CURRENT_SOURCE_DIR}/src/config/calibrations_section_parser.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/config/config_validation.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/config/geometry_config.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/config/geometry_section_parser.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/config/motor_calibration_validator.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/config/runtime_section_parser.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/config/toml_parser.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/config/transport_section_parser.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/config/tuning_section_parser.cpp"

    # control
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/pipeline/body_controller.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/calibration/calibration_probe.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/input/cli_options.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/pipeline/control_config.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/pipeline/control_pipeline.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/pipeline/contact_manager.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/pipeline/estimator.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/pipeline/freshness_policy.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/pipeline/foothold_planner.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/pipeline/gait_policy_planner.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/pipeline/gait_scheduler.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/pipeline/geometry_profile_service.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/calibration/interactive_calibration_actions.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/input/interactive_input_mapper.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/runtime/joint_oscillation_tracker.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/runtime/loop_executor.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/runtime/loop_timing.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/input/mode_runners.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/pipeline/motion_limiter.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/input/motion_intent_utils.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/calibration/plane_estimation.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/calibration/probe_contact_logic.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/pipeline/robot_control.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/pipeline/stability_tracker.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/runtime/robot_runtime.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/runtime/runtime_diagnostics_reporter.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/runtime/runtime_freshness_gate.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/runtime/runtime_timing_metrics.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/runtime/safety_supervisor.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/calibration/servo_dynamics_fit.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/runtime/status_reporter.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/telemetry/telemetry_json.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/telemetry/telemetry_publisher.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/calibration/touch_residuals.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/telemetry/visualiser_telemetry.cpp"

    # hardware
    "${CMAKE_CURRENT_SOURCE_DIR}/src/hardware/bridge_command_api.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/hardware/bridge_link_manager.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/hardware/command_client.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/hardware/handshake_client.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/hardware/hardware_bridge.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/hardware/hardware_bridge_commands.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/hardware/hardware_bridge_metadata.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/hardware/hardware_state_codec.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/hardware/imu_unit.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/hardware/joint_feedback_estimator.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/hardware/serialCommsServer.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/hardware/sim_hardware_bridge.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/hardware/transport_session.cpp"

    # input
    "${CMAKE_CURRENT_SOURCE_DIR}/src/input/evdev_gamepad_controller.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/input/virtual_gamepad_controller.cpp"

    # kinematics
    "${CMAKE_CURRENT_SOURCE_DIR}/src/kinematics/leg_fk.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/kinematics/leg_ik.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/kinematics/leg_kinematics_utils.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/kinematics/types.cpp"

    # scenario
    "${CMAKE_CURRENT_SOURCE_DIR}/src/scenario/scenario_driver.cpp"

    # utils
    "${CMAKE_CURRENT_SOURCE_DIR}/src/utils/logger.cpp"
)

add_library(hexapod_server_core
    ${HEXAPOD_SERVER_SOURCES}
    ../hexapod-common/framing.cpp
)

target_include_directories(hexapod_server_core
    PUBLIC
        include
        include/app
        include/config
        include/control
        include/hardware
        include/input
        include/kinematics
        include/scenario
        include/utils
        ../hexapod-common/include
)

if(MSVC)
    target_compile_options(hexapod_server_core PRIVATE /W4 /WX)
else()
    target_compile_options(hexapod_server_core PRIVATE -pedantic-errors -Wall -Wextra -Werror -pthread)
endif()

target_link_libraries(hexapod_server_core
    PUBLIC
        CppLinuxSerial::CppLinuxSerial
        toml11::toml11
)

add_executable(hexapod-server
    ${HEXAPOD_SERVER_MAIN_SOURCE}
)

target_link_libraries(hexapod-server
    PRIVATE
        hexapod_server_core
)

function(hexapod_server_add_test test_target test_name)
    add_executable(${test_target} ${ARGN})
    target_link_libraries(${test_target} PRIVATE hexapod_server_core)
    if(NOT MSVC)
        target_link_options(${test_target} PRIVATE -pthread)
    endif()
    add_test(NAME ${test_name} COMMAND ${test_target})
endfunction()

if(HEXAPOD_SERVER_BUILD_TESTS)
    enable_testing()

    # autonomy
    hexapod_server_add_test(test_autonomy_contract_enforcer autonomy_contract_enforcer
        tests/test_autonomy_contract_enforcer.cpp
    )
    hexapod_server_add_test(test_autonomy_module_stubs autonomy_module_stubs
        tests/test_autonomy_module_stubs.cpp
    )
    hexapod_server_add_test(test_autonomy_module_shells autonomy_module_shells
        tests/test_autonomy_module_shells.cpp
    )
    hexapod_server_add_test(test_autonomy_planner_regression autonomy_planner_regression
        tests/test_autonomy_planner_regression.cpp
    )
    hexapod_server_add_test(test_autonomy_stack autonomy_stack
        tests/test_autonomy_stack.cpp
    )
    hexapod_server_add_test(test_autonomy_supervision_integration autonomy_supervision_integration
        tests/test_autonomy_supervision_integration.cpp
    )
    hexapod_server_add_test(test_mission_executive mission_executive
        tests/test_mission_executive.cpp
    )
    hexapod_server_add_test(test_mission_scripting mission_scripting
        tests/test_mission_scripting.cpp
    )
    hexapod_server_add_test(test_motion_arbiter motion_arbiter
        tests/test_motion_arbiter.cpp
    )
    hexapod_server_add_test(test_navigation_manager navigation_manager
        tests/test_navigation_manager.cpp
    )
    hexapod_server_add_test(test_progress_recovery progress_recovery
        tests/test_progress_recovery.cpp
    )

    # runtime
    hexapod_server_add_test(test_robot_runtime_loop robot_runtime_loop_behavior
        tests/test_robot_runtime_loop.cpp
    )
    hexapod_server_add_test(test_robot_runtime_autonomy_integration robot_runtime_autonomy_integration
        tests/test_robot_runtime_autonomy_integration.cpp
    )
    hexapod_server_add_test(test_safety_supervisor_faults safety_supervisor_transitions
        tests/test_safety_supervisor_faults.cpp
    )
    hexapod_server_add_test(test_control_pipeline_sanity control_pipeline_output_sanity
        tests/test_control_pipeline_sanity.cpp
    )
    hexapod_server_add_test(test_hardware_bridge_transport hardware_bridge_transport_behavior
        tests/test_hardware_bridge_transport_main.cpp
        tests/test_hardware_bridge_transport_handshake.cpp
        tests/test_hardware_bridge_transport_failures.cpp
        tests/test_hardware_bridge_transport_commands_power.cpp
        tests/test_hardware_bridge_transport_commands_sensor.cpp
        tests/test_hardware_bridge_transport_commands_servo.cpp
        tests/test_hardware_bridge_transport_commands_led.cpp
        tests/test_hardware_bridge_transport_commands_calibration.cpp
        tests/test_hardware_bridge_transport_capabilities.cpp
    )

    # calibration
    hexapod_server_add_test(test_servo_calibration_unified_conversion servo_calibration_unified_conversion
        tests/test_servo_calibration_unified_conversion.cpp
    )
    hexapod_server_add_test(test_calibration_probe_fit calibration_probe_fit
        tests/test_calibration_probe_fit.cpp
    )

    # kinematics
    hexapod_server_add_test(test_motion_intent_through_ik_fk motion_intent_through_ik_fk_chain
        tests/test_motion_intent_through_ik_fk.cpp
    )
    hexapod_server_add_test(test_randomized_geometry_control_updates randomized_geometry_control_updates
        tests/test_randomized_geometry_control_updates.cpp
    )

    hexapod_server_add_test(test_motion_limiter_behavior motion_limiter_behavior
        tests/test_motion_limiter_behavior.cpp
    )

    # input
    hexapod_server_add_test(test_virtual_gamepad_controller virtual_gamepad_controller
        tests/test_virtual_gamepad_controller.cpp
    )
    hexapod_server_add_test(test_cli_mode_selection cli_mode_selection
        tests/test_cli_mode_selection.cpp
    )
    hexapod_server_add_test(test_interactive_input_mapper interactive_input_mapper
        tests/test_interactive_input_mapper.cpp
    )

    # scenario
    hexapod_server_add_test(test_scenario_driver_validation scenario_driver_validation
        tests/test_scenario_driver_validation.cpp
    )
    hexapod_server_add_test(test_scenario_driver_refresh_freshness scenario_driver_refresh_freshness
        tests/test_scenario_driver_refresh_freshness.cpp
    )
    hexapod_server_add_test(test_scenario_autonomy_outcomes scenario_autonomy_outcomes
        tests/test_scenario_autonomy_outcomes.cpp
    )
    hexapod_server_add_test(test_hardware_hil_acceptance hardware_hil_acceptance
        tests/test_hardware_hil_acceptance.cpp
    )

    # control
    hexapod_server_add_test(test_simple_estimator_ground_plane simple_estimator_ground_plane
        tests/test_simple_estimator_ground_plane.cpp
    )
    hexapod_server_add_test(test_body_controller_velocity body_controller_velocity
        tests/test_body_controller_velocity.cpp
    )
    hexapod_server_add_test(test_gait_plausibility_flat_plane gait_plausibility_flat_plane
        tests/test_gait_plausibility_flat_plane.cpp
    )
    hexapod_server_add_test(test_contact_manager contact_manager
        tests/test_contact_manager.cpp
    )
    hexapod_server_add_test(test_gait_policy_planner gait_policy_planner
        tests/test_gait_policy_planner.cpp
    )
    hexapod_server_add_test(test_gait_dynamics_turn_safety gait_dynamics_turn_safety
        tests/test_gait_dynamics_turn_safety.cpp
    )
    hexapod_server_add_test(test_stability_tracker stability_tracker
        tests/test_stability_tracker.cpp
    )
    hexapod_server_add_test(test_freshness_policy freshness_policy
        tests/test_freshness_policy.cpp
    )
    hexapod_server_add_test(test_robot_runtime_freshness_gate_matrix robot_runtime_freshness_gate_matrix
        tests/test_robot_runtime_freshness_gate_matrix.cpp
    )
    hexapod_server_add_test(test_runtime_helpers runtime_helpers
        tests/test_runtime_helpers.cpp
    )
    hexapod_server_add_test(test_visualiser_telemetry visualiser_telemetry
        tests/test_visualiser_telemetry.cpp
    )
    hexapod_server_add_test(test_shutdown_summary_logging shutdown_summary_logging
        tests/test_shutdown_summary_logging.cpp
    )
    hexapod_server_add_test(test_runtime_teardown_logging runtime_teardown_logging
        tests/test_runtime_teardown_logging.cpp
    )
    hexapod_server_add_test(test_geometry_profile_service geometry_profile_service
        tests/test_geometry_profile_service.cpp
    )
    hexapod_server_add_test(test_telemetry_json_serialization telemetry_json_serialization
        tests/test_telemetry_json_serialization.cpp
    )
    hexapod_server_add_test(test_telemetry_publisher_fallback telemetry_publisher_fallback
        tests/test_telemetry_publisher_fallback.cpp
    )
    hexapod_server_add_test(test_telemetry_publisher_autonomy_debug telemetry_publisher_autonomy_debug
        tests/test_telemetry_publisher_autonomy_debug.cpp
    )
    hexapod_server_add_test(test_loop_executor_timing loop_executor_timing
        tests/test_loop_executor_timing.cpp
    )
    # config
    hexapod_server_add_test(test_toml_parser_sections toml_parser_sections
        tests/test_toml_parser_sections.cpp
    )

    # transport
    hexapod_server_add_test(test_transport_components transport_components
        tests/test_transport_components.cpp
    )
    hexapod_server_add_test(test_protocol_command_metadata protocol_command_metadata
        tests/test_protocol_command_metadata.cpp
    )

    # hardware
    hexapod_server_add_test(test_imu_units imu_units
        tests/test_imu_units.cpp
    )
    hexapod_server_add_test(test_bridge_link_manager bridge_link_manager
        tests/test_bridge_link_manager.cpp
    )
    hexapod_server_add_test(test_bridge_command_api bridge_command_api
        tests/test_bridge_command_api.cpp
    )
    hexapod_server_add_test(test_joint_feedback_estimator joint_feedback_estimator
        tests/test_joint_feedback_estimator.cpp
    )
    hexapod_server_add_test(test_command_flow_integration command_flow_integration
        tests/test_command_flow_integration.cpp
    )

    # diagnostics
    hexapod_server_add_test(test_runtime_diagnostics_reporter runtime_diagnostics_reporter
        tests/test_runtime_diagnostics_reporter.cpp
    )

    set_tests_properties(
        scenario_autonomy_outcomes
        autonomy_supervision_integration
        progress_recovery
        PROPERTIES LABELS "fast_sim"
    )
    set_tests_properties(
        hardware_hil_acceptance
        PROPERTIES LABELS "extended_hil;manual_gate"
    )
endif()
