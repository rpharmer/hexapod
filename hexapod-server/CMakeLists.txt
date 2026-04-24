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
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/body_controller.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/body_pose_controller.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/calibration_probe.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/cli_options.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/contact_foot_response.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/control_config.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/control_pipeline.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/estimator.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/freshness_policy.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/foot_planners.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/foot_terrain.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/foothold_planner.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/foot_reachability.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/gait_params.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/gait_scheduler.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/geometry_profile_service.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/interactive_calibration_actions.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/interactive_input_mapper.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/local_map.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/local_planner.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/loop_executor.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/loop_timing.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/locomotion_command.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/locomotion_stability.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/mode_runners.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/motion_intent_utils.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/navigation_manager.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/nav_primitives.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/nav_to_locomotion.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/nav_locomotion_bridge.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/physics_sim_estimator.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/matrix_lidar_local_map_source.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/physics_sim_local_map_source.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/plane_estimation.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/probe_contact_logic.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/replay_json.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/replay_logger.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/robot_control.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/robot_runtime.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/runtime_diagnostics_reporter.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/runtime_freshness_gate.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/runtime_timing_metrics.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/safety_supervisor.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/state_fusion.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/servo_dynamics_fit.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/status_reporter.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/swing_trajectory.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/telemetry_json.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/telemetry_publisher.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/touch_residuals.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/visualiser_telemetry.cpp"

    # hardware
    "${CMAKE_CURRENT_SOURCE_DIR}/src/hardware/bridge_command_api.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/hardware/bridge_link_manager.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/hardware/command_client.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/hardware/handshake_client.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/hardware/hardware_bridge.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/hardware/hardware_state_codec.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/hardware/joint_feedback_estimator.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/hardware/physics_sim_bridge.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/hardware/serialCommsServer.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/hardware/sim_hardware_bridge.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/hardware/transport_session.cpp"

    # input
    "${CMAKE_CURRENT_SOURCE_DIR}/src/input/evdev_gamepad_controller.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/input/virtual_gamepad_controller.cpp"

    # kinematics
    "${CMAKE_CURRENT_SOURCE_DIR}/src/kinematics/leg_fk.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/kinematics/leg_ik.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/kinematics/types.cpp"

    # scenario
    "${CMAKE_CURRENT_SOURCE_DIR}/src/scenario/scenario_driver.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/sensors/imu_physics.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/sensors/matrix_lidar_physics.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/sensors/software_imu.cpp"

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
        include/sensors
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

if(HEXAPOD_SERVER_BUILD_TESTS)
    enable_testing()

    add_executable(test_robot_runtime_loop tests/test_robot_runtime_loop.cpp)
    target_link_libraries(test_robot_runtime_loop PRIVATE hexapod_server_core)
    add_test(NAME robot_runtime_loop_behavior COMMAND test_robot_runtime_loop)

    add_executable(test_safety_supervisor_faults tests/test_safety_supervisor_faults.cpp)
    target_link_libraries(test_safety_supervisor_faults PRIVATE hexapod_server_core)
    add_test(NAME safety_supervisor_transitions COMMAND test_safety_supervisor_faults)

    add_executable(test_contact_foot_response tests/test_contact_foot_response.cpp)
    target_link_libraries(test_contact_foot_response PRIVATE hexapod_server_core)
    add_test(NAME contact_foot_response COMMAND test_contact_foot_response)

    add_executable(test_state_fusion tests/test_state_fusion.cpp)
    target_link_libraries(test_state_fusion PRIVATE hexapod_server_core)
    add_test(NAME state_fusion COMMAND test_state_fusion)

    add_executable(test_imu_physics tests/test_imu_physics.cpp)
    target_link_libraries(test_imu_physics PRIVATE hexapod_server_core)
    add_test(NAME imu_physics COMMAND test_imu_physics)

    add_executable(test_matrix_lidar tests/test_matrix_lidar.cpp)
    target_link_libraries(test_matrix_lidar PRIVATE hexapod_server_core)
    add_test(NAME matrix_lidar COMMAND test_matrix_lidar)

    add_executable(test_control_pipeline_sanity tests/test_control_pipeline_sanity.cpp)
    target_link_libraries(test_control_pipeline_sanity PRIVATE hexapod_server_core)
    add_test(NAME control_pipeline_output_sanity COMMAND test_control_pipeline_sanity)

    add_executable(test_hardware_bridge_transport
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
    target_link_libraries(test_hardware_bridge_transport PRIVATE hexapod_server_core)
    add_test(NAME hardware_bridge_transport_behavior COMMAND test_hardware_bridge_transport)

    add_executable(test_servo_calibration_unified_conversion tests/test_servo_calibration_unified_conversion.cpp)
    target_link_libraries(test_servo_calibration_unified_conversion PRIVATE hexapod_server_core)
    add_test(NAME servo_calibration_unified_conversion COMMAND test_servo_calibration_unified_conversion)

    add_executable(test_motion_intent_through_ik_fk tests/test_motion_intent_through_ik_fk.cpp)
    target_link_libraries(test_motion_intent_through_ik_fk PRIVATE hexapod_server_core)
    add_test(NAME motion_intent_through_ik_fk_chain COMMAND test_motion_intent_through_ik_fk)

    add_executable(test_twist_field tests/test_twist_field.cpp)
    target_link_libraries(test_twist_field PRIVATE hexapod_server_core)
    add_test(NAME twist_field COMMAND test_twist_field)

    add_executable(test_locomotion_command tests/test_locomotion_command.cpp)
    target_link_libraries(test_locomotion_command PRIVATE hexapod_server_core)
    add_test(NAME locomotion_command COMMAND test_locomotion_command)

    add_executable(test_foothold_planner tests/test_foothold_planner.cpp)
    target_link_libraries(test_foothold_planner PRIVATE hexapod_server_core)
    add_test(NAME foothold_planner COMMAND test_foothold_planner)

    add_executable(test_foot_reachability tests/test_foot_reachability.cpp)
    target_link_libraries(test_foot_reachability PRIVATE hexapod_server_core)
    add_test(NAME foot_reachability COMMAND test_foot_reachability)

    add_executable(test_foot_terrain tests/test_foot_terrain.cpp)
    target_link_libraries(test_foot_terrain PRIVATE hexapod_server_core)
    add_test(NAME foot_terrain COMMAND test_foot_terrain)

    add_executable(test_swing_trajectory tests/test_swing_trajectory.cpp)
    target_link_libraries(test_swing_trajectory PRIVATE hexapod_server_core)
    add_test(NAME swing_trajectory COMMAND test_swing_trajectory)

    add_executable(test_gait_params tests/test_gait_params.cpp)
    target_link_libraries(test_gait_params PRIVATE hexapod_server_core)
    add_test(NAME gait_params COMMAND test_gait_params)

    add_executable(test_nav_primitives tests/test_nav_primitives.cpp)
    target_link_libraries(test_nav_primitives PRIVATE hexapod_server_core)
    add_test(NAME nav_primitives COMMAND test_nav_primitives)

    add_executable(test_nav_locomotion_bridge tests/test_nav_locomotion_bridge.cpp)
    target_link_libraries(test_nav_locomotion_bridge PRIVATE hexapod_server_core)
    add_test(NAME nav_locomotion_bridge COMMAND test_nav_locomotion_bridge)

    add_executable(test_local_map tests/test_local_map.cpp)
    target_link_libraries(test_local_map PRIVATE hexapod_server_core)
    add_test(NAME local_map COMMAND test_local_map)

    add_executable(test_local_planner tests/test_local_planner.cpp)
    target_link_libraries(test_local_planner PRIVATE hexapod_server_core)
    add_test(NAME local_planner COMMAND test_local_planner)

    add_executable(test_navigation_manager tests/test_navigation_manager.cpp)
    target_link_libraries(test_navigation_manager PRIVATE hexapod_server_core)
    add_test(NAME navigation_manager COMMAND test_navigation_manager)

    add_executable(test_navigation_matrix_lidar tests/test_navigation_matrix_lidar.cpp)
    target_link_libraries(test_navigation_matrix_lidar PRIVATE hexapod_server_core)
    add_test(NAME navigation_matrix_lidar COMMAND test_navigation_matrix_lidar)

    add_executable(test_navigation_runtime tests/test_navigation_runtime.cpp)
    target_link_libraries(test_navigation_runtime PRIVATE hexapod_server_core)
    add_test(NAME navigation_runtime COMMAND test_navigation_runtime)

    add_executable(test_physics_sim_local_map_source tests/test_physics_sim_local_map_source.cpp)
    target_link_libraries(test_physics_sim_local_map_source PRIVATE hexapod_server_core)
    add_test(NAME physics_sim_local_map_source COMMAND test_physics_sim_local_map_source)

    add_executable(test_matrix_lidar_local_map_source tests/test_matrix_lidar_local_map_source.cpp)
    target_link_libraries(test_matrix_lidar_local_map_source PRIVATE hexapod_server_core)
    add_test(NAME matrix_lidar_local_map_source COMMAND test_matrix_lidar_local_map_source)

    add_executable(test_physics_sim_mapping_roundtrip tests/test_physics_sim_mapping_roundtrip.cpp)
    target_link_libraries(test_physics_sim_mapping_roundtrip PRIVATE hexapod_server_core)
    add_test(NAME physics_sim_mapping_roundtrip COMMAND test_physics_sim_mapping_roundtrip)

    add_executable(test_physics_sim_ik_wire_roundtrip tests/test_physics_sim_ik_wire_roundtrip.cpp)
    target_link_libraries(test_physics_sim_ik_wire_roundtrip PRIVATE hexapod_server_core)
    add_test(NAME physics_sim_ik_wire_roundtrip COMMAND test_physics_sim_ik_wire_roundtrip)

    add_executable(test_physics_sim_bridge_frame_conversion tests/test_physics_sim_bridge_frame_conversion.cpp)
    target_link_libraries(test_physics_sim_bridge_frame_conversion PRIVATE hexapod_server_core)
    add_test(NAME physics_sim_bridge_frame_conversion COMMAND test_physics_sim_bridge_frame_conversion)

    add_executable(test_physics_sim_bridge_state_correction tests/test_physics_sim_bridge_state_correction.cpp)
    target_link_libraries(test_physics_sim_bridge_state_correction PRIVATE hexapod_server_core)
    add_test(NAME physics_sim_bridge_state_correction COMMAND test_physics_sim_bridge_state_correction)

    add_executable(test_robot_runtime_physics_sim_corrections tests/test_robot_runtime_physics_sim_corrections.cpp)
    target_link_libraries(test_robot_runtime_physics_sim_corrections PRIVATE hexapod_server_core)
    add_test(NAME robot_runtime_physics_sim_corrections COMMAND test_robot_runtime_physics_sim_corrections)

    add_executable(test_physics_sim_serve_zero_g_udp tests/test_physics_sim_serve_zero_g_udp.cpp)
    target_link_libraries(test_physics_sim_serve_zero_g_udp PRIVATE hexapod_server_core)

    add_executable(test_physics_sim_walk_distance tests/test_physics_sim_walk_distance.cpp)
    target_link_libraries(test_physics_sim_walk_distance PRIVATE hexapod_server_core)
    add_test(NAME physics_sim_walk_distance COMMAND test_physics_sim_walk_distance)

    add_executable(test_physics_sim_nav_waypoints tests/test_physics_sim_nav_waypoints.cpp)
    target_link_libraries(test_physics_sim_nav_waypoints PRIVATE hexapod_server_core)
    add_test(NAME physics_sim_nav_waypoints COMMAND test_physics_sim_nav_waypoints)

    add_executable(test_physics_sim_navigation_acceptance tests/test_physics_sim_navigation_acceptance.cpp)
    target_link_libraries(test_physics_sim_navigation_acceptance PRIVATE hexapod_server_core)
    add_test(NAME physics_sim_navigation_acceptance COMMAND test_physics_sim_navigation_acceptance)

    add_executable(test_virtual_gamepad_controller tests/test_virtual_gamepad_controller.cpp)
    target_link_libraries(test_virtual_gamepad_controller PRIVATE hexapod_server_core)
    add_test(NAME virtual_gamepad_controller COMMAND test_virtual_gamepad_controller)

    add_executable(test_scenario_driver_validation tests/test_scenario_driver_validation.cpp)
    target_link_libraries(test_scenario_driver_validation PRIVATE hexapod_server_core)
    add_test(NAME scenario_driver_validation COMMAND test_scenario_driver_validation)

    add_executable(test_simple_estimator_ground_plane tests/test_simple_estimator_ground_plane.cpp)
    target_link_libraries(test_simple_estimator_ground_plane PRIVATE hexapod_server_core)
    add_test(NAME simple_estimator_ground_plane COMMAND test_simple_estimator_ground_plane)

    add_executable(test_physics_sim_estimator tests/test_physics_sim_estimator.cpp)
    target_link_libraries(test_physics_sim_estimator PRIVATE hexapod_server_core)
    add_test(NAME physics_sim_estimator COMMAND test_physics_sim_estimator)

    add_executable(test_body_controller_velocity tests/test_body_controller_velocity.cpp)
    target_link_libraries(test_body_controller_velocity PRIVATE hexapod_server_core)
    add_test(NAME body_controller_velocity COMMAND test_body_controller_velocity)

    add_executable(test_locomotion_pose_and_stability tests/test_locomotion_pose_and_stability.cpp)
    target_link_libraries(test_locomotion_pose_and_stability PRIVATE hexapod_server_core)
    add_test(NAME locomotion_pose_and_stability COMMAND test_locomotion_pose_and_stability)

    add_executable(test_toml_parser_sections tests/test_toml_parser_sections.cpp)
    target_link_libraries(test_toml_parser_sections PRIVATE hexapod_server_core)
    add_test(NAME toml_parser_sections COMMAND test_toml_parser_sections)

    add_executable(test_cli_mode_selection tests/test_cli_mode_selection.cpp)
    target_link_libraries(test_cli_mode_selection PRIVATE hexapod_server_core)
    add_test(NAME cli_mode_selection COMMAND test_cli_mode_selection)

    add_executable(test_interactive_input_mapper tests/test_interactive_input_mapper.cpp)
    target_link_libraries(test_interactive_input_mapper PRIVATE hexapod_server_core)
    add_test(NAME interactive_input_mapper COMMAND test_interactive_input_mapper)
    add_executable(test_transport_components tests/test_transport_components.cpp)
    target_link_libraries(test_transport_components PRIVATE hexapod_server_core)
    add_test(NAME transport_components COMMAND test_transport_components)

    add_executable(test_freshness_policy tests/test_freshness_policy.cpp)
    target_link_libraries(test_freshness_policy PRIVATE hexapod_server_core)
    add_test(NAME freshness_policy COMMAND test_freshness_policy)

    add_executable(test_robot_runtime_freshness_gate_matrix tests/test_robot_runtime_freshness_gate_matrix.cpp)
    target_link_libraries(test_robot_runtime_freshness_gate_matrix PRIVATE hexapod_server_core)
    add_test(NAME robot_runtime_freshness_gate_matrix COMMAND test_robot_runtime_freshness_gate_matrix)

    add_executable(test_runtime_helpers tests/test_runtime_helpers.cpp)
    target_link_libraries(test_runtime_helpers PRIVATE hexapod_server_core)
    add_test(NAME runtime_helpers COMMAND test_runtime_helpers)

    add_executable(test_visualiser_telemetry tests/test_visualiser_telemetry.cpp)
    target_link_libraries(test_visualiser_telemetry PRIVATE hexapod_server_core)
    add_test(NAME visualiser_telemetry COMMAND test_visualiser_telemetry)

    add_executable(test_shutdown_summary_logging tests/test_shutdown_summary_logging.cpp)
    target_link_libraries(test_shutdown_summary_logging PRIVATE hexapod_server_core)
    add_test(NAME shutdown_summary_logging COMMAND test_shutdown_summary_logging)

    add_executable(test_runtime_teardown_logging tests/test_runtime_teardown_logging.cpp)
    target_link_libraries(test_runtime_teardown_logging PRIVATE hexapod_server_core)
    add_test(NAME runtime_teardown_logging COMMAND test_runtime_teardown_logging)

    add_executable(test_runtime_diagnostics_reporter tests/test_runtime_diagnostics_reporter.cpp)
    target_link_libraries(test_runtime_diagnostics_reporter PRIVATE hexapod_server_core)
    add_test(NAME runtime_diagnostics_reporter COMMAND test_runtime_diagnostics_reporter)

    add_executable(test_geometry_profile_service tests/test_geometry_profile_service.cpp)
    target_link_libraries(test_geometry_profile_service PRIVATE hexapod_server_core)
    add_test(NAME geometry_profile_service COMMAND test_geometry_profile_service)
    add_executable(test_calibration_probe_fit tests/test_calibration_probe_fit.cpp)
    target_link_libraries(test_calibration_probe_fit PRIVATE hexapod_server_core)
    add_test(NAME calibration_probe_fit COMMAND test_calibration_probe_fit)

    add_executable(test_protocol_command_metadata tests/test_protocol_command_metadata.cpp)
    target_link_libraries(test_protocol_command_metadata PRIVATE hexapod_server_core)
    add_test(NAME protocol_command_metadata COMMAND test_protocol_command_metadata)

    add_executable(test_telemetry_json_serialization tests/test_telemetry_json_serialization.cpp)
    target_link_libraries(test_telemetry_json_serialization PRIVATE hexapod_server_core)
    add_test(NAME telemetry_json_serialization COMMAND test_telemetry_json_serialization)

    add_executable(test_replay_logging tests/test_replay_logging.cpp)
    target_link_libraries(test_replay_logging PRIVATE hexapod_server_core)
    add_test(NAME replay_logging COMMAND test_replay_logging)


    add_executable(test_bridge_link_manager tests/test_bridge_link_manager.cpp)
    target_link_libraries(test_bridge_link_manager PRIVATE hexapod_server_core)
    add_test(NAME bridge_link_manager COMMAND test_bridge_link_manager)

    add_executable(test_bridge_command_api tests/test_bridge_command_api.cpp)
    target_link_libraries(test_bridge_command_api PRIVATE hexapod_server_core)
    add_test(NAME bridge_command_api COMMAND test_bridge_command_api)

    add_executable(test_joint_feedback_estimator tests/test_joint_feedback_estimator.cpp)
    target_link_libraries(test_joint_feedback_estimator PRIVATE hexapod_server_core)
    add_test(NAME joint_feedback_estimator COMMAND test_joint_feedback_estimator)

    add_executable(test_command_flow_integration tests/test_command_flow_integration.cpp)
    target_link_libraries(test_command_flow_integration PRIVATE hexapod_server_core)
    add_test(NAME command_flow_integration COMMAND test_command_flow_integration)
    
    add_executable(test_loop_executor_timing tests/test_loop_executor_timing.cpp)
    target_link_libraries(test_loop_executor_timing PRIVATE hexapod_server_core)
    add_test(NAME loop_executor_timing COMMAND test_loop_executor_timing)
endif()
