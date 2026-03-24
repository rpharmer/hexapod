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

file(GLOB_RECURSE HEXAPOD_SERVER_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
list(APPEND HEXAPOD_SERVER_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/cli_options.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/interactive_input_mapper.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/control/interactive_calibration_actions.cpp"
)
list(REMOVE_DUPLICATES HEXAPOD_SERVER_SOURCES)
set(HEXAPOD_SERVER_MAIN_SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/src/app/hexapod-server.cpp")
list(REMOVE_ITEM HEXAPOD_SERVER_SOURCES "${HEXAPOD_SERVER_MAIN_SOURCE}")

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

if(HEXAPOD_SERVER_BUILD_TESTS)
    enable_testing()

    add_executable(test_robot_runtime_loop tests/test_robot_runtime_loop.cpp)
    target_link_libraries(test_robot_runtime_loop PRIVATE hexapod_server_core)
    add_test(NAME robot_runtime_loop_behavior COMMAND test_robot_runtime_loop)

    add_executable(test_safety_supervisor_faults tests/test_safety_supervisor_faults.cpp)
    target_link_libraries(test_safety_supervisor_faults PRIVATE hexapod_server_core)
    add_test(NAME safety_supervisor_transitions COMMAND test_safety_supervisor_faults)

    add_executable(test_control_pipeline_sanity tests/test_control_pipeline_sanity.cpp)
    target_link_libraries(test_control_pipeline_sanity PRIVATE hexapod_server_core)
    add_test(NAME control_pipeline_output_sanity COMMAND test_control_pipeline_sanity)

    add_executable(test_hardware_bridge_transport tests/test_hardware_bridge_transport.cpp)
    target_link_libraries(test_hardware_bridge_transport PRIVATE hexapod_server_core)
    add_test(NAME hardware_bridge_transport_behavior COMMAND test_hardware_bridge_transport)

    add_executable(test_servo_calibration_unified_conversion tests/test_servo_calibration_unified_conversion.cpp)
    target_link_libraries(test_servo_calibration_unified_conversion PRIVATE hexapod_server_core)
    add_test(NAME servo_calibration_unified_conversion COMMAND test_servo_calibration_unified_conversion)

    add_executable(test_motion_intent_through_ik_fk tests/test_motion_intent_through_ik_fk.cpp)
    target_link_libraries(test_motion_intent_through_ik_fk PRIVATE hexapod_server_core)
    add_test(NAME motion_intent_through_ik_fk_chain COMMAND test_motion_intent_through_ik_fk)

    add_executable(test_virtual_gamepad_controller tests/test_virtual_gamepad_controller.cpp)
    target_link_libraries(test_virtual_gamepad_controller PRIVATE hexapod_server_core)
    add_test(NAME virtual_gamepad_controller COMMAND test_virtual_gamepad_controller)

    add_executable(test_scenario_driver_validation tests/test_scenario_driver_validation.cpp)
    target_link_libraries(test_scenario_driver_validation PRIVATE hexapod_server_core)
    add_test(NAME scenario_driver_validation COMMAND test_scenario_driver_validation)

    add_executable(test_simple_estimator_ground_plane tests/test_simple_estimator_ground_plane.cpp)
    target_link_libraries(test_simple_estimator_ground_plane PRIVATE hexapod_server_core)
    add_test(NAME simple_estimator_ground_plane COMMAND test_simple_estimator_ground_plane)

    add_executable(test_body_controller_velocity tests/test_body_controller_velocity.cpp)
    target_link_libraries(test_body_controller_velocity PRIVATE hexapod_server_core)
    add_test(NAME body_controller_velocity COMMAND test_body_controller_velocity)


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


    add_executable(test_geometry_profile_service tests/test_geometry_profile_service.cpp)
    target_link_libraries(test_geometry_profile_service PRIVATE hexapod_server_core)
    add_test(NAME geometry_profile_service COMMAND test_geometry_profile_service)
    add_executable(test_calibration_probe_fit tests/test_calibration_probe_fit.cpp)
    target_link_libraries(test_calibration_probe_fit PRIVATE hexapod_server_core)
    add_test(NAME calibration_probe_fit COMMAND test_calibration_probe_fit)

    add_executable(test_protocol_command_metadata tests/test_protocol_command_metadata.cpp)
    target_link_libraries(test_protocol_command_metadata PRIVATE hexapod_server_core)
    add_test(NAME protocol_command_metadata COMMAND test_protocol_command_metadata)


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
