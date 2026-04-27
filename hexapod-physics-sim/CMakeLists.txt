cmake_minimum_required(VERSION 3.16)

project(hexapod_physics_sim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(MINPHYS3D_BUILD_DEMO "Build hexapod-physics-sim demo executable" ON)
option(MINPHYS3D_BUILD_TESTS "Build minphys3d test executables" ON)

set(MINPHYS3D_CORE_SOURCES
    src/core/world.cpp
    src/core/world_private_methods.cpp
    src/core/world_broadphase.cpp
    src/core/world_collision.cpp
    src/core/cylinder_contacts.cpp
    src/core/world_solver.cpp
    src/core/broadphase_system.cpp
    src/core/contact_pipeline.cpp
    src/core/constraint_solver.cpp
    src/core/sleep_system.cpp
    src/solver/block2_solver.cpp
    src/solver/block4_solver.cpp
    src/solver/island_ordering.cpp
    src/narrowphase/gjk.cpp
    src/narrowphase/epa.cpp
)

set(MINPHYS3D_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../hexapod-common/include
)

# Single compilation of shared engine sources; link this into the demo and each test.
# (Listing the same .cpp files on many add_executable() targets is common but recompiles per target.)
if(MINPHYS3D_BUILD_DEMO OR MINPHYS3D_BUILD_TESTS)
    add_library(minphys3d_core STATIC ${MINPHYS3D_CORE_SOURCES})
    target_include_directories(minphys3d_core PUBLIC ${MINPHYS3D_INCLUDE_DIRS})
    if(MSVC)
        target_compile_options(minphys3d_core PRIVATE /W4)
    else()
        target_compile_options(minphys3d_core PRIVATE -Wall -Wextra -pedantic)
    endif()
    if(MINPHYS3D_BUILD_TESTS)
        # Regression tests assert on solver telemetry; keep the definition public so
        # minphys3d_core and linked test executables agree on World's layout.
        target_compile_definitions(minphys3d_core PUBLIC MINPHYS3D_ENABLE_SOLVER_TELEMETRY=1)
    endif()
endif()

if(MINPHYS3D_BUILD_DEMO)
    add_executable(hexapod-physics-sim
        src/main.cpp
        src/demo/matrix_lidar_sim.cpp
        src/demo/serve_mode.cpp
        src/demo/scenes.cpp
        src/demo/frame_sink.cpp
        src/demo/interactive_presets.cpp
        src/demo/interactive_terminal.cpp
        src/demo/scene_json.cpp
    )
    target_link_libraries(hexapod-physics-sim PRIVATE minphys3d_core)
    target_include_directories(hexapod-physics-sim PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
    if(MSVC)
        target_compile_options(hexapod-physics-sim PRIVATE /W4)
    else()
        target_compile_options(hexapod-physics-sim PRIVATE -Wall -Wextra -pedantic)
    endif()
endif()

if(MINPHYS3D_BUILD_TESTS)
    include(CTest)

    function(add_minphys3d_test target_name test_source)
        add_executable(${target_name} ${test_source})
        target_link_libraries(${target_name} PRIVATE minphys3d_core)
        target_include_directories(${target_name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
        if(MSVC)
            target_compile_options(${target_name} PRIVATE /W4)
        else()
            target_compile_options(${target_name} PRIVATE -Wall -Wextra -pedantic)
        endif()
        add_test(NAME ${target_name} COMMAND ${target_name})
    endfunction()

    add_minphys3d_test(test_unit_math tests/test_unit_math.cpp)
    add_minphys3d_test(test_regression_step tests/test_regression_step.cpp)
    add_minphys3d_test(test_broadphase_dynamic_tree tests/test_broadphase_dynamic_tree.cpp)
    add_minphys3d_test(regression_scene_suite tests/regression_scene_suite.cpp)
    add_minphys3d_test(test_convex_dispatch_equivalence tests/test_convex_dispatch_equivalence.cpp)
    add_minphys3d_test(test_gjk_regression tests/test_gjk_regression.cpp)
    add_minphys3d_test(test_block2_solver tests/test_block2_solver.cpp)
    add_minphys3d_test(test_block4_solver tests/test_block4_solver.cpp)
    add_minphys3d_test(test_island_ordering tests/test_island_ordering.cpp)
    add_minphys3d_test(test_joint_creation_refactor tests/test_joint_creation_refactor.cpp)
    add_minphys3d_test(test_compound_shapes tests/test_compound_shapes.cpp)
    add_minphys3d_test(test_cylinder_collision tests/test_cylinder_collision.cpp)
    add_minphys3d_test(test_terrain_patch tests/test_terrain_patch.cpp)
    add_minphys3d_test(test_terrain_scroll tests/test_terrain_scroll.cpp)
add_minphys3d_test(test_servo_joint_target tests/test_servo_joint_target.cpp)
add_minphys3d_test(test_servo_chain_stability tests/test_servo_chain_stability.cpp)
add_minphys3d_test(test_servo_stability_regression tests/test_servo_stability_regression.cpp)
add_minphys3d_test(test_servo_regression_matrix tests/test_servo_regression_matrix.cpp)
add_minphys3d_test(test_hexapod_live tests/test_hexapod_live.cpp)
add_minphys3d_test(test_hexapod_live_pose_hold tests/test_hexapod_live_pose_hold.cpp)
add_minphys3d_test(test_hexapod_zero_g tests/test_hexapod_zero_g.cpp)
add_minphys3d_test(hex_diag tests/hex_diag.cpp)

    execute_process(
        COMMAND git rev-parse --short HEAD
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        OUTPUT_VARIABLE MPH3D_STABILITY_GIT_SHORT
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )
    if(MPH3D_STABILITY_GIT_SHORT STREQUAL "")
        set(MPH3D_STABILITY_GIT_SHORT "unknown")
    endif()
    file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/stability_git.inc
        "\"${MPH3D_STABILITY_GIT_SHORT}\"\n")

    add_executable(record_hexapod_stability tests/record_hexapod_stability.cpp)
    target_link_libraries(record_hexapod_stability PRIVATE minphys3d_core)
    target_include_directories(record_hexapod_stability PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_BINARY_DIR})
    if(MSVC)
        target_compile_options(record_hexapod_stability PRIVATE /W4)
    else()
        target_compile_options(record_hexapod_stability PRIVATE -Wall -Wextra -pedantic)
    endif()

    add_executable(sweep_hexapod_knobs tests/sweep_hexapod_knobs.cpp)
    target_link_libraries(sweep_hexapod_knobs PRIVATE minphys3d_core)
    target_include_directories(sweep_hexapod_knobs PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
    if(MSVC)
        target_compile_options(sweep_hexapod_knobs PRIVATE /W4)
    else()
        target_compile_options(sweep_hexapod_knobs PRIVATE -Wall -Wextra -pedantic)
    endif()

    add_executable(compare_servo_stride_long tests/compare_servo_stride_long.cpp)
    target_link_libraries(compare_servo_stride_long PRIVATE minphys3d_core)
    target_include_directories(compare_servo_stride_long PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
    if(MSVC)
        target_compile_options(compare_servo_stride_long PRIVATE /W4)
    else()
        target_compile_options(compare_servo_stride_long PRIVATE -Wall -Wextra -pedantic)
    endif()

    add_executable(world_resource_profile_oneoff profiling/world_resource_profile_oneoff.cpp)
    target_link_libraries(world_resource_profile_oneoff PRIVATE minphys3d_core)
    target_include_directories(world_resource_profile_oneoff PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
    if(MSVC)
        target_compile_options(world_resource_profile_oneoff PRIVATE /W4)
    else()
        target_compile_options(world_resource_profile_oneoff PRIVATE -Wall -Wextra -pedantic)
    endif()

    add_executable(test_servo_visual_presets_json
        tests/test_servo_visual_presets_json.cpp
        src/demo/scene_json.cpp
        src/demo/frame_sink.cpp)
    target_link_libraries(test_servo_visual_presets_json PRIVATE minphys3d_core)
    target_include_directories(test_servo_visual_presets_json PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
    if(MSVC)
        target_compile_options(test_servo_visual_presets_json PRIVATE /W4)
    else()
        target_compile_options(test_servo_visual_presets_json PRIVATE -Wall -Wextra -pedantic)
    endif()
    add_test(NAME test_servo_visual_presets_json COMMAND test_servo_visual_presets_json)

    add_executable(test_terrain_patch_scene_json
        tests/test_terrain_patch_scene_json.cpp
        src/demo/scene_json.cpp
        src/demo/frame_sink.cpp)
    target_link_libraries(test_terrain_patch_scene_json PRIVATE minphys3d_core)
    target_include_directories(test_terrain_patch_scene_json PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
    if(MSVC)
        target_compile_options(test_terrain_patch_scene_json PRIVATE /W4)
    else()
        target_compile_options(test_terrain_patch_scene_json PRIVATE -Wall -Wextra -pedantic)
    endif()
    add_test(NAME test_terrain_patch_scene_json COMMAND test_terrain_patch_scene_json)

    add_minphys3d_test(test_process_resource_monitoring tests/test_process_resource_monitoring.cpp)
    add_minphys3d_test(test_world_resource_monitoring tests/test_world_resource_monitoring.cpp)
    add_minphys3d_test(test_world_topology_snapshot tests/test_world_topology_snapshot.cpp)

    add_executable(test_serve_ipc tests/test_serve_ipc.cpp)
    target_include_directories(test_serve_ipc PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../hexapod-common/include)
    if(MSVC)
        target_compile_options(test_serve_ipc PRIVATE /W4)
    else()
        target_compile_options(test_serve_ipc PRIVATE -Wall -Wextra -pedantic)
    endif()
    add_test(NAME test_serve_ipc COMMAND test_serve_ipc $<TARGET_FILE:hexapod-physics-sim>)

    add_executable(test_state_correction_protocol tests/test_state_correction_protocol.cpp)
    target_include_directories(test_state_correction_protocol PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../hexapod-common/include)
    if(MSVC)
        target_compile_options(test_state_correction_protocol PRIVATE /W4)
    else()
        target_compile_options(test_state_correction_protocol PRIVATE -Wall -Wextra -pedantic)
    endif()
    add_test(NAME test_state_correction_protocol COMMAND test_state_correction_protocol)
endif()
