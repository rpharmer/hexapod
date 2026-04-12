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
endif()

if(MINPHYS3D_BUILD_DEMO)
    add_executable(hexapod-physics-sim
        src/main.cpp
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
    add_minphys3d_test(test_servo_joint_target tests/test_servo_joint_target.cpp)
    add_minphys3d_test(test_servo_chain_stability tests/test_servo_chain_stability.cpp)
    add_minphys3d_test(test_servo_stability_regression tests/test_servo_stability_regression.cpp)
endif()
