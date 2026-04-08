cmake_minimum_required(VERSION 3.16)

project(hexapod_physics_sim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(MINPHYS3D_BUILD_DEMO "Build hexapod-physics-sim demo executable" ON)
option(MINPHYS3D_BUILD_TESTS "Build minphys3d test executables" ON)

set(MINPHYS3D_CORE_SOURCES
    src/minphys3d/core/world.cpp
    src/minphys3d/core/world_broadphase.cpp
    src/minphys3d/core/world_collision.cpp
    src/minphys3d/core/world_solver.cpp
    src/minphys3d/core/broadphase_system.cpp
    src/minphys3d/core/contact_pipeline.cpp
    src/minphys3d/core/constraint_solver.cpp
    src/minphys3d/core/sleep_system.cpp
    src/minphys3d/solver/block2_solver.cpp
    src/minphys3d/solver/block4_solver.cpp
    src/minphys3d/solver/island_ordering.cpp
    src/minphys3d/narrowphase/gjk.cpp
    src/minphys3d/narrowphase/epa.cpp
)

set(MINPHYS3D_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}
)

if(MINPHYS3D_BUILD_DEMO)
    add_executable(hexapod-physics-sim
        src/main.cpp
        ${MINPHYS3D_CORE_SOURCES}
        src/demo/scenes.cpp
    )
    target_include_directories(hexapod-physics-sim PRIVATE ${MINPHYS3D_INCLUDE_DIRS})
    if(MSVC)
        target_compile_options(hexapod-physics-sim PRIVATE /W4)
    else()
        target_compile_options(hexapod-physics-sim PRIVATE -Wall -Wextra -pedantic)
    endif()
endif()

if(MINPHYS3D_BUILD_TESTS)
    include(CTest)

    function(add_minphys3d_test target_name test_source)
        add_executable(${target_name}
            ${test_source}
            ${MINPHYS3D_CORE_SOURCES}
        )
        target_include_directories(${target_name} PRIVATE ${MINPHYS3D_INCLUDE_DIRS})
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
endif()
