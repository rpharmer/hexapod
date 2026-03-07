cmake_minimum_required(VERSION 3.16)
include(FetchContent)

project(hexapod-server LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Disable third-party tests when fetching dependencies.
set(BUILD_TESTING OFF CACHE BOOL "Disable CTest-based tests" FORCE)
set(BUILD_TESTS OFF CACHE BOOL "Disable project-specific tests" FORCE)

find_package(CppLinuxSerial REQUIRED)
find_package(toml11 REQUIRED)

add_executable(hexapod-server
    src/body_controller.cpp
    src/estimator.cpp
    src/gait_scheduler.cpp
    src/hardware_bridge.cpp
    src/hexapod-server.cpp
    src/leg_fk.cpp
    src/leg_ik.cpp
    src/robot_control.cpp
    src/safety_supervisor.cpp
    src/serialCommsServer.cpp
    ../hexapod-common/framing.cpp
)

target_include_directories(hexapod-server
    PRIVATE
        include
        ../hexapod-common/include
)

if(MSVC)
    target_compile_options(hexapod-server PRIVATE /W4 /WX)
else()
    target_compile_options(hexapod-server PRIVATE -pedantic-errors -Wall -Wextra -Werror -pthread)
endif()

target_link_libraries(hexapod-server
    PRIVATE
        CppLinuxSerial::CppLinuxSerial
        toml11::toml11
)
