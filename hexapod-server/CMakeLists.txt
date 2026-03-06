cmake_minimum_required(VERSION 3.16)
include(FetchContent)

# Disable third-party tests when fetching dependencies.
set(BUILD_TESTING OFF CACHE BOOL "Disable CTest-based tests" FORCE)
set(BUILD_TESTS OFF CACHE BOOL "Disable project-specific tests" FORCE)

find_package(CppLinuxSerial QUIET)
find_package(toml11 QUIET)

if(NOT CppLinuxSerial_FOUND)
    FetchContent_Declare(
        CppLinuxSerial
        GIT_REPOSITORY https://github.com/gbmhunter/CppLinuxSerial.git
        GIT_TAG v2.8.5
    )
    FetchContent_MakeAvailable(CppLinuxSerial)

    if(TARGET CppLinuxSerial AND NOT TARGET CppLinuxSerial::CppLinuxSerial)
        add_library(CppLinuxSerial::CppLinuxSerial ALIAS CppLinuxSerial)
    endif()
endif()

if(NOT toml11_FOUND)
    FetchContent_Declare(
        toml11
        GIT_REPOSITORY https://github.com/ToruNiina/toml11.git
        GIT_TAG v4.4.0
    )
    FetchContent_MakeAvailable(toml11)
endif()

project(hexapod-server LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

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
