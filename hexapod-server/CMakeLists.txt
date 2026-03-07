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
