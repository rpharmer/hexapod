cmake_minimum_required(VERSION 3.12)

# Change your executable name to something creative!
set(NAME hexapod-client) # <-- Name your project/executable here!
set(FLASH_SCRIPT program-pico.sh)
option(HEXAPOD_CLIENT_SETUP_SDKS_ONLY "Populate/prebuild SDK and Pimoroni dependency objects without compiling project firmware sources" OFF)
option(HEXAPOD_CLIENT_ENABLE_HOST_TESTS "Build native host tests under hexapod-client/tests and register them with CTest" OFF)

project(${NAME} C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

if (HEXAPOD_CLIENT_ENABLE_HOST_TESTS)
    enable_testing()

    set(HEXAPOD_CLIENT_HOST_TEST_COMMON_SOURCES
        command_router.cpp
        firmware_context.cpp
        motion_commands.cpp
        sensing_commands.cpp
        power_commands.cpp
        firmware_boot.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../hexapod-common/framing.cpp
    )

    add_executable(hexapod-client-test-command-router
        tests/test_command_router.cpp
        command_router.cpp
    )
    target_compile_definitions(hexapod-client-test-command-router PRIVATE HEXAPOD_CLIENT_HOST_TEST=1)
    target_include_directories(hexapod-client-test-command-router PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/../hexapod-common/include
    )
    add_test(NAME hexapod-client.command_router COMMAND hexapod-client-test-command-router)

    add_executable(hexapod-client-test-command-handlers
        tests/test_command_handlers.cpp
        ${HEXAPOD_CLIENT_HOST_TEST_COMMON_SOURCES}
    )
    target_compile_definitions(hexapod-client-test-command-handlers PRIVATE HEXAPOD_CLIENT_HOST_TEST=1)
    target_include_directories(hexapod-client-test-command-handlers PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/../hexapod-common/include
    )
    add_test(NAME hexapod-client.command_handlers COMMAND hexapod-client-test-command-handlers)

    return()
endif ()

include(pico_sdk_import.cmake)
include(pimoroni_pico_import.cmake)

# Initialize the SDK
pico_sdk_init()

set(HEXAPOD_CLIENT_EXCLUDE_FROM_ALL)
set(HEXAPOD_CLIENT_SOURCES
    firmware_boot.cpp
    firmware_context.cpp
    command_router.cpp
    command_dispatch.cpp
    power_commands.cpp
    sensing_commands.cpp
    motion_commands.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../hexapod-common/framing.cpp
    serialCommsClient.cpp # <-- Add source files here!
)
if (HEXAPOD_CLIENT_SETUP_SDKS_ONLY)
    set(HEXAPOD_CLIENT_EXCLUDE_FROM_ALL EXCLUDE_FROM_ALL)
    message(STATUS "HEXAPOD_CLIENT_SETUP_SDKS_ONLY=ON: prebuild dependency objects for ${NAME} without compiling project firmware sources")
    set(HEXAPOD_CLIENT_SETUP_STUB "${CMAKE_CURRENT_BINARY_DIR}/hexapod-client-setup-stub.cpp")
    file(WRITE "${HEXAPOD_CLIENT_SETUP_STUB}" "int main() { return 0; }\n")
    set(HEXAPOD_CLIENT_SOURCES "${HEXAPOD_CLIENT_SETUP_STUB}")
endif ()

# Add your source files
add_executable(${NAME} ${HEXAPOD_CLIENT_EXCLUDE_FROM_ALL}
    ${HEXAPOD_CLIENT_SOURCES}
)

# Specify the directory containing hexapod-common header files
target_include_directories(${NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../hexapod-common/include)

# Include required libraries
# This assumes `pimoroni-pico` is stored alongside your project
include(common/pimoroni_i2c)
include(common/pimoroni_bus)
include(libraries/servo2040/servo2040)
include(drivers/button/button)
include(drivers/analogmux/analogmux)
include(drivers/analog/analog)

# Don't forget to link the libraries you need!
target_link_libraries(${NAME}
    pimoroni_i2c
    servo2040
    button
    analogmux
    analog # <-- List libraries here!
)

# create map/bin/hex file etc.
pico_add_extra_outputs(${NAME})

# Set up files for the release packages
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${NAME}.uf2
    ${CMAKE_CURRENT_LIST_DIR}/README.md
    DESTINATION .
)

# Write Shell script to flash pico and give it execute permissions
set(FLASH_SCRIPT_PATH "${CMAKE_CURRENT_BINARY_DIR}/${FLASH_SCRIPT}")
file(WRITE "${FLASH_SCRIPT_PATH}" "#!/bin/bash\n\nopenocd -f interface/linuxgpiod-new.cfg -f target/rp2040.cfg -c \"program ${CMAKE_CURRENT_BINARY_DIR}/${NAME}.elf verify reset exit\"\n")
file(CHMOD "${FLASH_SCRIPT_PATH}" PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)

# upload executable to pico
add_custom_target(
    program-pico
    COMMAND "${CMAKE_CURRENT_BINARY_DIR}/${FLASH_SCRIPT}"
    WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
)

set(HEXAPOD_CLIENT_SETUP_DEPENDENCIES
    ${NAME}
    pimoroni_i2c
    servo2040
    button
    analogmux
    analog
)
if (TARGET pioasmBuild)
    list(APPEND HEXAPOD_CLIENT_SETUP_DEPENDENCIES pioasmBuild)
endif ()
if (TARGET picotoolBuild)
    list(APPEND HEXAPOD_CLIENT_SETUP_DEPENDENCIES picotoolBuild)
endif ()

add_custom_target(setup-sdks
    DEPENDS ${HEXAPOD_CLIENT_SETUP_DEPENDENCIES}
    COMMENT "Populate and prebuild SDK/tool dependencies required by ${NAME}."
)

set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY OFF)
set(CPACK_GENERATOR "ZIP" "TGZ")
include(CPack)

pico_enable_stdio_usb(${NAME} 1)
pico_enable_stdio_uart(${NAME} 0)
