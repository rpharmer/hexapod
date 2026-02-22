cmake_minimum_required(VERSION 3.12)

# Change your executable name to something creative!
set(NAME hexapod-client) # <-- Name your project/executable here!
set(FLASH_SCRIPT program-pico.sh)


include(pico_sdk_import.cmake)
include(pimoroni_pico_import.cmake)

# Gooey boilerplate
project(${NAME} C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()

# Add your source files
add_executable(${NAME}
    hexapod-client.cpp
    serialCommsClient.cpp # <-- Add source files here!
)

# Specify the directory containing hexapod-common header files
target_include_directories(${NAME} PRIVATE /home/volly/pico/hexapod/hexapod-common/include)

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
file(WRITE "${CMAKE_BINARY_DIR}/${FLASH_SCRIPT}" "#!/bin/bash\n\nopenocd -f interface/linuxgpiod-new.cfg -f target/rp2040.cfg -c \"program ${CMAKE_CURRENT_BINARY_DIR}/${NAME}.elf verify reset exit\"\n")
file(CHMOD "${CMAKE_CURRENT_BINARY_DIR}/${FLASH_SCRIPT}" PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)

# upload executable to pico
add_custom_target(
    program-pico
    COMMAND "${CMAKE_CURRENT_BINARY_DIR}/${FLASH_SCRIPT}"
    WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
)

set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY OFF)
set(CPACK_GENERATOR "ZIP" "TGZ")
include(CPack)

pico_enable_stdio_usb(${NAME} 1)
pico_enable_stdio_uart(${NAME} 0)