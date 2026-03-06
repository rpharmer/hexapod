cmake_minimum_required(VERSION 3.16)

project(hexapod LANGUAGES C CXX)

# Allow a root-level configure to bring in embedded SDK dependencies
# automatically when local SDK paths are not set.
if(NOT PICO_SDK_PATH)
    set(PICO_SDK_FETCH_FROM_GIT ON CACHE BOOL "Fetch Pico SDK when configuring hexapod-client" FORCE)
endif()

if(NOT PIMORONI_PICO_PATH)
    set(PIMORONI_PICO_FETCH_FROM_GIT ON CACHE BOOL "Fetch Pimoroni Pico SDK when configuring hexapod-client" FORCE)
endif()

add_subdirectory(hexapod-server)
add_subdirectory(hexapod-client)
