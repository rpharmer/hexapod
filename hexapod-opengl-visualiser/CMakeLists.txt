cmake_minimum_required(VERSION 3.16)
project(hexapod_opengl_visualiser LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenGL REQUIRED)
find_package(glfw3 3.3 REQUIRED)

add_executable(hexapod-opengl-visualiser
  src/main.cpp
)

target_link_libraries(hexapod-opengl-visualiser
  PRIVATE
    OpenGL::GL
    glfw
)
