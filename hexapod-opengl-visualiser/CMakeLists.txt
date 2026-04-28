cmake_minimum_required(VERSION 3.16)
project(hexapod_opengl_visualiser LANGUAGES CXX)

include(FetchContent)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenGL REQUIRED)
find_package(glfw3 3.3 REQUIRED)

FetchContent_Declare(
  imgui
  GIT_REPOSITORY https://github.com/ocornut/imgui.git
  GIT_TAG v1.91.5
)

FetchContent_GetProperties(imgui)
if(NOT imgui_POPULATED)
  FetchContent_Populate(imgui)

  add_library(imgui STATIC
    "${imgui_SOURCE_DIR}/imgui.cpp"
    "${imgui_SOURCE_DIR}/imgui_draw.cpp"
    "${imgui_SOURCE_DIR}/imgui_tables.cpp"
    "${imgui_SOURCE_DIR}/imgui_widgets.cpp"
    "${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp"
    "${imgui_SOURCE_DIR}/backends/imgui_impl_opengl2.cpp"
  )

  target_include_directories(imgui
    PUBLIC
      "${imgui_SOURCE_DIR}"
      "${imgui_SOURCE_DIR}/backends"
  )

  target_link_libraries(imgui
    PUBLIC
      glfw
      OpenGL::GL
  )
endif()

add_executable(hexapod-opengl-visualiser
  src/main.cpp
)

target_include_directories(hexapod-opengl-visualiser
  PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/../hexapod-common/include"
)

target_link_libraries(hexapod-opengl-visualiser
  PRIVATE
  OpenGL::GL
  glfw
  imgui
)
