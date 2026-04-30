cmake_minimum_required(VERSION 3.16)
project(hexapod_opengl_visualiser LANGUAGES CXX C)

include(FetchContent)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenGL REQUIRED)
find_package(glfw3 3.3 REQUIRED)

add_library(glad STATIC
  third_party/glad/src/gl.c
)
target_include_directories(glad PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/third_party/glad/include"
)

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
    "${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp"
  )

  target_include_directories(imgui
    PUBLIC
      "${imgui_SOURCE_DIR}"
      "${imgui_SOURCE_DIR}/backends"
  )

  target_link_libraries(imgui
    PUBLIC
      glad
      glfw
      OpenGL::GL
  )

  target_compile_definitions(imgui PUBLIC IMGUI_IMPL_OPENGL_LOADER_GLAD)
endif()

set(VISUALISER_SOURCES
  src/app/legacy_main.cpp
  src/app/options.cpp
  src/math/geometry.cpp
  src/net/udp_receiver.cpp
  src/parsing/json_extract.cpp
  src/parsing/json_packets.cpp
  src/parsing/packet_dispatch.cpp
  src/parsing/viz_binary.cpp
  src/gl/debug.cpp
  src/gl/loader.cpp
  src/gl/shader.cpp
  src/render/camera.cpp
  src/render/line_renderer.cpp
  src/render/mat4.cpp
  src/render/mesh_renderer.cpp
  src/render/point_renderer.cpp
  src/render/primitive_draw.cpp
  src/robot/enum_names.cpp
  src/robot/geometry_state.cpp
  src/robot/kinematics.cpp
  src/scene/scene_bounds.cpp
  src/ui/overlay.cpp
)

add_library(visualiser_core STATIC
  ${VISUALISER_SOURCES}
)

target_include_directories(visualiser_core
  PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
  PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/../hexapod-common/include"
)

target_link_libraries(visualiser_core
  PUBLIC
    glad
    OpenGL::GL
    glfw
    imgui
)

add_executable(hexapod-opengl-visualiser
  src/main.cpp
)

target_link_libraries(hexapod-opengl-visualiser
  PRIVATE
    visualiser_core
)

enable_testing()

add_executable(test_visualiser_frame_math
  tests/test_visualiser_frame_math.cpp
)

target_include_directories(test_visualiser_frame_math
  PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
)

add_test(NAME test_visualiser_frame_math COMMAND test_visualiser_frame_math)

add_executable(test_body_pose_transform
  tests/test_body_pose_transform.cpp
)
target_link_libraries(test_body_pose_transform PRIVATE visualiser_core)
add_test(NAME test_body_pose_transform COMMAND test_body_pose_transform)

add_executable(test_server_visualiser_roundtrip
  tests/test_server_visualiser_roundtrip.cpp
)
target_link_libraries(test_server_visualiser_roundtrip PRIVATE visualiser_core)
add_test(NAME test_server_visualiser_roundtrip COMMAND test_server_visualiser_roundtrip)

add_executable(test_json_extract
  tests/test_json_extract.cpp
)
target_link_libraries(test_json_extract PRIVATE visualiser_core)
add_test(NAME test_json_extract COMMAND test_json_extract)

add_executable(test_json_packets
  tests/test_json_packets.cpp
)
target_link_libraries(test_json_packets PRIVATE visualiser_core)
add_test(NAME test_json_packets COMMAND test_json_packets)

add_executable(test_viz_binary
  tests/test_viz_binary.cpp
)
target_link_libraries(test_viz_binary PRIVATE visualiser_core)
target_include_directories(test_viz_binary PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../hexapod-common/include")
add_test(NAME test_viz_binary COMMAND test_viz_binary)

add_executable(test_kinematics
  tests/test_kinematics.cpp
)
target_link_libraries(test_kinematics PRIVATE visualiser_core)
add_test(NAME test_kinematics COMMAND test_kinematics)

add_executable(test_scene_bounds
  tests/test_scene_bounds.cpp
)
target_link_libraries(test_scene_bounds PRIVATE visualiser_core)
add_test(NAME test_scene_bounds COMMAND test_scene_bounds)

add_executable(test_mat4
  tests/test_mat4.cpp
  src/render/mat4.cpp
)
target_include_directories(test_mat4 PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include")
add_test(NAME test_mat4 COMMAND test_mat4)

add_executable(test_camera
  tests/test_camera.cpp
  src/render/camera.cpp
  src/render/mat4.cpp
)
target_include_directories(test_camera PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include")
add_test(NAME test_camera COMMAND test_camera)
