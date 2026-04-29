#pragma once

namespace visualiser::render {

inline constexpr const char* kWireVertGlsl = R"(
#version 330 core
layout(location=0) in vec3 a_pos;
layout(location=1) in vec3 a_color;
uniform mat4 u_mvp;
out vec3 v_color;
void main() {
  v_color = a_color;
  gl_Position = u_mvp * vec4(a_pos, 1.0);
}
)";

inline constexpr const char* kWireFragGlsl = R"(
#version 330 core
in vec3 v_color;
out vec4 frag_color;
void main() {
  frag_color = vec4(v_color, 1.0);
}
)";

inline constexpr const char* kMeshVertGlsl = R"(
#version 330 core
layout(location=0) in vec3 a_pos;
layout(location=1) in vec3 a_normal;
layout(location=2) in vec3 a_color;
uniform mat4 u_mvp;
uniform mat3 u_normal_matrix;
out vec3 v_normal;
out vec3 v_color;
void main() {
  v_normal = u_normal_matrix * a_normal;
  v_color = a_color;
  gl_Position = u_mvp * vec4(a_pos, 1.0);
}
)";

inline constexpr const char* kMeshFragGlsl = R"(
#version 330 core
in vec3 v_normal;
in vec3 v_color;
uniform vec3 u_light_dir;
out vec4 frag_color;
void main() {
  vec3 n = normalize(v_normal);
  vec3 L = normalize(u_light_dir);
  float lambert = max(dot(n, L), 0.15);
  frag_color = vec4(v_color * lambert, 1.0);
}
)";

inline constexpr const char* kPointVertGlsl = R"(
#version 330 core
layout(location=0) in vec3 a_pos;
layout(location=1) in vec3 a_color;
uniform mat4 u_mvp;
uniform float u_point_size;
out vec3 v_color;
void main() {
  v_color = a_color;
  gl_Position = u_mvp * vec4(a_pos, 1.0);
  gl_PointSize = u_point_size;
}
)";

inline constexpr const char* kPointFragGlsl = R"(
#version 330 core
in vec3 v_color;
out vec4 frag_color;
void main() {
  frag_color = vec4(v_color, 1.0);
}
)";

}  // namespace visualiser::render
