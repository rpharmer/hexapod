#include "visualiser/render/projection.hpp"

#include <GLFW/glfw3.h>
#include <cmath>

namespace visualiser::render {

void ConfigureProjection(int width, int height) {
  constexpr float kPi = 3.14159265358979323846f;
  const float aspect = height > 0 ? static_cast<float>(width) / static_cast<float>(height) : 1.0f;
  constexpr float near_plane = 0.1f;
  constexpr float far_plane = 100.0f;
  constexpr float fov_y_degrees = 50.0f;
  const float top = std::tan(fov_y_degrees * 0.5f * kPi / 180.0f) * near_plane;
  const float right = top * aspect;
  glViewport(0, 0, width, height);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glFrustum(-right, right, -top, top, near_plane, far_plane);
}

}  // namespace visualiser::render
