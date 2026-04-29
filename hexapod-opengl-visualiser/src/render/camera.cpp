#include "visualiser/render/camera.hpp"

#include <algorithm>
#include <cmath>

namespace visualiser::render {

Mat4 LegacyViewMatrix(float scene_radius,
                      float camera_distance,
                      float pitch_deg,
                      float yaw_deg,
                      float center_x,
                      float center_y,
                      float center_z,
                      float pan_x,
                      float pan_y) {
  // Matches fixed-function stack: M = T_back * R_pitch * R_yaw * T_center
  const Mat4 t_back = Mat4::Translate(0.0f, -0.35f * scene_radius, -camera_distance);
  const Mat4 r_pitch = Mat4::RotateX(DegToRad(pitch_deg));
  const Mat4 r_yaw = Mat4::RotateY(DegToRad(yaw_deg));
  const Mat4 t_center =
      Mat4::Translate(-(center_x + pan_x), -(center_y + pan_y), -center_z);
  return Mat4::Mul(t_back, Mat4::Mul(r_pitch, Mat4::Mul(r_yaw, t_center)));
}

Mat4 ProjectionFromLegacyFrustum(int width, int height) {
  const float aspect = height > 0 ? static_cast<float>(width) / static_cast<float>(height) : 1.0f;
  constexpr float near_plane = 0.1f;
  constexpr float far_plane = 100.0f;
  constexpr float fov_y_degrees = 50.0f;
  const float top = std::tan(fov_y_degrees * 0.5f * kPi / 180.0f) * near_plane;
  const float right = top * aspect;
  return Mat4::FrustumSymmetric(right, top, near_plane, far_plane);
}

}  // namespace visualiser::render
