#include "visualiser/math/geometry.hpp"

#include <algorithm>
#include <cmath>

namespace visualiser::math {

float Clamp(float value, float lo, float hi) {
  return std::max(lo, std::min(value, hi));
}

Vec3 Normalize(const Vec3& value) {
  const float length = std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
  if (length <= 1e-6f) {
    return {0.0f, 1.0f, 0.0f};
  }
  return {value.x / length, value.y / length, value.z / length};
}

Vec3 Cross(const Vec3& a, const Vec3& b) {
  return {
      a.y * b.z - a.z * b.y,
      a.z * b.x - a.x * b.z,
      a.x * b.y - a.y * b.x,
  };
}

float Dot(const Vec3& a, const Vec3& b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

float WrapDegrees(float deg) {
  return std::atan2(std::sin(deg * kPi / 180.0f), std::cos(deg * kPi / 180.0f)) * 180.0f / kPi;
}

Quat NormalizeQuat(const Quat& quat) {
  const float length = std::sqrt(quat.w * quat.w + quat.x * quat.x + quat.y * quat.y + quat.z * quat.z);
  if (length <= 1e-6f) {
    return {};
  }
  return {quat.w / length, quat.x / length, quat.y / length, quat.z / length};
}

Quat MultiplyQuat(const Quat& a, const Quat& b) {
  return {
      a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
      a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
      a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
      a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
  };
}

Vec3 RotateVector(const Quat& quat, const Vec3& vector) {
  const Quat q = NormalizeQuat(quat);
  const Quat p{0.0f, vector.x, vector.y, vector.z};
  const Quat qc{q.w, -q.x, -q.y, -q.z};
  const Quat rotated = MultiplyQuat(MultiplyQuat(q, p), qc);
  return {rotated.x, rotated.y, rotated.z};
}

Vec3 RotateAroundSceneY(const Vec3& value, float yaw_rad) {
  const float c = std::cos(yaw_rad);
  const float s = std::sin(yaw_rad);
  return Vec3{
      value.x * c + value.z * s,
      value.y,
      -value.x * s + value.z * c,
  };
}

}  // namespace visualiser::math
