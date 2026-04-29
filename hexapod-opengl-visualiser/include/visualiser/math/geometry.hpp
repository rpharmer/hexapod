#pragma once

#include "visualiser/math/vec3.hpp"

namespace visualiser::math {

constexpr float kPi = 3.14159265358979323846f;

float Clamp(float value, float lo, float hi);
Vec3 Normalize(const Vec3& value);
Vec3 Cross(const Vec3& a, const Vec3& b);
float Dot(const Vec3& a, const Vec3& b);
float WrapDegrees(float deg);
Quat NormalizeQuat(const Quat& quat);
Quat MultiplyQuat(const Quat& a, const Quat& b);
Vec3 RotateVector(const Quat& quat, const Vec3& vector);
Vec3 RotateAroundSceneY(const Vec3& value, float yaw_rad);

}  // namespace visualiser::math
