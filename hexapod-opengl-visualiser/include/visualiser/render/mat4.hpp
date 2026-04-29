#pragma once

#include <cmath>
#include <cstring>

namespace visualiser::render {

struct Vec3 {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};

struct Mat3 {
  float m[9]{};

  static Mat3 Identity();
  static Mat3 FromMat4Upper(const float m4[16]);
  static Mat3 Transpose(const Mat3& a);
  static Mat3 Inverse(const Mat3& a);
  static Mat3 Mul(const Mat3& a, const Mat3& b);
};

struct Mat4 {
  float m[16]{};

  static Mat4 Identity();
  static Mat4 Translate(float x, float y, float z);
  static Mat4 Scale(float x, float y, float z);
  static Mat4 RotateX(float radians);
  static Mat4 RotateY(float radians);
  static Mat4 RotateZ(float radians);
  /// Column-major rotation from unit quaternion (w,x,y,z).
  static Mat4 FromQuat(float w, float x, float y, float z);
  /// Symmetric glFrustum: left=-right, bottom=-top.
  static Mat4 FrustumSymmetric(float right, float top, float near_plane, float far_plane);
  static Mat4 Mul(const Mat4& a, const Mat4& b);
  static Mat4 Transpose(const Mat4& a);
  Vec3 TransformPoint(const Vec3& v) const;
};

Vec3 Normalize(const Vec3& v);
/// Direction (ignore translation) using upper-left 3x3 of an affine matrix.
Vec3 TransformDirection(const Mat4& m, const Vec3& d);

constexpr float kPi = 3.14159265358979323846f;

inline float DegToRad(float d) { return d * kPi / 180.0f; }

}  // namespace visualiser::render
