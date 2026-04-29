#include "visualiser/render/mat4.hpp"

#include <algorithm>

namespace visualiser::render {

Mat3 Mat3::Identity() {
  Mat3 r{};
  r.m[0] = r.m[4] = r.m[8] = 1.0f;
  return r;
}

Mat3 Mat3::FromMat4Upper(const float m4[16]) {
  Mat3 r{};
  r.m[0] = m4[0];
  r.m[1] = m4[1];
  r.m[2] = m4[2];
  r.m[3] = m4[4];
  r.m[4] = m4[5];
  r.m[5] = m4[6];
  r.m[6] = m4[8];
  r.m[7] = m4[9];
  r.m[8] = m4[10];
  return r;
}

Mat3 Mat3::Transpose(const Mat3& a) {
  Mat3 r{};
  r.m[0] = a.m[0];
  r.m[1] = a.m[3];
  r.m[2] = a.m[6];
  r.m[3] = a.m[1];
  r.m[4] = a.m[4];
  r.m[5] = a.m[7];
  r.m[6] = a.m[2];
  r.m[7] = a.m[5];
  r.m[8] = a.m[8];
  return r;
}

Mat3 Mat3::Inverse(const Mat3& m) {
  const float a = m.m[0], b = m.m[1], c = m.m[2];
  const float d = m.m[3], e = m.m[4], f = m.m[5];
  const float g = m.m[6], h = m.m[7], i = m.m[8];
  const float A = e * i - f * h;
  const float B = -(d * i - f * g);
  const float C = d * h - e * g;
  const float D = -(b * i - c * h);
  const float E = a * i - c * g;
  const float F = -(a * h - b * g);
  const float G = b * f - c * e;
  const float H = -(a * f - c * d);
  const float I = a * e - b * d;
  float det = a * A + b * B + c * C;
  if (std::fabs(det) < 1e-12f) {
    return Identity();
  }
  det = 1.0f / det;
  Mat3 r{};
  r.m[0] = A * det;
  r.m[1] = D * det;
  r.m[2] = G * det;
  r.m[3] = B * det;
  r.m[4] = E * det;
  r.m[5] = H * det;
  r.m[6] = C * det;
  r.m[7] = F * det;
  r.m[8] = I * det;
  return r;
}

Mat3 Mat3::Mul(const Mat3& a, const Mat3& b) {
  Mat3 r{};
  for (int col = 0; col < 3; ++col) {
    for (int row = 0; row < 3; ++row) {
      r.m[col * 3 + row] = a.m[0 * 3 + row] * b.m[col * 3 + 0] + a.m[1 * 3 + row] * b.m[col * 3 + 1] +
                           a.m[2 * 3 + row] * b.m[col * 3 + 2];
    }
  }
  return r;
}

Mat4 Mat4::Identity() {
  Mat4 r{};
  r.m[0] = r.m[5] = r.m[10] = r.m[15] = 1.0f;
  return r;
}

Mat4 Mat4::Translate(float x, float y, float z) {
  Mat4 r = Identity();
  r.m[12] = x;
  r.m[13] = y;
  r.m[14] = z;
  return r;
}

Mat4 Mat4::Scale(float x, float y, float z) {
  Mat4 r = Identity();
  r.m[0] = x;
  r.m[5] = y;
  r.m[10] = z;
  return r;
}

Mat4 Mat4::RotateX(float radians) {
  const float c = std::cos(radians);
  const float s = std::sin(radians);
  Mat4 r = Identity();
  r.m[5] = c;
  r.m[6] = s;
  r.m[9] = -s;
  r.m[10] = c;
  return r;
}

Mat4 Mat4::RotateY(float radians) {
  const float c = std::cos(radians);
  const float s = std::sin(radians);
  Mat4 r = Identity();
  r.m[0] = c;
  r.m[2] = -s;
  r.m[8] = s;
  r.m[10] = c;
  return r;
}

Mat4 Mat4::RotateZ(float radians) {
  const float c = std::cos(radians);
  const float s = std::sin(radians);
  Mat4 r = Identity();
  r.m[0] = c;
  r.m[1] = s;
  r.m[4] = -s;
  r.m[5] = c;
  return r;
}

Mat4 Mat4::FromQuat(float w, float x, float y, float z) {
  const float xx = x * x;
  const float yy = y * y;
  const float zz = z * z;
  const float xy = x * y;
  const float xz = x * z;
  const float yz = y * z;
  const float wx = w * x;
  const float wy = w * y;
  const float wz = w * z;
  Mat4 r = Identity();
  r.m[0] = 1.0f - 2.0f * (yy + zz);
  r.m[1] = 2.0f * (xy + wz);
  r.m[2] = 2.0f * (xz - wy);
  r.m[4] = 2.0f * (xy - wz);
  r.m[5] = 1.0f - 2.0f * (xx + zz);
  r.m[6] = 2.0f * (yz + wx);
  r.m[8] = 2.0f * (xz + wy);
  r.m[9] = 2.0f * (yz - wx);
  r.m[10] = 1.0f - 2.0f * (xx + yy);
  return r;
}

Mat4 Mat4::FrustumSymmetric(float right, float top, float near_plane, float far_plane) {
  Mat4 r{};
  const float n = near_plane;
  const float f = far_plane;
  r.m[0] = n / right;
  r.m[5] = n / top;
  r.m[10] = -(f + n) / (f - n);
  r.m[11] = -1.0f;
  r.m[14] = -(2.0f * f * n) / (f - n);
  return r;
}

Mat4 Mat4::Mul(const Mat4& a, const Mat4& b) {
  Mat4 r{};
  for (int c = 0; c < 4; ++c) {
    for (int row = 0; row < 4; ++row) {
      r.m[c * 4 + row] = a.m[0 * 4 + row] * b.m[c * 4 + 0] + a.m[1 * 4 + row] * b.m[c * 4 + 1] +
                         a.m[2 * 4 + row] * b.m[c * 4 + 2] + a.m[3 * 4 + row] * b.m[c * 4 + 3];
    }
  }
  return r;
}

Mat4 Mat4::Transpose(const Mat4& a) {
  Mat4 r{};
  for (int c = 0; c < 4; ++c) {
    for (int row = 0; row < 4; ++row) {
      r.m[row * 4 + c] = a.m[c * 4 + row];
    }
  }
  return r;
}

Vec3 Mat4::TransformPoint(const Vec3& v) const {
  const float x = m[0] * v.x + m[4] * v.y + m[8] * v.z + m[12];
  const float y = m[1] * v.x + m[5] * v.y + m[9] * v.z + m[13];
  const float z = m[2] * v.x + m[6] * v.y + m[10] * v.z + m[14];
  const float w = m[3] * v.x + m[7] * v.y + m[11] * v.z + m[15];
  if (std::fabs(w - 1.0f) > 1e-6f && std::fabs(w) > 1e-12f) {
    const float inv = 1.0f / w;
    return {x * inv, y * inv, z * inv};
  }
  return {x, y, z};
}

Vec3 Normalize(const Vec3& v) {
  const float len = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
  if (len <= 1e-12f) {
    return {0.0f, 1.0f, 0.0f};
  }
  const float inv = 1.0f / len;
  return {v.x * inv, v.y * inv, v.z * inv};
}

Vec3 TransformDirection(const Mat4& m, const Vec3& d) {
  const float x = m.m[0] * d.x + m.m[4] * d.y + m.m[8] * d.z;
  const float y = m.m[1] * d.x + m.m[5] * d.y + m.m[9] * d.z;
  const float z = m.m[2] * d.x + m.m[6] * d.y + m.m[10] * d.z;
  return Normalize({x, y, z});
}

}  // namespace visualiser::render
