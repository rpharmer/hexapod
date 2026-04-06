#pragma once

#include <cmath>

#include "minphys3d/math/vec3.hpp"

namespace minphys3d {

struct Quat {
    float w = 1.0f;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    Quat() = default;
    Quat(float w_, float x_, float y_, float z_) : w(w_), x(x_), y(y_), z(z_) {}

    Quat operator+(const Quat& rhs) const { return {w + rhs.w, x + rhs.x, y + rhs.y, z + rhs.z}; }
    Quat operator*(float s) const { return {w * s, x * s, y * s, z * s}; }
};

inline Quat operator*(const Quat& a, const Quat& b) {
    return {
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
    };
}

inline Quat Conjugate(const Quat& q) { return {q.w, -q.x, -q.y, -q.z}; }
inline Quat Normalize(const Quat& q) {
    const float len = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (len <= kEpsilon) return {1.0f, 0.0f, 0.0f, 0.0f};
    return {q.w / len, q.x / len, q.y / len, q.z / len};
}

inline Vec3 Rotate(const Quat& q, const Vec3& v) {
    const Quat p{0.0f, v.x, v.y, v.z};
    const Quat r = q * p * Conjugate(q);
    return {r.x, r.y, r.z};
}

} // namespace minphys3d
