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

// Cross-product form: t = 2*(q_vec × v); v' = v + q.w*t + q_vec × t
// ~30 flops vs ~64 for the quat-multiply sandwich. Changes FP rounding
// order relative to the old form — baselines must be regenerated if used.
inline Vec3 Rotate(const Quat& q, const Vec3& v) {
    const float tx = 2.0f * (q.y * v.z - q.z * v.y);
    const float ty = 2.0f * (q.z * v.x - q.x * v.z);
    const float tz = 2.0f * (q.x * v.y - q.y * v.x);
    return {
        v.x + q.w * tx + (q.y * tz - q.z * ty),
        v.y + q.w * ty + (q.z * tx - q.x * tz),
        v.z + q.w * tz + (q.x * ty - q.y * tx),
    };
}

// Equivalent to Rotate(Conjugate(q), v) for unit quaternion q.
// Same cross-product form with q.w sign flipped.
inline Vec3 RotateInverse(const Quat& q, const Vec3& v) {
    const float tx = 2.0f * (q.y * v.z - q.z * v.y);
    const float ty = 2.0f * (q.z * v.x - q.x * v.z);
    const float tz = 2.0f * (q.x * v.y - q.y * v.x);
    return {
        v.x - q.w * tx + (q.y * tz - q.z * ty),
        v.y - q.w * ty + (q.z * tx - q.x * tz),
        v.z - q.w * tz + (q.x * ty - q.y * tx),
    };
}

} // namespace minphys3d
