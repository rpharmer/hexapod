#pragma once

#include <cmath>

#include "minphys3d/math/vec3.hpp"

namespace minphys3d {

// Default: Hamilton sandwich q * (0,v) * conj(q) / conj(q) * (0,v) * q (numerically
// well-behaved relative ordering vs the cross-product shortcut).
// Define MINPHYS3D_USE_FAST_QUAT_VECTOR_ROTATE to 1 (e.g. -D on the compile line) to use
// the cheaper cross-product form; changes floating-point results vs the sandwich.
#if defined(MINPHYS3D_USE_FAST_QUAT_VECTOR_ROTATE) && MINPHYS3D_USE_FAST_QUAT_VECTOR_ROTATE
#define MINPHYS3D_DETAIL_QUAT_VECTOR_ROTATE_FAST 1
#else
#define MINPHYS3D_DETAIL_QUAT_VECTOR_ROTATE_FAST 0
#endif

struct Quat {
    Real w = 1.0;
    Real x = 0.0;
    Real y = 0.0;
    Real z = 0.0;

    Quat() = default;
    Quat(Real w_, Real x_, Real y_, Real z_) : w(w_), x(x_), y(y_), z(z_) {}

    Quat operator+(const Quat& rhs) const { return {w + rhs.w, x + rhs.x, y + rhs.y, z + rhs.z}; }
    Quat operator*(Real s) const { return {w * s, x * s, y * s, z * s}; }
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
    const Real len = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (len <= kEpsilon) return {1.0, 0.0, 0.0, 0.0};
    return {q.w / len, q.x / len, q.y / len, q.z / len};
}

#if MINPHYS3D_DETAIL_QUAT_VECTOR_ROTATE_FAST
// Cross-product form: t = 2*(q_vec × v); v' = v + q.w*t + q_vec × t
inline Vec3 Rotate(const Quat& q, const Vec3& v) {
    const Real tx = 2.0 * (q.y * v.z - q.z * v.y);
    const Real ty = 2.0 * (q.z * v.x - q.x * v.z);
    const Real tz = 2.0 * (q.x * v.y - q.y * v.x);
    return {
        v.x + q.w * tx + (q.y * tz - q.z * ty),
        v.y + q.w * ty + (q.z * tx - q.x * tz),
        v.z + q.w * tz + (q.x * ty - q.y * tx),
    };
}

inline Vec3 RotateInverse(const Quat& q, const Vec3& v) {
    const Real tx = 2.0 * (q.y * v.z - q.z * v.y);
    const Real ty = 2.0 * (q.z * v.x - q.x * v.z);
    const Real tz = 2.0 * (q.x * v.y - q.y * v.x);
    return {
        v.x - q.w * tx + (q.y * tz - q.z * ty),
        v.y - q.w * ty + (q.z * tx - q.x * tz),
        v.z - q.w * tz + (q.x * ty - q.y * tx),
    };
}
#else
inline Vec3 Rotate(const Quat& q, const Vec3& v) {
    const Quat p{0.0, v.x, v.y, v.z};
    const Quat t = q * p * Conjugate(q);
    return {t.x, t.y, t.z};
}

inline Vec3 RotateInverse(const Quat& q, const Vec3& v) {
    const Quat qc = Conjugate(q);
    const Quat p{0.0, v.x, v.y, v.z};
    const Quat t = qc * p * q;
    return {t.x, t.y, t.z};
}
#endif

#undef MINPHYS3D_DETAIL_QUAT_VECTOR_ROTATE_FAST

} // namespace minphys3d
