#pragma once

#include <cmath>

#include "minphys3d/math/scalar.hpp"

namespace minphys3d {

struct Vec3 {
    Real x = 0.0;
    Real y = 0.0;
    Real z = 0.0;

    Vec3() = default;
    Vec3(Real x_, Real y_, Real z_) : x(x_), y(y_), z(z_) {}

    Vec3 operator+(const Vec3& rhs) const { return {x + rhs.x, y + rhs.y, z + rhs.z}; }
    Vec3 operator-(const Vec3& rhs) const { return {x - rhs.x, y - rhs.y, z - rhs.z}; }
    Vec3 operator-() const { return {-x, -y, -z}; }
    Vec3 operator*(Real s) const { return {x * s, y * s, z * s}; }
    Vec3 operator/(Real s) const { return {x / s, y / s, z / s}; }

    Vec3& operator+=(const Vec3& rhs) { x += rhs.x; y += rhs.y; z += rhs.z; return *this; }
    Vec3& operator-=(const Vec3& rhs) { x -= rhs.x; y -= rhs.y; z -= rhs.z; return *this; }
    Vec3& operator*=(Real s) { x *= s; y *= s; z *= s; return *this; }
};

inline Vec3 operator*(Real s, const Vec3& v) { return {v.x * s, v.y * s, v.z * s}; }
inline Real Dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 Cross(const Vec3& a, const Vec3& b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
inline Real LengthSquared(const Vec3& v) { return Dot(v, v); }
inline Real Length(const Vec3& v) { return std::sqrt(LengthSquared(v)); }
inline bool TryNormalize(const Vec3& in, Vec3& out) {
    const Real len = Length(in);
    if (len <= kEpsilon) {
        out = {};
        return false;
    }
    out = in / len;
    return true;
}
inline Vec3 Normalize(const Vec3& v) {
    Vec3 out{};
    if (!TryNormalize(v, out)) return {1.0, 0.0, 0.0};
    return out;
}

} // namespace minphys3d
