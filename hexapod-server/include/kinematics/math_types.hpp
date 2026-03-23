#pragma once

#include <algorithm>

constexpr double kPi = 3.14159265358979323846;

template <typename Tag>
struct UnitValue {
    double value{0.0};

    constexpr UnitValue() = default;
    explicit constexpr UnitValue(double raw) : value(raw) {}

    constexpr UnitValue operator+(UnitValue rhs) const {
        return UnitValue{value + rhs.value};
    }

    constexpr UnitValue operator-(UnitValue rhs) const {
        return UnitValue{value - rhs.value};
    }

    constexpr UnitValue operator*(double scalar) const {
        return UnitValue{value * scalar};
    }

    constexpr UnitValue operator/(double scalar) const {
        return UnitValue{value / scalar};
    }

    constexpr UnitValue& operator+=(UnitValue rhs) {
        value += rhs.value;
        return *this;
    }

    constexpr UnitValue& operator-=(UnitValue rhs) {
        value -= rhs.value;
        return *this;
    }
};

template <typename Tag>
constexpr UnitValue<Tag> operator*(double scalar, UnitValue<Tag> unit) {
    return unit * scalar;
}

struct AngleRadTag {};
struct AngularRateRadPerSecTag {};
struct FrequencyHzTag {};
struct DurationSecTag {};
struct LengthMTag {};
struct LinearRateMpsTag {};

using AngleRad = UnitValue<AngleRadTag>;
using AngularRateRadPerSec = UnitValue<AngularRateRadPerSecTag>;
using FrequencyHz = UnitValue<FrequencyHzTag>;
using DurationSec = UnitValue<DurationSecTag>;
using LengthM = UnitValue<LengthMTag>;
using LinearRateMps = UnitValue<LinearRateMpsTag>;

double deg2rad(double deg);
double rad2deg(AngleRad rad);
double rad2deg(double rad);

double clamp(double value, double lo, double hi);
inline double clamp01(double value) {
    return std::clamp(value, 0.0, 1.0);
}

struct Vec3 {
    double x{0.0};
    double y{0.0};
    double z{0.0};

    Vec3 operator+(const Vec3& other) const;
    Vec3 operator-(const Vec3& other) const;
    Vec3 operator*(double s) const;
};

Vec3 cross(const Vec3& lhs, const Vec3& rhs);
double vecNorm(const Vec3& v);

template <typename Tag>
struct UnitVec3 {
    double x{0.0};
    double y{0.0};
    double z{0.0};

    constexpr UnitVec3() = default;
    constexpr UnitVec3(double x_in, double y_in, double z_in)
        : x(x_in), y(y_in), z(z_in) {}
    constexpr UnitVec3(const Vec3& vec)
        : x(vec.x), y(vec.y), z(vec.z) {}


    constexpr UnitVec3 operator+(const UnitVec3& rhs) const {
        return UnitVec3{x + rhs.x, y + rhs.y, z + rhs.z};
    }

    constexpr UnitVec3 operator-(const UnitVec3& rhs) const {
        return UnitVec3{x - rhs.x, y - rhs.y, z - rhs.z};
    }

    constexpr UnitVec3 operator*(double scalar) const {
        return UnitVec3{x * scalar, y * scalar, z * scalar};
    }

    constexpr UnitVec3& operator+=(const UnitVec3& rhs) {
        x += rhs.x;
        y += rhs.y;
        z += rhs.z;
        return *this;
    }

    constexpr UnitVec3& operator-=(const UnitVec3& rhs) {
        x -= rhs.x;
        y -= rhs.y;
        z -= rhs.z;
        return *this;
    }

    constexpr Vec3 raw() const {
        return Vec3{x, y, z};
    }

    constexpr operator Vec3() const {
        return raw();
    }
};

template <typename Tag>
constexpr UnitVec3<Tag> operator*(double scalar, const UnitVec3<Tag>& vec) {
    return vec * scalar;
}

struct PositionM3Tag {};
struct VelocityMps3Tag {};
struct EulerAnglesRad3Tag {};
struct AngularVelocityRadPerSec3Tag {};

using PositionM3 = UnitVec3<PositionM3Tag>;
using VelocityMps3 = UnitVec3<VelocityMps3Tag>;
using EulerAnglesRad3 = UnitVec3<EulerAnglesRad3Tag>;
using AngularVelocityRadPerSec3 = UnitVec3<AngularVelocityRadPerSec3Tag>;

struct Mat3 {
    double m[3][3]{};

    static Mat3 identity();
    static Mat3 rotX(double roll);
    static Mat3 rotY(double pitch);
    static Mat3 rotZ(double yaw);

    Mat3 transpose() const;

    Vec3 operator*(const Vec3& v) const;
    Mat3 operator*(const Mat3& other) const;
};
