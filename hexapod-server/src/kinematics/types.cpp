#include "types.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace {

template <typename LegStateT>
AngleRad leg_joint_angle_at(const LegStateT& leg, std::size_t idx);

template <>
AngleRad leg_joint_angle_at<LegState>(const LegState& leg,
                                      std::size_t idx) {
    return leg.joint_state[idx].pos_rad;
}

template <typename LegStateT>
void set_leg_joint_angle_at(LegStateT& leg, std::size_t idx, AngleRad angle);

template <>
void set_leg_joint_angle_at<LegState>(LegState& leg, std::size_t idx,
                                      AngleRad angle) {
    leg.joint_state[idx].pos_rad = angle;
}

double normalized_servo_sign(double sign) {
    if (!std::isfinite(sign)) {
        return 1.0;
    }
    return sign >= 0.0 ? 1.0 : -1.0;
}

template <typename LegStateT>
LegStateT convert_leg_angles(const ServoCalibration& calibration,
                             const LegStateT& input,
                             bool to_servo_angles) {
    static_assert(kJointsPerLeg == 3,
                  "Calibration conversion assumes coxa/femur/tibia ordering.");
    const std::array<double, kJointsPerLeg> signs{
        normalized_servo_sign(calibration.coxaSign),
        normalized_servo_sign(calibration.femurSign),
        normalized_servo_sign(calibration.tibiaSign)};
    const std::array<AngleRad, kJointsPerLeg> offsets{
        calibration.coxaOffset, calibration.femurOffset, calibration.tibiaOffset};

    LegStateT output{};
    for (std::size_t joint = 0; joint < kJointsPerLeg; ++joint) {
        const double input_value = leg_joint_angle_at(input, joint).value;
        const double transformed = to_servo_angles
                                       ? (signs[joint] * input_value + offsets[joint].value)
                                       : (signs[joint] * input_value - offsets[joint].value);
        set_leg_joint_angle_at(output, joint, AngleRad{transformed});
    }
    return output;
}

}  // namespace


// ============================================================
// Utility functions
// ============================================================
double deg2rad(double deg) {
    return deg * kPi / 180.0;
}

double rad2deg(double rad) {
    return rad * 180.0 / kPi;
}

double rad2deg(AngleRad rad) {
    return rad2deg(rad.value);
}

double clamp(double value, double lo, double hi) {
    if (value < lo) return lo;
    if (value > hi) return hi;
    return value;
}

double clampedDelta(double candidate, double limit_abs) {
    return std::clamp(candidate, -std::abs(limit_abs), std::abs(limit_abs));
}

double wrap01(double x) {
    x = std::fmod(x, 1.0);
    if (x < 0.0) {
        x += 1.0;
    }
    return x;
}

double normalizeAngleRad(double rad) {
    constexpr double kTwoPi = 2.0 * kPi;
    while (rad > kPi) {
        rad -= kTwoPi;
    }
    while (rad < -kPi) {
        rad += kTwoPi;
    }
    return rad;
}

double lerp(double a, double b, double t) {
    return a + ((b - a) * t);
}

double lerpAngleShortest(double from_rad, double to_rad, double t) {
    const double delta = normalizeAngleRad(to_rad - from_rad);
    return normalizeAngleRad(from_rad + (delta * t));
}

double smoothstep01(double t) {
    const double alpha = clamp01(t);
    return alpha * alpha * (3.0 - (2.0 * alpha));
}

double det3x3(double m00, double m01, double m02,
              double m10, double m11, double m12,
              double m20, double m21, double m22) {
    return m00 * (m11 * m22 - m12 * m21) -
           m01 * (m10 * m22 - m12 * m20) +
           m02 * (m10 * m21 - m11 * m20);
}

bool solve3x3Cramers(double m00, double m01, double m02,
                     double m10, double m11, double m12,
                     double m20, double m21, double m22,
                     double r0, double r1, double r2,
                     double& x0, double& x1, double& x2,
                     double singular_threshold) {
    const double det = det3x3(m00, m01, m02,
                              m10, m11, m12,
                              m20, m21, m22);
    if (std::abs(det) < singular_threshold) {
        return false;
    }

    const double det_x0 = det3x3(r0, m01, m02,
                                 r1, m11, m12,
                                 r2, m21, m22);
    const double det_x1 = det3x3(m00, r0, m02,
                                 m10, r1, m12,
                                 m20, r2, m22);
    const double det_x2 = det3x3(m00, m01, r0,
                                 m10, m11, r1,
                                 m20, m21, r2);

    x0 = det_x0 / det;
    x1 = det_x1 / det;
    x2 = det_x2 / det;
    return true;
}

// ============================================================
// Vec3
// ============================================================
Vec3 Vec3::operator+(const Vec3& other) const {
    return {x + other.x, y + other.y, z + other.z};
}

Vec3 Vec3::operator-(const Vec3& other) const {
    return {x - other.x, y - other.y, z - other.z};
}

Vec3 Vec3::operator*(double s) const {
    return {x * s, y * s, z * s};
}

Vec3 Vec3::operator-() const {
    return {-x, -y, -z};
}

Vec3 operator*(double s, const Vec3& v) {
    return v * s;
}

Vec3 cross(const Vec3& lhs, const Vec3& rhs) {
    return Vec3{
        lhs.y * rhs.z - lhs.z * rhs.y,
        lhs.z * rhs.x - lhs.x * rhs.z,
        lhs.x * rhs.y - lhs.y * rhs.x,
    };
}

double vecNorm(const Vec3& v) {
    return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

double vecNormSquared(const Vec3& v) {
    return v.x * v.x + v.y * v.y + v.z * v.z;
}

bool isFinite(const Vec3& vec) {
    return std::isfinite(vec.x) && std::isfinite(vec.y) && std::isfinite(vec.z);
}

Vec3 lerp(const Vec3& a, const Vec3& b, double t) {
    return Vec3{
        lerp(a.x, b.x, t),
        lerp(a.y, b.y, t),
        lerp(a.z, b.z, t),
    };
}

double cross(const Point2& o, const Point2& a, const Point2& b) {
    return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
}

double edgeSignedDistance(const Point2& a, const Point2& b, const Point2& p) {
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double len = std::hypot(dx, dy);
    if (len <= 1e-9) {
        return 0.0;
    }
    return (dx * (p.y - a.y) - dy * (p.x - a.x)) / len;
}

double polygonSignedArea(const std::vector<Point2>& polygon) {
    double area = 0.0;
    for (std::size_t i = 0; i < polygon.size(); ++i) {
        const Point2& a = polygon[i];
        const Point2& b = polygon[(i + 1) % polygon.size()];
        area += (a.x * b.y) - (b.x * a.y);
    }
    return 0.5 * area;
}

std::vector<Point2> convexHull(std::vector<Point2> points) {
    if (points.size() <= 1) {
        return points;
    }

    std::sort(points.begin(), points.end(), [](const Point2& lhs, const Point2& rhs) {
        if (lhs.x == rhs.x) {
            return lhs.y < rhs.y;
        }
        return lhs.x < rhs.x;
    });

    std::vector<Point2> hull;
    hull.reserve(points.size() * 2);

    for (const Point2& point : points) {
        while (hull.size() >= 2 && cross(hull[hull.size() - 2], hull[hull.size() - 1], point) <= 0.0) {
            hull.pop_back();
        }
        hull.push_back(point);
    }

    const std::size_t lower_size = hull.size();
    for (auto it = points.rbegin() + 1; it != points.rend(); ++it) {
        while (hull.size() > lower_size && cross(hull[hull.size() - 2], hull[hull.size() - 1], *it) <= 0.0) {
            hull.pop_back();
        }
        hull.push_back(*it);
    }

    if (!hull.empty()) {
        hull.pop_back();
    }
    return hull;
}

// ============================================================
// Mat3
// ============================================================
Mat3 Mat3::identity() {
    Mat3 R{};
    R.m[0][0] = 1.0; R.m[0][1] = 0.0; R.m[0][2] = 0.0;
    R.m[1][0] = 0.0; R.m[1][1] = 1.0; R.m[1][2] = 0.0;
    R.m[2][0] = 0.0; R.m[2][1] = 0.0; R.m[2][2] = 1.0;
    return R;
}

Mat3 Mat3::rotX(double roll) {
    Mat3 R = identity();
    const double c = std::cos(roll);
    const double s = std::sin(roll);

    R.m[1][1] = c;  R.m[1][2] = -s;
    R.m[2][1] = s;  R.m[2][2] = c;
    return R;
}

Mat3 Mat3::rotY(double pitch) {
    Mat3 R = identity();
    const double c = std::cos(pitch);
    const double s = std::sin(pitch);

    R.m[0][0] = c;   R.m[0][2] = s;
    R.m[2][0] = -s;  R.m[2][2] = c;
    return R;
}

Mat3 Mat3::rotZ(double yaw) {
    Mat3 R = identity();
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);

    R.m[0][0] = c;  R.m[0][1] = -s;
    R.m[1][0] = s;  R.m[1][1] = c;
    return R;
}

Mat3 Mat3::transpose() const {
    Mat3 T{};
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            T.m[r][c] = m[c][r];
        }
    }
    return T;
}

Vec3 Mat3::operator*(const Vec3& v) const {
    return {
        m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
        m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
        m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z
    };
}

Mat3 Mat3::operator*(const Mat3& other) const {
    Mat3 out{};
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            out.m[r][c] =
                m[r][0] * other.m[0][c] +
                m[r][1] * other.m[1][c] +
                m[r][2] * other.m[2][c];
        }
    }
    return out;
}

// ============================================================
// BodyPose
// ============================================================
Mat3 BodyPose::rotationBodyToWorld() const {
    return Mat3::rotZ(yaw.value) * Mat3::rotY(pitch.value) * Mat3::rotX(roll.value);
}

// ============================================================
// ServoCalibration
// ============================================================
LegState ServoCalibration::toServoAngles(const LegState& leg) const {
    return convert_leg_angles(*this, leg, true);
}

LegState ServoCalibration::toJointAngles(const LegState& leg) const {
    return convert_leg_angles(*this, leg, false);
}
