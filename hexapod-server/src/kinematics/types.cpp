#include "types.hpp"

#include <cmath>

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
                                       // Inverse of servo = sign * joint + offset is:
                                       // joint = sign * (servo - offset), with sign constrained to +/-1.
                                       : (signs[joint] * (input_value - offsets[joint].value));
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
