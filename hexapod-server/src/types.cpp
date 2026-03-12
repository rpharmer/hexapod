#include "types.hpp"

#include <cmath>


// ============================================================
// Utility functions
// ============================================================
double deg2rad(double deg) {
    return deg * kPi / 180.0;
}

double rad2deg(double rad) {
    return rad * 180.0 / kPi;
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
// ServoCalibration
// ============================================================
LegRawState ServoCalibration::toServoAngles(const LegRawState& leg) const {
    const double coxaAngle = leg.joint_raw_state[0].pos_rad;
    const double femurAngle = leg.joint_raw_state[1].pos_rad;
    const double tibiaAngle = leg.joint_raw_state[2].pos_rad;
    
    LegRawState legServos{};
    
    legServos.joint_raw_state[0].pos_rad = coxaSign * coxaAngle + coxaOffset;
    legServos.joint_raw_state[1].pos_rad = femurSign * femurAngle + femurOffset;
    legServos.joint_raw_state[2].pos_rad = tibiaSign * tibiaAngle + tibiaOffset;
    
    return legServos;
}
LegState ServoCalibration::toServoAngles(const LegState& leg) const {
    const double coxaAngle = leg.joint_state[0].pos_rad;
    const double femurAngle = leg.joint_state[1].pos_rad;
    const double tibiaAngle = leg.joint_state[2].pos_rad;
    
    LegState legServos{};
    
    legServos.joint_state[0].pos_rad = coxaSign * coxaAngle + coxaOffset;
    legServos.joint_state[1].pos_rad = femurSign * femurAngle + femurOffset;
    legServos.joint_state[2].pos_rad = tibiaSign * tibiaAngle + tibiaOffset;
    
    return legServos;
}

LegRawState ServoCalibration::toJointAngles(const LegRawState& leg) const {
    const double coxaAngle = leg.joint_raw_state[0].pos_rad;
    const double femurAngle = leg.joint_raw_state[1].pos_rad;
    const double tibiaAngle = leg.joint_raw_state[2].pos_rad;
    
    LegRawState legServos{};
    
    legServos.joint_raw_state[0].pos_rad = coxaSign * coxaAngle - coxaOffset;
    legServos.joint_raw_state[1].pos_rad = femurSign * femurAngle - femurOffset;
    legServos.joint_raw_state[2].pos_rad = tibiaSign * tibiaAngle - tibiaOffset;
    
    return legServos;
}
LegState ServoCalibration::toJointAngles(const LegState& leg) const {
    const double coxaAngle = leg.joint_state[0].pos_rad;
    const double femurAngle = leg.joint_state[1].pos_rad;
    const double tibiaAngle = leg.joint_state[2].pos_rad;
    
    LegState legServos{};
    
    legServos.joint_state[0].pos_rad = coxaSign * coxaAngle - coxaOffset;
    legServos.joint_state[1].pos_rad = femurSign * femurAngle - femurOffset;
    legServos.joint_state[2].pos_rad = tibiaSign * tibiaAngle - tibiaOffset;
    
    return legServos;
}