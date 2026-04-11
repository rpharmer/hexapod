#pragma once

#include <cmath>

#include "minphys3d/math/quat.hpp"

namespace minphys3d {

struct Mat3 {
    float m[3][3]{};

    static Mat3 Identity() {
        Mat3 r{};
        r.m[0][0] = 1.0f;
        r.m[1][1] = 1.0f;
        r.m[2][2] = 1.0f;
        return r;
    }
};

inline Vec3 operator*(const Mat3& A, const Vec3& v) {
    return {
        A.m[0][0] * v.x + A.m[0][1] * v.y + A.m[0][2] * v.z,
        A.m[1][0] * v.x + A.m[1][1] * v.y + A.m[1][2] * v.z,
        A.m[2][0] * v.x + A.m[2][1] * v.y + A.m[2][2] * v.z,
    };
}

inline Mat3 operator*(const Mat3& A, const Mat3& B) {
    Mat3 r{};
    for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) for (int k = 0; k < 3; ++k) r.m[i][j] += A.m[i][k] * B.m[k][j];
    return r;
}

inline Mat3 Transpose(const Mat3& A) {
    Mat3 r{};
    for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) r.m[i][j] = A.m[j][i];
    return r;
}

inline Mat3 operator+(const Mat3& A, const Mat3& B) {
    Mat3 r{};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            r.m[i][j] = A.m[i][j] + B.m[i][j];
        }
    }
    return r;
}

inline Mat3 operator-(const Mat3& A, const Mat3& B) {
    Mat3 r{};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            r.m[i][j] = A.m[i][j] - B.m[i][j];
        }
    }
    return r;
}

inline Mat3 operator*(float s, const Mat3& M) {
    Mat3 r{};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            r.m[i][j] = s * M.m[i][j];
        }
    }
    return r;
}

inline Mat3 OuterProduct(const Vec3& u, const Vec3& v) {
    Mat3 r{};
    r.m[0][0] = u.x * v.x;
    r.m[0][1] = u.x * v.y;
    r.m[0][2] = u.x * v.z;
    r.m[1][0] = u.y * v.x;
    r.m[1][1] = u.y * v.y;
    r.m[1][2] = u.y * v.z;
    r.m[2][0] = u.z * v.x;
    r.m[2][1] = u.z * v.y;
    r.m[2][2] = u.z * v.z;
    return r;
}

inline Mat3 ScaleIdentity(float s) {
    Mat3 r{};
    r.m[0][0] = s;
    r.m[1][1] = s;
    r.m[2][2] = s;
    return r;
}

// General 3×3 inverse; returns false if singular.
inline bool InvertMat3(const Mat3& a, Mat3& out) {
    const float c00 = a.m[1][1] * a.m[2][2] - a.m[1][2] * a.m[2][1];
    const float c01 = a.m[1][2] * a.m[2][0] - a.m[1][0] * a.m[2][2];
    const float c02 = a.m[1][0] * a.m[2][1] - a.m[1][1] * a.m[2][0];

    const float det = a.m[0][0] * c00 + a.m[0][1] * c01 + a.m[0][2] * c02;
    if (!(std::abs(det) > 1e-18f)) {
        return false;
    }

    const float invDet = 1.0f / det;

    const float c10 = a.m[0][2] * a.m[2][1] - a.m[0][1] * a.m[2][2];
    const float c11 = a.m[0][0] * a.m[2][2] - a.m[0][2] * a.m[2][0];
    const float c12 = a.m[0][1] * a.m[2][0] - a.m[0][0] * a.m[2][1];

    const float c20 = a.m[0][1] * a.m[1][2] - a.m[0][2] * a.m[1][1];
    const float c21 = a.m[0][2] * a.m[1][0] - a.m[0][0] * a.m[1][2];
    const float c22 = a.m[0][0] * a.m[1][1] - a.m[0][1] * a.m[1][0];

    out.m[0][0] = c00 * invDet;
    out.m[0][1] = c10 * invDet;
    out.m[0][2] = c20 * invDet;
    out.m[1][0] = c01 * invDet;
    out.m[1][1] = c11 * invDet;
    out.m[1][2] = c21 * invDet;
    out.m[2][0] = c02 * invDet;
    out.m[2][1] = c12 * invDet;
    out.m[2][2] = c22 * invDet;
    return true;
}

inline Mat3 RotationMatrix(const Quat& q_) {
    const Quat q = Normalize(q_);
    const float xx = q.x * q.x;
    const float yy = q.y * q.y;
    const float zz = q.z * q.z;
    const float xy = q.x * q.y;
    const float xz = q.x * q.z;
    const float yz = q.y * q.z;
    const float wx = q.w * q.x;
    const float wy = q.w * q.y;
    const float wz = q.w * q.z;

    Mat3 r{};
    r.m[0][0] = 1.0f - 2.0f * (yy + zz);
    r.m[0][1] = 2.0f * (xy - wz);
    r.m[0][2] = 2.0f * (xz + wy);
    r.m[1][0] = 2.0f * (xy + wz);
    r.m[1][1] = 1.0f - 2.0f * (xx + zz);
    r.m[1][2] = 2.0f * (yz - wx);
    r.m[2][0] = 2.0f * (xz - wy);
    r.m[2][1] = 2.0f * (yz + wx);
    r.m[2][2] = 1.0f - 2.0f * (xx + yy);
    return r;
}

} // namespace minphys3d
