#pragma once

#include "types.hpp"

enum class MotionLimiterState {
    IDLE,
    BODY_PRELOAD,
    LOCOMOTING,
    BODY_SETTLE,
};

struct MotionLimiterOutput {
    MotionIntent intent{};
    MotionLimiterState state{MotionLimiterState::IDLE};
};

class MotionLimiter {
public:
    MotionLimiterOutput update(const MotionIntent& raw_intent, const SafetyState& safety);
    MotionLimiterState state() const { return state_; }

private:
    static constexpr double kPlanarSpeedEnterMps = 0.04;
    static constexpr double kPlanarSpeedExitMps = 0.02;
    static constexpr double kYawRateEnterRadps = 0.20;
    static constexpr double kYawRateExitRadps = 0.12;

    static constexpr uint64_t kCommandHysteresisMs = 80;
    static constexpr uint64_t kBodyPreloadTimeoutMs = 350;
    static constexpr uint64_t kBodySettleHoldMs = 120;
    static constexpr uint64_t kBodySettleTimeoutMs = 800;

    static constexpr double kBodyTranslationAccelMps2 = 0.80;
    static constexpr double kBodyTranslationMaxVelMps = 0.25;
    static constexpr double kBodyOrientationAccelRadps2 = 2.00;
    static constexpr double kBodyOrientationMaxVelRadps = 0.80;
    static constexpr double kPlanarVelocityAccelMps2 = 0.65;
    static constexpr double kYawRateAccelRadps2 = 1.60;

    static constexpr double kBodyTranslationSettleEpsM = 0.01;
    static constexpr double kBodyOrientationSettleEpsRad = 0.05;

    static double updateAxis(double target,
                             double dt_s,
                             double accel_limit,
                             double vel_limit,
                             double* value,
                             double* velocity);
    static bool bodyPoseNear(const BodyPoseKinematics& lhs, const BodyPoseKinematics& rhs);

    MotionLimiterState state_{MotionLimiterState::IDLE};
    TimePointUs last_update_us_{};
    TimePointUs state_enter_us_{};
    TimePointUs motion_request_since_us_{};
    TimePointUs motion_clear_since_us_{};

    BodyPoseKinematics filtered_body_{};
    Vec3 filtered_body_trans_vel_mps_{};
    Vec3 filtered_body_orientation_vel_radps_{};

    Vec3 filtered_planar_velocity_mps_{};
    Vec3 filtered_planar_velocity_dot_mps2_{};
    double filtered_yaw_rate_radps_{0.0};
    double filtered_yaw_rate_dot_radps2_{0.0};

    bool settle_stride_halted_{false};
    BodyPoseKinematics body_settle_pose_{};
};
