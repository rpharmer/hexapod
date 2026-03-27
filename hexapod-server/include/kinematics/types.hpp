#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <string>

#include "protocol_layout.hpp"
#include "kinematics/math_types.hpp"

// ============================================================

constexpr int COXA = static_cast<int>(LegJointID::Coxa);
constexpr int FEMUR = static_cast<int>(LegJointID::Femur);
constexpr int TIBIA = static_cast<int>(LegJointID::Tibia);

constexpr int kNumLegs = static_cast<int>(kProtocolLegCount);
constexpr int kJointsPerLeg = static_cast<int>(kProtocolJointsPerLeg);
constexpr int kNumJoints = static_cast<int>(kProtocolJointCount);

using Clock = std::chrono::steady_clock;

struct TimePointUs {
    uint64_t value{0};

    constexpr bool isZero() const { return value == 0; }
};

struct DurationUs {
    uint64_t value{0};
};

struct DurationMs {
    uint64_t value{0};

    constexpr DurationMs() = default;
    constexpr DurationMs(uint64_t raw_value) : value(raw_value) {}
    constexpr operator uint64_t() const { return value; }
};

struct TimestampMs {
    uint64_t value{0};

    constexpr TimestampMs() = default;
    constexpr TimestampMs(uint64_t raw_value) : value(raw_value) {}
    constexpr operator uint64_t() const { return value; }
};

inline TimePointUs now_us() {
    const auto t = Clock::now().time_since_epoch();
    return TimePointUs{
        static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(t).count())
    };
}

inline DurationUs operator-(TimePointUs lhs, TimePointUs rhs) {
    if (lhs.value <= rhs.value) {
        return DurationUs{};
    }
    return DurationUs{lhs.value - rhs.value};
}

inline bool operator>(DurationUs lhs, DurationUs rhs) {
    return lhs.value > rhs.value;
}

constexpr bool operator==(TimestampMs lhs, TimestampMs rhs) { return lhs.value == rhs.value; }
constexpr bool operator!=(TimestampMs lhs, TimestampMs rhs) { return !(lhs == rhs); }
constexpr bool operator<(TimestampMs lhs, TimestampMs rhs) { return lhs.value < rhs.value; }
constexpr bool operator<=(TimestampMs lhs, TimestampMs rhs) { return lhs.value <= rhs.value; }
constexpr bool operator>(TimestampMs lhs, TimestampMs rhs) { return rhs < lhs; }
constexpr bool operator>=(TimestampMs lhs, TimestampMs rhs) { return rhs <= lhs; }

constexpr bool operator>(TimestampMs lhs, uint64_t rhs) { return lhs.value > rhs; }
constexpr bool operator>=(TimestampMs lhs, uint64_t rhs) { return lhs.value >= rhs; }
constexpr bool operator<(TimestampMs lhs, uint64_t rhs) { return lhs.value < rhs; }
constexpr bool operator<=(TimestampMs lhs, uint64_t rhs) { return lhs.value <= rhs; }
constexpr bool operator>(uint64_t lhs, TimestampMs rhs) { return lhs > rhs.value; }
constexpr bool operator>=(uint64_t lhs, TimestampMs rhs) { return lhs >= rhs.value; }
constexpr bool operator<(uint64_t lhs, TimestampMs rhs) { return lhs < rhs.value; }
constexpr bool operator<=(uint64_t lhs, TimestampMs rhs) { return lhs <= rhs.value; }

constexpr DurationMs operator-(TimestampMs lhs, TimestampMs rhs) {
    return DurationMs{lhs.value <= rhs.value ? 0 : lhs.value - rhs.value};
}
constexpr DurationMs operator-(uint64_t lhs, TimestampMs rhs) {
    return DurationMs{lhs <= rhs.value ? 0 : lhs - rhs.value};
}
constexpr TimestampMs operator+(TimestampMs lhs, uint64_t rhs) {
    return TimestampMs{lhs.value + rhs};
}

template <typename T>
struct StreamSample {
    T value{};
    uint64_t sample_id{0};
    TimePointUs timestamp_us{};
};

enum class RobotMode {
    SAFE_IDLE,
    HOMING,
    STAND,
    WALK,
    FAULT
};

enum class GaitType {
    TRIPOD,
    RIPPLE,
    WAVE
};

struct GaitState {
    std::array<double, kNumLegs> phase{};
    std::array<bool, kNumLegs> in_stance{};
    FrequencyHz stride_phase_rate_hz{FrequencyHz{1.0}};
    bool stable{true};
    int support_contact_count{0};
    double stability_margin_m{0.0};
    TimePointUs timestamp_us{};
};

enum class FaultCode : uint8_t {
    NONE = 0,
    BUS_TIMEOUT,
    ESTOP,
    TIP_OVER,
    ESTIMATOR_INVALID,
    MOTOR_FAULT,
    JOINT_LIMIT,
    COMMAND_TIMEOUT
};

enum class FaultLifecycle : uint8_t {
    ACTIVE = 0,
    LATCHED,
    RECOVERING
};

struct SafetyState {
    bool inhibit_motion{true};
    bool torque_cut{false};
    bool stable{true};
    int support_contact_count{0};
    double stability_margin_m{0.0};
    FaultCode active_fault{FaultCode::NONE};
    FaultLifecycle fault_lifecycle{FaultLifecycle::ACTIVE};
    uint32_t active_fault_trip_count{0};
    TimePointUs active_fault_last_trip_us{};
    std::array<bool, kNumLegs> leg_enabled{true, true, true, true, true, true};
};

struct FootTarget {
    PositionM3 pos_body_m{};
    VelocityMps3 vel_body_mps{};
};

struct LegTargets {
    std::array<FootTarget, kNumLegs> feet{};
    TimePointUs timestamp_us{};
};

struct JointState {
  AngleRad pos_rad{};
  AngularRateRadPerSec vel_radps{};
};

struct LegState {
  std::array<JointState, kJointsPerLeg> joint_state{};
};

struct JointTargets {
  std::array<LegState,kNumLegs> leg_states{};
};

struct BodyPoseKinematics {
  // Body orientation in radians as {roll, pitch, yaw}.
  EulerAnglesRad3 orientation_rad{};
  // Body angular velocity in rad/s as {roll_rate, pitch_rate, yaw_rate}.
  AngularVelocityRadPerSec3 angular_velocity_radps{};

  // Body translation relative to nominal stance in meters as {x, y, z}.
  // x/y are planar body translation offsets; z is vertical body height.
  // (0, 0, 0) is the nominal stance translation.
  PositionM3 body_trans_m{};
  // Body translation velocity in m/s as {x_rate, y_rate, z_rate}.
  VelocityMps3 body_trans_mps{};
};

struct MotionIntent {
  RobotMode requested_mode{RobotMode::SAFE_IDLE};
  GaitType gait{GaitType::TRIPOD};
  // Planar walk command magnitude in m/s and heading in body frame radians.
  LinearRateMps speed_mps{};
  AngleRad heading_rad{};
  BodyPoseKinematics body_pose_setpoint{};
  uint64_t sample_id{0};
  TimePointUs timestamp_us{};
};

struct RobotState {
  std::array<LegState, kNumLegs> leg_states{};
  std::array<bool, kNumLegs> foot_contacts{};
  BodyPoseKinematics body_pose_state{};
  float voltage{0.0f};
  float current{0.0f};
  bool bus_ok{true};
  bool valid{false};
  uint64_t sample_id{0};
  TimePointUs timestamp_us{};

  // Optional-field validity flags for context-specific usage.
  bool has_body_pose_state{false};
  bool has_power_state{false};
  bool has_valid_flag{false};
};

struct ControlStatus {
    struct DynamicGaitTelemetry {
        bool valid{false};
        GaitType gait_family{GaitType::TRIPOD};
        uint8_t region{0};
        uint8_t turn_mode{0};
        uint8_t fallback_stage{0};
        double cadence_hz{0.0};
        double reach_utilization{0.0};
        double envelope_max_speed_normalized{0.0};
        double envelope_max_yaw_normalized{0.0};
        double envelope_max_roll_pitch_rad{0.0};
        bool envelope_allow_tripod{false};
        bool suppress_stride_progression{false};
        bool suppress_turning{false};
        bool prioritize_stability{true};
        std::array<double, kNumLegs> leg_phase{};
        std::array<double, kNumLegs> leg_duty_cycle{};
        std::array<bool, kNumLegs> leg_in_stance{};
        bool limiter_enabled{false};
        uint8_t limiter_phase{0};
        uint8_t active_constraint_reason{0};
        double adaptation_scale_linear{1.0};
        double adaptation_scale_yaw{1.0};
        double adaptation_scale_cadence{1.0};
        double adaptation_scale_step{1.0};
        bool hard_clamp_linear{false};
        bool hard_clamp_yaw{false};
        bool hard_clamp_reach{false};
        bool hard_clamp_cadence{false};
        bool saturated{false};
    };

    struct AutonomyTelemetry {
        bool enabled{false};
        bool step_ok{false};
        bool blocked{false};
        bool no_progress{false};
        bool recovery_active{false};
        bool motion_allowed{false};
        bool locomotion_sent{false};
        bool mission_loaded{false};
        bool mission_running{false};
        uint8_t mission_state{0};
        uint64_t mission_completed_waypoints{0};
        uint64_t mission_total_waypoints{0};
    };

    RobotMode active_mode{RobotMode::SAFE_IDLE};
    bool estimator_valid{false};
    bool bus_ok{false};
    FaultCode active_fault{FaultCode::NONE};
    uint64_t loop_counter{0};
    DynamicGaitTelemetry dynamic_gait{};
    AutonomyTelemetry autonomy{};
};

// ============================================================
// BodyPose
// ============================================================
struct BodyPose {
  PositionM3 position{0.0, 0.0, 0.0};
  
  AngleRad yaw{};
  AngleRad pitch{};
  AngleRad roll{};
  
  Mat3 rotationBodyToWorld() const;
};

// ============================================================
// Servo calibration
// ------------------------------------------------------------
// Servos usually need:
//
//   servo = sign * joint + offset
//
// This handles:
//   - left/right mirroring
//   - horn offsets
//   - installation differences/variations
// ============================================================
struct ServoCalibration {
    AngleRad coxaOffset{};
    AngleRad femurOffset{};
    AngleRad tibiaOffset{};

    double coxaSign{1.0};
    double femurSign{1.0};
    double tibiaSign{1.0};

    // Convert joint angles to servo angles
    LegState toServoAngles(const LegState& leg) const;
    
    // Convert servo angles to joint angles
    LegState toJointAngles(const LegState& leg) const;
};

struct ServoDirectionDynamics {
    double tau_s{0.08};
    double vmax_radps{8.0};
};

struct ServoJointDynamics {
    ServoDirectionDynamics positive_direction{};
    ServoDirectionDynamics negative_direction{};
};

// ============================================================
// Leg geometry
// ============================================================
struct LegGeometry {
    LegID legID;

    PositionM3 bodyCoxaOffset{};  // Coxa joint location in body frame
    AngleRad mountAngle{};

    LengthM coxaLength{};   // L1
    LengthM femurLength{};  // L2
    LengthM tibiaLength{};  // L3

    ServoCalibration servo;
    std::array<ServoJointDynamics, kJointsPerLeg> servoDynamics{};
};

// ============================================================
// Hexapod geometry
// ============================================================
struct HexapodGeometry {
  std::array<LegGeometry, kNumLegs> legGeometry{};
  LengthM toBottom{LengthM{0.04}}; // change to vec3 (centre of mass)
};

// Hexapod frame of reference
/*

             5    front  4       * legId - legName  - localYaxisAngle |
              \   head  /        *  0    - R3       - (135 + 8)       | 143
               *---*---*         *  1    - L3       - (225 - 8)       | 217
              /    |    \        *  2    - R2       -  90             | 90
             /     |     \       *  3    - L2       -  270            | 270
         3 -*-----cog-----*- 2   *  4    - R1       - (45  - 8)       | 37
             \     |     /       *  5    - L1       - (315 + 8)       | 323
              \    |    /
               *---*---*          ^ hexapodY
              /  back   \         |
             1           0        *---> hexapodX
                                 /
                                * hexapodZ
                                
                                localAxisAngle is angle between forward vector (hexapodY) 
                                and local leg axis
                                hexapodZ points up (positive from ground to hexapod)
                                
                                * legId |      leg mount point (m)   |   
                                *   0   | ( 0.063,  -0.0835, -0.007) |
                                *   1   | (-0.063,  -0.0835, -0.007) |
                                *   2   | ( 0.0815,       0, -0.007) |
                                *   3   | (-0.0815,       0, -0.007) |
                                *   4   | ( 0.063,   0.0835, -0.007) |
                                *   5   | (-0.063,   0.0835, -0.007) |
                                
                                centre of body is (0,0,0)
                                when on flat surface, centre of body is 40mm above ground

*/
