#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include "hexapod-common.hpp"

// ============================================================
// Constants / utility
// ============================================================
constexpr double kPi = 3.14159265358979323846;

double deg2rad(double deg);

double rad2deg(double rad);

double clamp(double value, double lo, double hi);

// ============================================================

constexpr int COXA = 0;
constexpr int FEMUR = 1;
constexpr int TIBIA = 2;



constexpr int kNumLegs = 6;
constexpr int kJointsPerLeg = 3;
constexpr int kNumJoints = kNumLegs * kJointsPerLeg;

using Clock = std::chrono::steady_clock;

inline uint64_t now_us() {
    const auto t = Clock::now().time_since_epoch();
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(t).count());
}

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

struct Vec3 {
    double x{0.0};
    double y{0.0};
    double z{0.0};
    
    Vec3 operator+(const Vec3& other) const;
    Vec3 operator-(const Vec3& other) const;
    Vec3 operator*(double s) const;
};

// ============================================================
// Basic 3x3 matrix for rotations
// ============================================================
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



struct GaitState {
    std::array<double, kNumLegs> phase{};
    std::array<bool, kNumLegs> in_stance{};
    double stride_phase_rate_hz{1.0};
    uint64_t timestamp_us{0};
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

struct SafetyState {
    bool inhibit_motion{true};
    bool torque_cut{false};
    FaultCode active_fault{FaultCode::NONE};
    std::array<bool, kNumLegs> leg_enabled{true, true, true, true, true, true};
};

struct FootTarget {
    Vec3 pos_body_m{};
    Vec3 vel_body_mps{};
};

struct LegTargets {
    std::array<FootTarget, kNumLegs> feet{};
    uint64_t timestamp_us{0};
};

struct JointRawState {
  double pos_rad {0.0};
};

struct JointState {
  double pos_rad {0.0};
  double vel_radps {0.0};
};

struct LegRawState {
  std::array<JointRawState, kJointsPerLeg> joint_raw_state{};
};

struct LegState {
  std::array<JointState, kJointsPerLeg> joint_state{};
};

struct JointTargets {
  std::array<LegRawState,kNumLegs> leg_raw_states{};
};

struct BodyTwistState {
  Vec3 twist_pos_rad{};
  Vec3 twist_vel_radps{};
  
  Vec3 body_trans_m{};
  Vec3 body_trans_mps{};
};

struct RawHardwareState {
  std::array<LegRawState, kNumLegs> leg_states{};
  std::array<bool, kNumLegs> foot_contacts{};
  float voltage{0.0};
  float current{0.0};
  uint64_t timestamp_us{0};
  bool bus_ok{true};
};

struct MotionIntent {
  RobotMode requested_mode{RobotMode::SAFE_IDLE};
  GaitType gait{GaitType::TRIPOD};
  BodyTwistState twist{};
  uint64_t timestamp_us{0};
};

struct EstimatedState {
  std::array<LegRawState, kNumLegs> leg_states{};
  std::array<bool, kNumLegs> foot_contacts{};
  
  BodyTwistState body_twist_state{};
  uint64_t timestamp_us{0};
};

struct TargetBodyState {
  std::array<LegState, kNumLegs> leg_states{};
  std::array<bool, kNumLegs> foot_contacts{};
  BodyTwistState body_twist_state{};
  bool valid{false};
  uint64_t timestamp_us{0};
};

struct ControlStatus {
    RobotMode active_mode{RobotMode::SAFE_IDLE};
    bool estimator_valid{false};
    bool bus_ok{false};
    FaultCode active_fault{FaultCode::NONE};
    uint64_t loop_counter{0};
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
    double coxaOffset{0.0};
    double femurOffset{0.0};
    double tibiaOffset{0.0};

    double coxaSign{1.0};
    double femurSign{1.0};
    double tibiaSign{1.0};

    // Convert joint angles to servo angles
    LegRawState toServoAngles(const LegRawState& leg) const;
    LegState toServoAngles(const LegState& leg) const;
    
    // Convert servo angles to joint angles
    LegRawState toJointAngles(const LegRawState& leg) const;
    LegState toJointAngles(const LegState& leg) const;
};

// ============================================================
// Leg geometry
// ============================================================
struct LegGeometry {
    LegID legID;

    Vec3 bodyCoxaOffset;  // Coxa joint location in body frame
    double mountAngle{0.0};

    double coxaLength{0.0};   // L1
    double femurLength{0.0};  // L2
    double tibiaLength{0.0};  // L3

    ServoCalibration servo;
};

// ============================================================
// Hexapod geometry
// ============================================================
struct HexapodGeometry {
  std::array<LegGeometry, kNumLegs> legGeometry{};
  double toBottom {0.04}; // change to vec3 (centre of mass)
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

