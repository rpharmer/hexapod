#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>

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
    WAVE,
    CRAWL,
    TURN_IN_PLACE
};

struct GaitState {
    std::array<double, kNumLegs> phase{};
    std::array<bool, kNumLegs> in_stance{};
    FrequencyHz stride_phase_rate_hz{FrequencyHz{1.0}};
    /** Fraction of each stride cycle spent in stance (0..1). */
    double duty_factor{0.5};
    /** Per-leg phase offsets (cycles) blended with the global stride integrator. */
    std::array<double, kNumLegs> phase_offset{};
    /** Velocity-scaled horizontal stride amplitude (m). */
    double step_length_m{0.06};
    /** Velocity-scaled swing apex (m). */
    double swing_height_m{0.03};
    /** 0 = linear swing phase, 1 = full smoothstep time warp (softer accel / decel). */
    double swing_time_ease_01{1.0};
    /** stance_duration_s = duty_factor / stride_frequency_hz (when walking). */
    double stance_duration_s{0.5};
    double swing_duration_s{0.5};
    /** When true, keep leg in stance kinematics even if phase is in swing (support polygon gate). */
    std::array<bool, kNumLegs> stability_hold_stance{};
    /** Remaining stance-polygon margin before each leg can safely lift (m). */
    std::array<double, kNumLegs> support_liftoff_clearance_m{};
    /** True when the remaining stance polygon can safely release the leg at the current phase. */
    std::array<bool, kNumLegs> support_liftoff_safe_to_lift{};
    /** Minimum distance (m) from projected COM to stance support boundary; negative if outside. */
    double static_stability_margin_m{0.0};
    /** Body XY finite-diff of commanded planar velocity (m/s²), for swing placement / cadence. */
    double cmd_accel_body_x_mps2{0.0};
    double cmd_accel_body_y_mps2{0.0};
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
    COMMAND_TIMEOUT,
    BODY_COLLAPSE,
};

enum class FaultLifecycle : uint8_t {
    ACTIVE = 0,
    LATCHED,
    RECOVERING
};

struct SafetyState {
    bool inhibit_motion{true};
    bool torque_cut{false};
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

struct BodyTwistState {
  // Body orientation setpoint in radians as {roll, pitch, yaw}.
  EulerAnglesRad3 twist_pos_rad{};
  // Body angular velocity setpoint in rad/s as {roll_rate, pitch_rate, yaw_rate}.
  AngularVelocityRadPerSec3 twist_vel_radps{};

  // Body translation setpoint relative to nominal stance in meters as {x, y, z}.
  // x/y are planar body translation offsets; z is vertical body-height setpoint.
  // (0, 0, 0) is the nominal stance translation.
  PositionM3 body_trans_m{};
  // Body translation velocity setpoint in m/s as {x_rate, y_rate, z_rate}.
  VelocityMps3 body_trans_mps{};
};

/** Inertial sample in body axes (server world: +Z up). Gyro is angular rate; accel is specific force
 *  (what a strapdown MEMS reports, ~+Z at rest when upright). */
struct ImuSample {
    TimePointUs timestamp_us{};
    AngularVelocityRadPerSec3 gyro_radps{};
    Vec3 accel_mps2{};
    bool valid{false};
};

/** Matrix ToF LiDAR ranges in millimetres (invalid cells use `0xFFFF`, same as physics wire format). */
struct MatrixLidarFrame {
    static constexpr std::size_t kMaxCells = 512;
    TimePointUs timestamp_us{};
    std::uint8_t model{0};
    std::uint8_t cols{0};
    std::uint8_t rows{0};
    std::array<std::uint16_t, kMaxCells> ranges_mm{};
    bool valid{false};
};

enum class ContactPhase : std::uint8_t {
    Swing = 0,
    ExpectedTouchdown,
    ContactCandidate,
    ConfirmedStance,
    LostCandidate,
    Search,
};

struct FootContactFusion {
    ContactPhase phase{ContactPhase::Search};
    float confidence{0.0f};
    TimePointUs touchdown_window_start_us{};
    TimePointUs touchdown_window_end_us{};
    TimePointUs last_transition_us{};
};

struct FusionResidualSummary {
    Vec3 body_position_error_m{};
    Vec3 body_velocity_error_mps{};
    EulerAnglesRad3 body_orientation_error_rad{};
    std::array<double, kNumLegs> foot_contact_error{};
    double max_body_position_error_m{0.0};
    double max_body_orientation_error_rad{0.0};
    double contact_mismatch_ratio{0.0};
    double terrain_residual_m{0.0};
};

struct FusionDiagnostics {
    double model_trust{0.0};
    bool resync_requested{false};
    bool hard_reset_requested{false};
    bool predictive_mode{false};
    FusionResidualSummary residuals{};
};

struct MotionIntent {
  RobotMode requested_mode{RobotMode::SAFE_IDLE};
  GaitType gait{GaitType::TRIPOD};
  // Body-frame planar velocity command (m/s): +x forward, +y left.
  LinearRateMps cmd_vx_mps{};
  LinearRateMps cmd_vy_mps{};
  // Body-frame yaw rate command (rad/s), positive counter-clockwise when viewed from above.
  AngularRateRadPerSec cmd_yaw_radps{};
  // Legacy polar walk command; `motion_intent_utils` keeps these aligned when building intents.
  LinearRateMps speed_mps{};
  AngleRad heading_rad{};
  BodyTwistState twist{};
  uint64_t sample_id{0};
  TimePointUs timestamp_us{};
};

struct RobotState {
  std::array<LegState, kNumLegs> leg_states{};
  /** Fused controller-facing contact estimate; raw bridge snapshots are reconciled before use. */
  std::array<bool, kNumLegs> foot_contacts{};
  std::array<FootContactFusion, kNumLegs> foot_contact_fusion{};
  BodyTwistState body_twist_state{};
  FusionDiagnostics fusion{};
  float voltage{0.0f};
  float current{0.0f};
  bool bus_ok{true};
  bool valid{false};
  uint64_t sample_id{0};
  TimePointUs timestamp_us{};

  // Optional-field validity flags for context-specific usage.
  bool has_body_twist_state{false};
  bool has_power_state{false};
  bool has_valid_flag{false};
  bool has_fusion_diagnostics{false};
  /** Populated by physics sim bridge, hardware when available, or test doubles. */
  ImuSample imu{};
  bool has_imu{false};
  MatrixLidarFrame matrix_lidar{};
  bool has_matrix_lidar{false};
};

struct ControlStatus {
    RobotMode active_mode{RobotMode::SAFE_IDLE};
    bool estimator_valid{false};
    bool bus_ok{false};
    FaultCode active_fault{FaultCode::NONE};
    uint64_t loop_counter{0};
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
