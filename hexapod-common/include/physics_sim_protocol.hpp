#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>

// Binary UDP protocol between hexapod-server (client) and hexapod-physics-sim --serve (host).
// All structs are packed; wire format is little-endian (native on supported targets).

namespace physics_sim {

// Assembly pitch (LegPitchDirection in Y-up sim) when placing femur/tibia in scenes.cpp.
inline constexpr float kAssemblyFemurPitchRad = -0.20f;
inline constexpr float kAssemblyTibiaPitchRad = -1.00f;
inline constexpr std::size_t kMaxObstacleFootprints = 16;

// The built-in hexapod used by `hexapod-physics-sim --serve` defines wire angle zero at the
// assembled relaxed pose created in scenes.cpp. In the hexapod-server mechanical joint
// convention that means:
//   coxa  = 0
//   femur = kAssemblyFemurPitchRad
//   tibia = kAssemblyTibiaPitchRad
inline constexpr float kWireZeroCoxaMechanicalRad = 0.0f;
inline constexpr float kWireZeroFemurMechanicalRad = kAssemblyFemurPitchRad;
inline constexpr float kWireZeroTibiaMechanicalRad = kAssemblyTibiaPitchRad;

/// Matrix ToF LiDAR models sharing the same wire layout (`StateResponse` matrix LiDAR tail).
enum class MatrixLidarModel : std::uint8_t {
    None = 0,
    /// 64×8 grid, 120°×20° FOV, 100–5000 mm range, 14 mm resolution (primary sim sensor).
    Matrix64x8 = 1,
    /// VL53L7CX-style 8×8 grid, 60°×60° FOV, 20–3500 mm (same payload; unused cells set to `kMatrixLidarInvalidMm`).
    Vl53l7Cx8x8 = 2,
};

/// Sentinel for “no return” / invalid cell in `matrix_lidar_ranges_mm`.
inline constexpr std::uint16_t kMatrixLidarInvalidMm = 0xFFFF;
inline constexpr std::size_t kMatrixLidarMaxCells = 512;

enum class MessageType : std::uint8_t {
    StepCommand = 1,
    StateResponse = 2,
    ConfigCommand = 3,
    ConfigAck = 4,
    StateCorrection = 5,
};

enum class ContactPhase : std::uint8_t {
    Swing = 0,
    ExpectedTouchdown = 1,
    ContactCandidate = 2,
    ConfirmedStance = 3,
    LostCandidate = 4,
    Search = 5,
};

inline constexpr std::uint8_t kStateCorrectionPoseValid = 1u << 0;
inline constexpr std::uint8_t kStateCorrectionTwistValid = 1u << 1;
inline constexpr std::uint8_t kStateCorrectionContactValid = 1u << 2;
inline constexpr std::uint8_t kStateCorrectionTerrainValid = 1u << 3;
inline constexpr std::uint8_t kStateCorrectionHardReset = 1u << 4;
inline constexpr std::uint8_t kStateCorrectionRelocalize = 1u << 5;

#pragma pack(push, 1)

struct StepCommand {
    std::uint8_t message_type{static_cast<std::uint8_t>(MessageType::StepCommand)};
    /// Normal steps use sequence_id != 0. sequence_id == 0 is reserved: server may request a
    /// state snapshot without advancing physics or applying joint_targets (see hexapod-physics-sim serve mode).
    std::uint32_t sequence_id{0};
    float dt_seconds{0.0f};
    std::array<float, 18> joint_targets{}; // radians, sim servo order (6 legs × 3 joints)
};

struct ObstacleFootprint {
    float center_x{0.0f};      // sim world X
    float center_z{0.0f};      // sim world Z (server world Y)
    float half_extent_x{0.0f}; // footprint half-width along local obstacle X
    float half_extent_z{0.0f}; // footprint half-width along local obstacle Z
    float yaw_rad{0.0f};       // yaw about sim +Y (same sign as server +Z yaw)
};

struct StateResponse {
    std::uint8_t message_type{static_cast<std::uint8_t>(MessageType::StateResponse)};
    std::uint32_t sequence_id{0};
    std::array<float, 3> body_position{}; // sim frame (Y-up)
    std::array<float, 4> body_orientation{}; // quaternion wxyz, sim frame
    std::array<float, 3> body_linear_velocity{};
    std::array<float, 3> body_angular_velocity{};
    std::array<float, 18> joint_angles{};
    std::array<float, 18> joint_velocities{};
    std::array<std::uint8_t, 6> foot_contacts{};
    std::array<std::array<float, 3>, 6> foot_contact_normals{}; // sim world
    std::uint32_t obstacle_count{0};
    std::array<ObstacleFootprint, kMaxObstacleFootprints> obstacles{};
    /// Populated by `hexapod-physics-sim --serve` when matrix LiDAR simulation is enabled.
    std::uint8_t matrix_lidar_valid{0};
    MatrixLidarModel matrix_lidar_model{MatrixLidarModel::None};
    std::uint8_t matrix_lidar_cols{0};
    std::uint8_t matrix_lidar_rows{0};
    std::array<std::uint16_t, kMatrixLidarMaxCells> matrix_lidar_ranges_mm{};
};

struct StateCorrection {
    std::uint8_t message_type{static_cast<std::uint8_t>(MessageType::StateCorrection)};
    std::uint32_t sequence_id{0};
    std::uint64_t timestamp_us{0};
    std::uint8_t flags{0};
    float correction_strength{1.0f};
    std::array<float, 3> body_position{};
    std::array<float, 4> body_orientation{};
    std::array<float, 3> body_linear_velocity{};
    std::array<float, 3> body_angular_velocity{};
    std::array<std::uint8_t, 6> foot_contact_phase{};
    std::array<float, 6> foot_contact_confidence{};
    std::array<float, 6> foot_ground_height_m{};
    std::array<float, 6> foot_ground_confidence{};
    std::array<float, 3> terrain_normal{0.0f, 1.0f, 0.0f};
    float terrain_height_m{0.0f};
    std::array<std::uint8_t, 7> reserved{};
};

struct ConfigCommand {
    std::uint8_t message_type{static_cast<std::uint8_t>(MessageType::ConfigCommand)};
    std::array<float, 3> gravity{};
    std::int32_t solver_iterations{8};
    std::int32_t reserved_i32{0};
    std::array<float, 4> reserved_float{};
};

struct ConfigAck {
    std::uint8_t message_type{static_cast<std::uint8_t>(MessageType::ConfigAck)};
    std::uint32_t body_count{0};
    std::uint32_t joint_count{0};
};

#pragma pack(pop)

static_assert(std::is_trivially_copyable_v<StepCommand>);
static_assert(std::is_trivially_copyable_v<ObstacleFootprint>);
static_assert(std::is_trivially_copyable_v<StateResponse>);
static_assert(std::is_trivially_copyable_v<StateCorrection>);
static_assert(std::is_trivially_copyable_v<ConfigCommand>);
static_assert(std::is_trivially_copyable_v<ConfigAck>);

inline constexpr std::size_t kStepCommandBytes = sizeof(StepCommand);
inline constexpr std::size_t kStateResponseBytes = sizeof(StateResponse);
inline constexpr std::size_t kStateCorrectionBytes = sizeof(StateCorrection);
inline constexpr std::size_t kConfigCommandBytes = sizeof(ConfigCommand);
inline constexpr std::size_t kConfigAckBytes = sizeof(ConfigAck);

inline bool tryDecodeStepCommand(const void* data, std::size_t len, StepCommand& out) {
    if (len < kStepCommandBytes) {
        return false;
    }
    std::memcpy(&out, data, kStepCommandBytes);
    return out.message_type == static_cast<std::uint8_t>(MessageType::StepCommand);
}

inline bool tryDecodeStateResponse(const void* data, std::size_t len, StateResponse& out) {
    if (len < kStateResponseBytes) {
        return false;
    }
    std::memcpy(&out, data, kStateResponseBytes);
    return out.message_type == static_cast<std::uint8_t>(MessageType::StateResponse);
}

inline bool tryDecodeStateCorrection(const void* data, std::size_t len, StateCorrection& out) {
    if (len < kStateCorrectionBytes) {
        return false;
    }
    std::memcpy(&out, data, kStateCorrectionBytes);
    return out.message_type == static_cast<std::uint8_t>(MessageType::StateCorrection);
}

inline bool tryDecodeConfigCommand(const void* data, std::size_t len, ConfigCommand& out) {
    if (len < kConfigCommandBytes) {
        return false;
    }
    std::memcpy(&out, data, kConfigCommandBytes);
    return out.message_type == static_cast<std::uint8_t>(MessageType::ConfigCommand);
}

inline bool tryDecodeConfigAck(const void* data, std::size_t len, ConfigAck& out) {
    if (len < kConfigAckBytes) {
        return false;
    }
    std::memcpy(&out, data, kConfigAckBytes);
    return out.message_type == static_cast<std::uint8_t>(MessageType::ConfigAck);
}

} // namespace physics_sim
