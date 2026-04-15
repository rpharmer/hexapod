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

// The built-in hexapod used by `hexapod-physics-sim --serve` defines wire angle zero at the
// assembled relaxed pose created in scenes.cpp. In the hexapod-server mechanical joint
// convention that means:
//   coxa  = 0
//   femur = kAssemblyFemurPitchRad
//   tibia = kAssemblyTibiaPitchRad
inline constexpr float kWireZeroCoxaMechanicalRad = 0.0f;
inline constexpr float kWireZeroFemurMechanicalRad = kAssemblyFemurPitchRad;
inline constexpr float kWireZeroTibiaMechanicalRad = kAssemblyTibiaPitchRad;

enum class MessageType : std::uint8_t {
    StepCommand = 1,
    StateResponse = 2,
    ConfigCommand = 3,
    ConfigAck = 4,
};

#pragma pack(push, 1)

struct StepCommand {
    std::uint8_t message_type{static_cast<std::uint8_t>(MessageType::StepCommand)};
    /// Normal steps use sequence_id != 0. sequence_id == 0 is reserved: server may request a
    /// state snapshot without advancing physics or applying joint_targets (see hexapod-physics-sim serve mode).
    std::uint32_t sequence_id{0};
    float dt_seconds{0.0f};
    std::array<float, 18> joint_targets{}; // radians, sim servo order (6 legs × 3 joints)
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
static_assert(std::is_trivially_copyable_v<StateResponse>);
static_assert(std::is_trivially_copyable_v<ConfigCommand>);
static_assert(std::is_trivially_copyable_v<ConfigAck>);

inline constexpr std::size_t kStepCommandBytes = sizeof(StepCommand);
inline constexpr std::size_t kStateResponseBytes = sizeof(StateResponse);
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
