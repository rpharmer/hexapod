#include "demo/serve_mode.hpp"

#include "demo/frame_sink.hpp"
#include "demo/matrix_lidar_sim.hpp"
#include "demo/process_resource_diagnostics.hpp"
#include "demo/scene_json.hpp"
#include "demo/scenes.hpp"
#include "demo/terrain_patch.hpp"
#include "minphys3d/collision/shapes.hpp"
#include "minphys3d/core/world.hpp"
#include "minphys3d/demo/hexapod_scene.hpp"
#include "minphys3d/math/vec3.hpp"
#include "minphys3d/solver/types.hpp"
#include "physics_sim_protocol.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <iostream>
#include <optional>
#include <memory>
#include <limits>
#include <string>
#include <vector>
#include <thread>
#include <sys/time.h>

#if defined(__linux__) || defined(__APPLE__)
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#else
#error "serve_mode requires POSIX sockets (Linux/macOS)"
#endif

namespace minphys3d::demo {
namespace {

constexpr std::size_t kRecvBufBytes = 512;
constexpr int kSocketRetryCount = 5;
constexpr int kSocketRetryDelayMs = 25;
constexpr std::uint64_t kMatrixLidarFramePeriodUs = 15'625; // 64 Hz full-frame cadence
constexpr float kServeMaxPhysicsSubstepSeconds = 1.0f / 240.0f;
constexpr int kServeMaxPhysicsSubsteps = 16;

struct CachedMatrixLidarFrame {
    bool valid{false};
    std::uint64_t timestamp_us{0};
    std::uint8_t model{static_cast<std::uint8_t>(physics_sim::MatrixLidarModel::None)};
    std::uint8_t cols{0};
    std::uint8_t rows{0};
    std::array<std::uint16_t, physics_sim::kMatrixLidarMaxCells> ranges_mm{};
};

void SyncTerrainHeightfield(World& world, const TerrainPatch& terrain_patch) {
    if (!terrain_patch.initialized()) {
        world.ClearTerrainHeightfield();
        return;
    }
    world.SetTerrainHeightfield(terrain_patch.BuildTerrainHeightfieldAttachment());
}

void writeCachedMatrixLidarFrame(const CachedMatrixLidarFrame& cache, physics_sim::StateResponse& rsp) {
    if (!cache.valid) {
        rsp.matrix_lidar_valid = 0;
        rsp.matrix_lidar_model = physics_sim::MatrixLidarModel::None;
        rsp.matrix_lidar_cols = 0;
        rsp.matrix_lidar_rows = 0;
        rsp.matrix_lidar_timestamp_us = 0;
        rsp.matrix_lidar_ranges_mm.fill(physics_sim::kMatrixLidarInvalidMm);
        return;
    }

    rsp.matrix_lidar_valid = 1;
    rsp.matrix_lidar_model = static_cast<physics_sim::MatrixLidarModel>(cache.model);
    rsp.matrix_lidar_cols = cache.cols;
    rsp.matrix_lidar_rows = cache.rows;
    rsp.matrix_lidar_timestamp_us = cache.timestamp_us;
    rsp.matrix_lidar_ranges_mm = cache.ranges_mm;
}

int OpenUdpSocketWithRetry() {
    for (int attempt = 0; attempt < kSocketRetryCount; ++attempt) {
        const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (fd >= 0) {
            return fd;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(kSocketRetryDelayMs));
    }
    return -1;
}

std::array<std::uint32_t, 18> ServoJointIdsInWireOrder(const HexapodSceneObjects& scene) {
    return {
        scene.legs[0].bodyToCoxaJoint,   scene.legs[0].coxaToFemurJoint,   scene.legs[0].femurToTibiaJoint,
        scene.legs[1].bodyToCoxaJoint,   scene.legs[1].coxaToFemurJoint,   scene.legs[1].femurToTibiaJoint,
        scene.legs[2].bodyToCoxaJoint,   scene.legs[2].coxaToFemurJoint,   scene.legs[2].femurToTibiaJoint,
        scene.legs[3].bodyToCoxaJoint,   scene.legs[3].coxaToFemurJoint,   scene.legs[3].femurToTibiaJoint,
        scene.legs[4].bodyToCoxaJoint,   scene.legs[4].coxaToFemurJoint,   scene.legs[4].femurToTibiaJoint,
        scene.legs[5].bodyToCoxaJoint,   scene.legs[5].coxaToFemurJoint,   scene.legs[5].femurToTibiaJoint,
    };
}

Vec3 AverageUnitNormal(const Vec3& acc, const Vec3& n) {
    const Vec3 sum = acc + n;
    const float len = Length(sum);
    if (len <= 1.0e-6f) {
        return n;
    }
    return sum * (1.0f / len);
}

float WrapAngleRad(const float angle_rad) {
    return std::atan2(std::sin(angle_rad), std::cos(angle_rad));
}

float YawAboutSimUp(const Quat& q) {
    const float siny_cosp = 2.0f * (q.w * q.y + q.x * q.z);
    const float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

template <std::size_t N>
bool IsFiniteArray(const std::array<float, N>& values) {
    for (const float v : values) {
        if (!std::isfinite(v)) {
            return false;
        }
    }
    return true;
}

float Clamp01(float value) {
    return std::clamp(value, 0.0f, 1.0f);
}

float QuatDot(const Quat& a, const Quat& b) {
    return a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
}

float QuatAngleBetween(const Quat& a, const Quat& b) {
    const Quat qa = Normalize(a);
    Quat qb = Normalize(b);
    if (QuatDot(qa, qb) < 0.0f) {
        qb = qb * -1.0f;
    }
    const float dot = std::clamp(std::abs(QuatDot(qa, qb)), 0.0f, 1.0f);
    return 2.0f * std::acos(dot);
}

Quat NlerpShortest(const Quat& a, const Quat& b, float t) {
    const Quat qa = Normalize(a);
    Quat qb = Normalize(b);
    if (QuatDot(qa, qb) < 0.0f) {
        qb = qb * -1.0f;
    }
    return Normalize(qa * (1.0f - t) + qb * t);
}

Vec3 LerpVec3(const Vec3& a, const Vec3& b, float t) {
    return a * (1.0f - t) + b * t;
}

float ContactPhaseWeight(std::uint8_t phase_value) {
    switch (static_cast<physics_sim::ContactPhase>(phase_value)) {
        case physics_sim::ContactPhase::Swing:
            return 0.0f;
        case physics_sim::ContactPhase::ExpectedTouchdown:
            return 0.35f;
        case physics_sim::ContactPhase::ContactCandidate:
            return 0.70f;
        case physics_sim::ContactPhase::ConfirmedStance:
            return 1.0f;
        case physics_sim::ContactPhase::LostCandidate:
            return 0.20f;
        case physics_sim::ContactPhase::Search:
            return 0.10f;
    }
    return 0.0f;
}

float ContactSupportWeight(const physics_sim::StateCorrection& correction) {
    float total = 0.0f;
    int count = 0;
    for (std::size_t i = 0; i < correction.foot_contact_phase.size(); ++i) {
        const float confidence = Clamp01(correction.foot_contact_confidence[i]);
        const float phase_weight = ContactPhaseWeight(correction.foot_contact_phase[i]);
        const float weight = confidence * phase_weight;
        if (weight > 0.0f) {
            total += weight;
            ++count;
        }
    }
    if (count == 0) {
        return 0.0f;
    }
    return Clamp01(total / static_cast<float>(count));
}

std::optional<Vec3> FootCenterWorld(const World& world, const HexapodSceneObjects& scene, const std::size_t leg_index) {
    if (leg_index >= scene.legs.size()) {
        return std::nullopt;
    }
    const Body& tibia = world.GetBody(scene.legs[leg_index].tibia);
    for (const CompoundChild& child : tibia.compoundChildren) {
        if (child.shape == ShapeType::Sphere) {
            return tibia.position + Rotate(tibia.orientation, child.localPosition);
        }
    }
    return std::nullopt;
}

std::vector<TerrainSample> BuildTerrainSamples(const World& world,
                                               const HexapodSceneObjects& scene,
                                               const physics_sim::StateCorrection& correction) {
    std::vector<TerrainSample> samples;
    for (std::size_t leg = 0; leg < scene.legs.size(); ++leg) {
        const float confidence = Clamp01(correction.foot_ground_confidence[leg]);
        const float phase_weight = ContactPhaseWeight(correction.foot_contact_phase[leg]);
        const float sample_confidence = confidence * phase_weight;
        const float height = correction.foot_ground_height_m[leg];
        if (sample_confidence <= 0.0f || !std::isfinite(height)) {
            continue;
        }
        const std::optional<Vec3> foot_world = FootCenterWorld(world, scene, leg);
        if (!foot_world.has_value()) {
            continue;
        }
        samples.push_back(TerrainSample{*foot_world, height, sample_confidence});
    }
    return samples;
}

bool NormalizeTerrainNormal(const std::array<float, 3>& in, Vec3& out) {
    const Vec3 candidate{in[0], in[1], in[2]};
    return TryNormalize(candidate, out);
}

void ApplyRigidTransformToRobot(World& world,
                                const HexapodSceneObjects& scene,
                                const Vec3& source_origin,
                                const Quat& rotation_delta,
                                const Vec3& target_origin,
                                bool zero_velocities) {
    for (const std::uint32_t body_id : scene.body_ids) {
        if (body_id == scene.plane) {
            continue;
        }
        Body& body = world.GetBody(body_id);
        body.position = target_origin + Rotate(rotation_delta, body.position - source_origin);
        body.orientation = Normalize(rotation_delta * body.orientation);
        if (zero_velocities) {
            body.velocity = {};
            body.angularVelocity = {};
        } else {
            body.velocity = Rotate(rotation_delta, body.velocity);
            body.angularVelocity = Rotate(rotation_delta, body.angularVelocity);
        }
        body.force = {};
        body.torque = {};
        body.isSleeping = false;
        body.sleepCounter = 0;
    }
}

struct AssimilationReport {
    enum class Mode : std::uint8_t {
        Soft = 0,
        Strong = 1,
        HardReset = 2,
    };

    Mode mode{Mode::Soft};
    float position_error{0.0f};
    float orientation_error_rad{0.0f};
    float linear_velocity_error{0.0f};
    float angular_velocity_error{0.0f};
    float terrain_error{0.0f};
    float correction_strength{1.0f};
    float contact_support_weight{0.0f};
    std::uint64_t timestamp_us{0};
};

struct AssimilationState {
    std::uint64_t last_timestamp_us{0};
    AssimilationReport last_report{};
    std::array<std::uint8_t, 6> last_contact_phase{};
    std::array<float, 6> last_contact_confidence{};
    std::array<float, 6> last_foot_ground_height{};
    std::array<float, 6> last_foot_ground_confidence{};
    float last_contact_support_weight{0.0f};
    bool has_contact_hints{false};
    Vec3 last_terrain_normal{0.0f, 1.0f, 0.0f};
    float last_terrain_height{0.0f};
    bool has_terrain_hints{false};
};

void ApplyContactArbitrationToLidarSamples(const World& world,
                                           const HexapodSceneObjects& scene,
                                           const AssimilationState& assimilation_state,
                                           const TerrainPatchConfig& terrain_config,
                                           std::vector<TerrainSample>& lidar_samples) {
    if (!assimilation_state.has_contact_hints || lidar_samples.empty()) {
        return;
    }

    const float radius = std::max(0.0f, terrain_config.lidar_contact_arbitration_radius_m);
    const float disagreement = std::max(0.0f, terrain_config.lidar_contact_disagreement_m);
    if (radius <= 1.0e-5f) {
        return;
    }

    std::vector<TerrainSample> filtered;
    filtered.reserve(lidar_samples.size());
    for (const TerrainSample& sample : lidar_samples) {
        bool gated_by_contact = false;
        for (std::size_t leg = 0; leg < scene.legs.size(); ++leg) {
            const float phase_weight = ContactPhaseWeight(assimilation_state.last_contact_phase[leg]);
            const float contact_conf = Clamp01(assimilation_state.last_contact_confidence[leg]);
            const float ground_conf = Clamp01(assimilation_state.last_foot_ground_confidence[leg]);
            const float contact_weight = phase_weight * contact_conf * ground_conf;
            if (contact_weight < 0.50f) {
                continue;
            }
            const float contact_height = assimilation_state.last_foot_ground_height[leg];
            if (!std::isfinite(contact_height)) {
                continue;
            }
            const std::optional<Vec3> foot_world = FootCenterWorld(world, scene, leg);
            if (!foot_world.has_value()) {
                continue;
            }
            const float dx = sample.world_position.x - foot_world->x;
            const float dz = sample.world_position.z - foot_world->z;
            if (dx * dx + dz * dz > radius * radius) {
                continue;
            }
            if (std::abs(sample.height_m - contact_height) > disagreement) {
                gated_by_contact = true;
                break;
            }
        }
        if (!gated_by_contact) {
            filtered.push_back(sample);
        }
    }
    lidar_samples.swap(filtered);
}

template <std::size_t MaxSections>
AssimilationReport ApplyStateCorrection(World& world,
                                        const HexapodSceneObjects& scene,
                                        const physics_sim::StateCorrection& correction,
                                        AssimilationState& assimilation_state,
                                        TerrainPatch& terrain_patch,
                                        resource_monitoring::SectionProfiler<MaxSections>* profiler = nullptr) {
    AssimilationReport report{};
    report.timestamp_us = correction.timestamp_us;
    report.correction_strength = Clamp01(correction.correction_strength);
    report.contact_support_weight = assimilation_state.has_contact_hints ? assimilation_state.last_contact_support_weight : 0.0f;

    if (correction.timestamp_us != 0 && correction.timestamp_us < assimilation_state.last_timestamp_us) {
        return report;
    }
    const std::uint64_t previous_timestamp_us = assimilation_state.last_timestamp_us;
    if (correction.timestamp_us != 0) {
        assimilation_state.last_timestamp_us = correction.timestamp_us;
    }

    Body& chassis = world.GetBody(scene.body);
    Body& plane = world.GetBody(scene.plane);

    const bool pose_valid =
        (correction.flags & physics_sim::kStateCorrectionPoseValid) != 0 &&
        IsFiniteArray(correction.body_position) &&
        IsFiniteArray(correction.body_orientation);
    const bool twist_valid =
        (correction.flags & physics_sim::kStateCorrectionTwistValid) != 0 &&
        IsFiniteArray(correction.body_linear_velocity) &&
        IsFiniteArray(correction.body_angular_velocity);
    const bool terrain_valid =
        (correction.flags & physics_sim::kStateCorrectionTerrainValid) != 0 &&
        std::isfinite(correction.terrain_height_m) &&
        IsFiniteArray(correction.terrain_normal);
    const bool hard_reset_requested =
        (correction.flags & physics_sim::kStateCorrectionHardReset) != 0 ||
        (correction.flags & physics_sim::kStateCorrectionRelocalize) != 0;
    float terrain_age_seconds = 0.0f;
    if (previous_timestamp_us != 0 &&
        correction.timestamp_us != 0 &&
        correction.timestamp_us > previous_timestamp_us) {
        terrain_age_seconds = static_cast<float>(correction.timestamp_us - previous_timestamp_us) / 1000000.0f;
    }

    Vec3 target_position = chassis.position;
    Quat target_orientation = chassis.orientation;
    Vec3 target_linear_velocity = chassis.velocity;
    Vec3 target_angular_velocity = chassis.angularVelocity;
    Vec3 target_terrain_normal = plane.planeNormal;
    float target_terrain_height = plane.planeOffset;
    std::vector<TerrainSample> terrain_samples;

    if (pose_valid) {
        target_position = {correction.body_position[0], correction.body_position[1], correction.body_position[2]};
        target_orientation = Normalize(Quat{
            correction.body_orientation[0],
            correction.body_orientation[1],
            correction.body_orientation[2],
            correction.body_orientation[3],
        });
        report.position_error = Length(target_position - chassis.position);
        report.orientation_error_rad = QuatAngleBetween(chassis.orientation, target_orientation);
    }

    if (twist_valid) {
        target_linear_velocity = {
            correction.body_linear_velocity[0],
            correction.body_linear_velocity[1],
            correction.body_linear_velocity[2],
        };
        target_angular_velocity = {
            correction.body_angular_velocity[0],
            correction.body_angular_velocity[1],
            correction.body_angular_velocity[2],
        };
        report.linear_velocity_error = Length(target_linear_velocity - chassis.velocity);
        report.angular_velocity_error = Length(target_angular_velocity - chassis.angularVelocity);
    }

    if ((correction.flags & physics_sim::kStateCorrectionContactValid) != 0) {
        assimilation_state.has_contact_hints = true;
        assimilation_state.last_contact_support_weight = ContactSupportWeight(correction);
        assimilation_state.last_contact_phase = correction.foot_contact_phase;
        assimilation_state.last_contact_confidence = correction.foot_contact_confidence;
        assimilation_state.last_foot_ground_height = correction.foot_ground_height_m;
        assimilation_state.last_foot_ground_confidence = correction.foot_ground_confidence;
        report.contact_support_weight = assimilation_state.last_contact_support_weight;
    }

    if (terrain_valid) {
        Vec3 normal{};
        if (NormalizeTerrainNormal(correction.terrain_normal, normal)) {
            target_terrain_normal = normal;
        }

        float weighted_height = correction.terrain_height_m;
        float weighted_sum = 0.0f;
        float weight_total = 0.0f;
        for (std::size_t i = 0; i < correction.foot_ground_height_m.size(); ++i) {
            const float height = correction.foot_ground_height_m[i];
            const float ground_conf = Clamp01(correction.foot_ground_confidence[i]);
            const float contact_conf = Clamp01(correction.foot_contact_confidence[i]);
            const float phase_weight = ContactPhaseWeight(correction.foot_contact_phase[i]);
            const float weight = ground_conf * contact_conf * phase_weight;
            if (std::isfinite(height) && weight > 0.0f) {
                weighted_sum += weight * height;
                weight_total += weight;
            }
        }
        if (weight_total > 0.0f) {
            weighted_height = weighted_sum / weight_total;
        }
        target_terrain_height = weighted_height;
        report.terrain_error = std::abs(target_terrain_height - plane.planeOffset);
        assimilation_state.has_terrain_hints = true;
        assimilation_state.last_terrain_normal = target_terrain_normal;
        assimilation_state.last_terrain_height = target_terrain_height;
    } else if (assimilation_state.has_terrain_hints && report.mode != AssimilationReport::Mode::HardReset) {
        target_terrain_normal = assimilation_state.last_terrain_normal;
        target_terrain_height = assimilation_state.last_terrain_height;
    } else if (report.mode == AssimilationReport::Mode::HardReset) {
        assimilation_state.has_terrain_hints = false;
        assimilation_state.last_terrain_normal = plane.planeNormal;
        assimilation_state.last_terrain_height = plane.planeOffset;
    }

    if (hard_reset_requested && !twist_valid) {
        target_linear_velocity = {};
        target_angular_velocity = {};
    }

    const bool large_divergence =
        report.position_error > 0.35f ||
        report.orientation_error_rad > 0.75f ||
        report.linear_velocity_error > 1.50f ||
        report.angular_velocity_error > 4.0f ||
        report.terrain_error > 0.20f;

    if (hard_reset_requested || large_divergence) {
        report.mode = AssimilationReport::Mode::HardReset;
    } else if (report.position_error > 0.12f ||
               report.orientation_error_rad > 0.25f ||
               report.terrain_error > 0.05f ||
               report.linear_velocity_error > 0.50f ||
               report.angular_velocity_error > 1.0f) {
        report.mode = AssimilationReport::Mode::Strong;
    } else {
        report.mode = AssimilationReport::Mode::Soft;
    }

    float pose_blend = 0.0f;
    float twist_blend = 0.0f;
    float terrain_blend = 0.0f;
    switch (report.mode) {
        case AssimilationReport::Mode::Soft:
            pose_blend = 0.15f * report.correction_strength;
            twist_blend = 0.20f * report.correction_strength;
            terrain_blend = 0.10f * report.correction_strength;
            break;
        case AssimilationReport::Mode::Strong:
            pose_blend = 0.55f * report.correction_strength;
            twist_blend = 0.65f * report.correction_strength;
            terrain_blend = 0.40f * report.correction_strength;
            break;
        case AssimilationReport::Mode::HardReset:
            pose_blend = 1.0f;
            twist_blend = 1.0f;
            terrain_blend = 1.0f;
            break;
    }

    const float support_bias = 0.5f + 0.5f * report.contact_support_weight;
    pose_blend = Clamp01(pose_blend * support_bias);
    twist_blend = Clamp01(twist_blend * (0.75f + 0.25f * report.contact_support_weight));
    terrain_blend = Clamp01(terrain_blend * (0.35f + 0.65f * report.contact_support_weight));

    if (terrain_valid) {
        if (report.mode == AssimilationReport::Mode::HardReset) {
            plane.planeNormal = Normalize(target_terrain_normal);
            plane.planeOffset = target_terrain_height;
        } else {
            plane.planeNormal = Normalize(LerpVec3(plane.planeNormal, target_terrain_normal, terrain_blend));
            plane.planeOffset = plane.planeOffset * (1.0f - terrain_blend) + target_terrain_height * terrain_blend;
        }
    }

    if (pose_valid) {
        const Vec3 source_origin = chassis.position;
        const Quat source_orientation = chassis.orientation;
        const Vec3 final_position =
            report.mode == AssimilationReport::Mode::HardReset
                ? target_position
                : LerpVec3(chassis.position, target_position, std::max(pose_blend, terrain_blend));
        const Quat final_orientation =
            report.mode == AssimilationReport::Mode::HardReset
                ? Normalize(target_orientation)
                : NlerpShortest(chassis.orientation, target_orientation, pose_blend);
        const Quat rotation_delta = final_orientation * Conjugate(source_orientation);
        const bool zero_velocities = report.mode == AssimilationReport::Mode::HardReset;
        ApplyRigidTransformToRobot(world, scene, source_origin, rotation_delta, final_position, zero_velocities);
    }

    if (twist_valid) {
        if (report.mode == AssimilationReport::Mode::HardReset) {
            chassis.velocity = target_linear_velocity;
            chassis.angularVelocity = target_angular_velocity;
        } else {
            chassis.velocity = LerpVec3(chassis.velocity, target_linear_velocity, twist_blend);
            chassis.angularVelocity = LerpVec3(chassis.angularVelocity, target_angular_velocity, twist_blend);
        }
    }

    if (report.mode == AssimilationReport::Mode::HardReset) {
        for (std::uint32_t i = 0; i < world.GetBodyCount(); ++i) {
            Body& body = world.GetBody(i);
            body.isSleeping = false;
            body.sleepCounter = 0;
        }
        chassis.force = {};
        chassis.torque = {};
    }

    if ((correction.flags & physics_sim::kStateCorrectionContactValid) != 0) {
        terrain_samples = BuildTerrainSamples(world, scene, correction);
    }

    const Vec3 patch_center = pose_valid ? target_position : chassis.position;
    if (profiler != nullptr) {
        const auto scope = profiler->scope(11); // serve.terrain_patch_update
        terrain_patch.update(
            world, patch_center, target_terrain_height, target_terrain_normal, terrain_samples, terrain_age_seconds);
        SyncTerrainHeightfield(world, terrain_patch);
        (void)scope;
    } else {
        terrain_patch.update(world, patch_center, target_terrain_height, target_terrain_normal, terrain_samples, terrain_age_seconds);
        SyncTerrainHeightfield(world, terrain_patch);
    }

    assimilation_state.last_report = report;
    std::cout << "[serve] correction mode="
              << (report.mode == AssimilationReport::Mode::Soft
                      ? "soft"
                      : report.mode == AssimilationReport::Mode::Strong ? "strong" : "hard_reset")
              << " pos_err=" << report.position_error
              << " yaw_err=" << report.orientation_error_rad
              << " vel_err=" << report.linear_velocity_error
              << " terrain_err=" << report.terrain_error
              << " support=" << report.contact_support_weight
              << " strength=" << report.correction_strength
              << "\n";
    return report;
}

std::optional<physics_sim::ObstacleFootprint> BuildObstacleFootprint(const Body& body) {
    if (body.shape == ShapeType::Plane) {
        return std::nullopt;
    }

    physics_sim::ObstacleFootprint footprint{};
    footprint.center_x = body.position.x;
    footprint.center_z = body.position.z;
    footprint.yaw_rad = WrapAngleRad(YawAboutSimUp(body.orientation));

    switch (body.shape) {
        case ShapeType::Box:
        case ShapeType::Compound:
            footprint.half_extent_x = body.halfExtents.x;
            footprint.half_extent_z = body.halfExtents.z;
            break;
        case ShapeType::Sphere:
            footprint.half_extent_x = body.radius;
            footprint.half_extent_z = body.radius;
            footprint.yaw_rad = 0.0f;
            break;
        case ShapeType::Capsule:
        case ShapeType::Cylinder:
        case ShapeType::HalfCylinder: {
            const float extent = std::max(body.radius, body.halfHeight + body.radius);
            footprint.half_extent_x = extent;
            footprint.half_extent_z = extent;
            footprint.yaw_rad = 0.0f;
            break;
        }
        default:
            return std::nullopt;
    }

    if (footprint.half_extent_x <= 0.0f || footprint.half_extent_z <= 0.0f) {
        return std::nullopt;
    }
    return footprint;
}

std::vector<std::uint32_t> CollectObstacleBodyIds(const World& world, const HexapodSceneObjects& scene) {
    std::vector<std::uint32_t> out{};
    std::vector<bool> robot_body_flags(world.GetBodyCount(), false);
    for (const std::uint32_t id : scene.body_ids) {
        if (id < robot_body_flags.size()) {
            robot_body_flags[id] = true;
        }
    }

    for (std::uint32_t body_id = 0; body_id < world.GetBodyCount(); ++body_id) {
        if (robot_body_flags[body_id]) {
            continue;
        }
        if (world.IsTerrainAttachmentBody(body_id)) {
            continue;
        }
        const Body& body = world.GetBody(body_id);
        if (body.shape == ShapeType::Plane) {
            continue;
        }
        if (BuildObstacleFootprint(body).has_value()) {
            out.push_back(body_id);
        }
    }
    return out;
}

void FillObstacleFootprints(const World& world,
                            const std::vector<std::uint32_t>& obstacle_body_ids,
                            physics_sim::StateResponse& rsp) {
    rsp.obstacle_count = 0;
    for (const std::uint32_t body_id : obstacle_body_ids) {
        if (rsp.obstacle_count >= physics_sim::kMaxObstacleFootprints) {
            break;
        }
        const auto footprint = BuildObstacleFootprint(world.GetBody(body_id));
        if (!footprint.has_value()) {
            continue;
        }
        rsp.obstacles[static_cast<std::size_t>(rsp.obstacle_count)] = *footprint;
        ++rsp.obstacle_count;
    }
}

void EmitObstacleBodies(FrameSink& sink, const World& world, const std::vector<std::uint32_t>& obstacle_body_ids) {
    for (const std::uint32_t body_id : obstacle_body_ids) {
        sink.emit_body(body_id, world.GetBody(body_id));
    }
}

void FillFootContactsFromManifolds(
    const World& world,
    const HexapodSceneObjects& scene,
    std::array<std::uint8_t, 6>& out_contact,
    std::array<std::array<float, 3>, 6>& out_normals) {
    out_contact.fill(0);
    for (auto& n : out_normals) {
        n = {0.0f, 1.0f, 0.0f};
    }
    std::array<Vec3, 6> accum{};
    std::array<int, 6> counts{};

    Vec3 plane_up{0.0f, 1.0f, 0.0f};
    const Body& plane_body = world.GetBody(scene.plane);
    if (plane_body.shape == ShapeType::Plane) {
        plane_up = Normalize(plane_body.planeNormal);
    }

    for (const Manifold& manifold : world.DebugManifolds()) {
        const bool a_is_ground = manifold.a == scene.plane || world.IsTerrainAttachmentBody(manifold.a);
        const bool b_is_ground = manifold.b == scene.plane || world.IsTerrainAttachmentBody(manifold.b);
        if (a_is_ground == b_is_ground) {
            continue;
        }
        const std::uint32_t dyn_id = a_is_ground ? manifold.b : manifold.a;
        Vec3 n = manifold.normal;
        if (Dot(n, plane_up) < 0.0f) {
            n = n * -1.0f;
        }
        for (std::size_t leg = 0; leg < scene.legs.size(); ++leg) {
            if (scene.legs[leg].tibia != dyn_id) {
                continue;
            }
            if (manifold.contacts.empty()) {
                continue;
            }
            out_contact[leg] = 1;
            accum[leg] = AverageUnitNormal(accum[leg], n);
            ++counts[leg];
            break;
        }
    }
    for (std::size_t leg = 0; leg < 6; ++leg) {
        if (counts[leg] > 0) {
            const Vec3 u = Normalize(accum[leg]);
            out_normals[leg] = {u.x, u.y, u.z};
        }
    }
}

} // namespace

int RunPhysicsServeMode(std::uint16_t listen_port,
                        SinkKind preview_sink,
                        const std::string& preview_udp_host,
                        int preview_udp_port,
                        const std::string& scene_file) {
    const int fd = OpenUdpSocketWithRetry();
    if (fd < 0) {
        std::cerr << "[serve] socket() failed\n";
        std::perror("[serve] socket");
        return 1;
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(listen_port);

    if (::bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
        std::cerr << "[serve] bind port " << listen_port << " failed\n";
        ::close(fd);
        return 1;
    }

    // Non-blocking socket; we gate recvfrom() with poll() so the serve loop spends
    // its idle time in poll() (a single syscall with kernel-side timeout) instead
    // of inside a 1-second blocking recvfrom(). This dramatically reduces the wall
    // time charged to the `serve.recv` profile section without changing CPU work.
    {
        const int flags = ::fcntl(fd, F_GETFL, 0);
        if (flags < 0 || ::fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) {
            std::perror("[serve] fcntl(O_NONBLOCK)");
        }
    }

    World world(Vec3{0.0f, -9.81f, 0.0f});
    HexapodSceneObjects scene = BuildHexapodScene(world);
    RelaxBuiltInHexapodServos(world, scene);
    int solver_iterations = 8;
    AssimilationState assimilation_state{};
    TerrainPatchConfig terrain_config{};
    TerrainPatchSeed terrain_seed{};

    if (!scene_file.empty()) {
        std::string error;
        int appended_joints = 0;
        if (!AppendWorldFromMinphysSceneJsonFile(
                scene_file, world, solver_iterations, error, &appended_joints, &terrain_config, &terrain_seed)) {
            std::cerr << "[serve] failed to append scene file '" << scene_file << "': " << error << "\n";
            ::close(fd);
            return 1;
        }
        std::cout << "[serve] appended scene file '" << scene_file << "' joints=" << appended_joints << "\n";
    }

    TerrainPatch terrain_patch{terrain_config};
    const Vec3 initial_center = world.GetBody(scene.body).position;
    const Vec3 patch_center = terrain_seed.has_center ? terrain_seed.center : initial_center;
    const float plane_height = terrain_seed.has_plane_height ? terrain_seed.plane_height_m : world.GetBody(scene.plane).planeOffset;
    const Vec3 plane_normal = terrain_seed.has_plane_normal ? terrain_seed.plane_normal : world.GetBody(scene.plane).planeNormal;
    terrain_patch.initialize(world, patch_center, plane_height, plane_normal);
    SyncTerrainHeightfield(world, terrain_patch);
    const std::vector<std::uint32_t> obstacle_body_ids = CollectObstacleBodyIds(world, scene);

    JointSolverConfig joint_cfg = world.GetJointSolverConfig();
    joint_cfg.servoPositionPasses = 8;
    joint_cfg.hingeAnchorBiasFactor = 0.25f;
    joint_cfg.hingeAnchorDampingFactor = 0.3f;
    world.SetJointSolverConfig(joint_cfg);

    const std::array<std::uint32_t, 18> wire_joints = ServoJointIdsInWireOrder(scene);
    std::array<float, 18> prev_angles{};
    for (std::size_t i = 0; i < wire_joints.size(); ++i) {
        prev_angles[i] = world.GetServoJointAngle(wire_joints[i]);
    }

    bool configured = false;
    std::vector<std::byte> recv_buf(kRecvBufBytes);

    std::unique_ptr<FrameSink> visual = MakeFrameSink(preview_sink, preview_udp_host, preview_udp_port);
    int preview_frame = 0;
    float preview_sim_time_s = 0.0f;

    std::cout << "[hexapod-physics-sim] serve mode UDP *:" << listen_port;
    if (preview_sink == SinkKind::Udp) {
        std::cout << "  preview UDP -> " << preview_udp_host << ":" << preview_udp_port;
    }
    if (!scene_file.empty()) {
        std::cout << "  scene_file=" << scene_file;
    }
    std::cout << "\n";

    ProcessResourceDiagnosticsState resource_diag{};
    enum class ServeSection : std::size_t {
        Recv = 0,
        DecodeConfig,
        ApplyConfig,
        DecodeCorrection,
        ApplyCorrection,
        DecodeStep,
        ApplyJointTargets,
        PhysicsStep,
        PackResponse,
        SendResponse,
        PreviewEmit,
        MatrixLidarScan,
        MatrixLidarFusion,
        TerrainPatchUpdate,
        Count,
    };

    static constexpr std::array<const char*, static_cast<std::size_t>(ServeSection::Count)> kServeSectionLabels = {
        "serve.recv",
        "serve.decode_config",
        "serve.apply_config",
        "serve.decode_correction",
        "serve.apply_correction",
        "serve.decode_step",
        "serve.apply_joint_targets",
        "serve.physics_step",
        "serve.pack_response",
        "serve.send_response",
        "serve.preview_emit",
        "serve.matrix_lidar_scan",
        "serve.matrix_lidar_fusion",
        "serve.terrain_patch_update",
    };
    resource_monitoring::SectionProfiler<kServeSectionLabels.size()> serve_profiler{kServeSectionLabels};

    auto logResourceSnapshot = [&](const char* prefix = "[resource]") {
        if (stdout == nullptr) {
            return;
        }
        const DemoSteadyClock::time_point now = DemoSteadyClock::now();
        if (resource_diag.next_emit_at.time_since_epoch().count() != 0 && now < resource_diag.next_emit_at) {
            return;
        }

        const resource_monitoring::ProcessResourceSnapshot process_snapshot = resource_diag.sampler.sample();
        const auto serve_sections = serve_profiler.topSections(10);
        const auto world_sections = world.SnapshotResourceSections();
        const auto topology = world.SnapshotTopology();
        std::fprintf(stdout,
                     "%s process_resource=%s serve_sections=%s world_sections=%s topology={bodies=%u dynamic=%u awake=%u contacts=%u manifolds=%u islands=%u max_island_bodies=%u max_island_manifolds=%u max_island_joints=%u max_island_servos=%u} terrain={adds=%llu cells=%llu emitted=%llu merges=%llu cache_hits=%llu dirty=%llu} face4={attempted=%llu used=%llu fallback_block2=%llu fallback_scalar=%llu blocked_coherence=%llu}\n",
                     prefix,
                     resource_monitoring::formatProcessResourceSnapshot(process_snapshot).c_str(),
                     resource_monitoring::formatResourceSectionSummary(serve_sections).c_str(),
                     resource_monitoring::formatResourceSectionSummary(world_sections).c_str(),
                     topology.bodyCount,
                     topology.dynamicBodyCount,
                     topology.awakeBodyCount,
                     topology.contactCount,
                     topology.manifoldCount,
                     topology.islandCount,
                     topology.maxIslandBodyCount,
                     topology.maxIslandManifoldCount,
                     topology.maxIslandJointCount,
                     topology.maxIslandServoCount,
#if MINPHYS3D_SOLVER_TELEMETRY_ENABLED
                     static_cast<unsigned long long>(world.GetSolverTelemetry().terrainContactAdds),
                     static_cast<unsigned long long>(world.GetSolverTelemetry().terrainCellsTested),
                     static_cast<unsigned long long>(world.GetSolverTelemetry().terrainContactsEmitted),
                     static_cast<unsigned long long>(world.GetSolverTelemetry().terrainManifoldMerges),
                     static_cast<unsigned long long>(world.GetSolverTelemetry().terrainCacheHits),
                     static_cast<unsigned long long>(world.GetSolverTelemetry().terrainDirtyCellRefreshes),
                     static_cast<unsigned long long>(world.GetSolverTelemetry().face4Attempted),
                     static_cast<unsigned long long>(world.GetSolverTelemetry().face4Used),
                     static_cast<unsigned long long>(world.GetSolverTelemetry().face4FallbackToBlock2),
                     static_cast<unsigned long long>(world.GetSolverTelemetry().face4FallbackToScalar),
                     static_cast<unsigned long long>(world.GetSolverTelemetry().face4BlockedByFrictionCoherenceGate)
#else
                     0ULL,
                     0ULL,
                     0ULL,
                     0ULL,
                     0ULL,
                     0ULL,
                     0ULL,
                     0ULL,
                     0ULL,
                     0ULL,
                     0ULL
#endif
        );
        std::fflush(stdout);
        resource_diag.next_emit_at = now + resource_diag.emit_interval;
    };
    logResourceSnapshot();

    CachedMatrixLidarFrame cached_lidar_frame{};
    std::uint64_t lidar_sim_time_us = 0;
    std::uint64_t next_lidar_capture_us = 0;

    auto refreshMatrixLidarFrame = [&](physics_sim::StateResponse& rsp,
                                       const Body& chassis,
                                       const float fusion_age_seconds,
                                       const bool advance_sensor_time) {
        if (advance_sensor_time && std::isfinite(fusion_age_seconds) && fusion_age_seconds > 0.0f) {
            lidar_sim_time_us += static_cast<std::uint64_t>(
                std::llround(static_cast<double>(fusion_age_seconds) * 1.0e6));
        }

        const bool need_capture = !cached_lidar_frame.valid || lidar_sim_time_us >= next_lidar_capture_us;
        if (!need_capture) {
            writeCachedMatrixLidarFrame(cached_lidar_frame, rsp);
            return;
        }

        {
            const auto scope = serve_profiler.scope(static_cast<std::size_t>(ServeSection::MatrixLidarScan));
            FillSimMatrixLidar64x8(world, scene, terrain_patch, chassis, rsp);
            (void)scope;
        }
        rsp.matrix_lidar_timestamp_us = lidar_sim_time_us == 0 ? 1ULL : lidar_sim_time_us;

        if (terrain_patch.config().lidar_fusion_enable) {
            std::vector<TerrainSample> lidar_samples;
            {
                const auto scope = serve_profiler.scope(static_cast<std::size_t>(ServeSection::MatrixLidarFusion));
                AppendMatrixLidarTerrainSamples(
                    world, scene, terrain_patch, chassis, rsp, terrain_patch.config(), lidar_samples);
                ApplyContactArbitrationToLidarSamples(
                    world, scene, assimilation_state, terrain_patch.config(), lidar_samples);
                (void)scope;
            }
            if (!lidar_samples.empty()) {
                Body& plane_body = world.GetBody(scene.plane);
                // LiDAR fusion intentionally follows response generation: the response reports what this
                // step saw, and the fused terrain influences subsequent contacts/rays.
                const auto scope =
                    serve_profiler.scope(static_cast<std::size_t>(ServeSection::TerrainPatchUpdate));
                terrain_patch.update(world,
                                     world.GetBody(scene.body).position,
                                     plane_body.planeOffset,
                                     plane_body.planeNormal,
                                     lidar_samples,
                                     fusion_age_seconds);
                SyncTerrainHeightfield(world, terrain_patch);
                (void)scope;
            }
        }

        cached_lidar_frame.valid = true;
        cached_lidar_frame.timestamp_us = rsp.matrix_lidar_timestamp_us;
        cached_lidar_frame.model = static_cast<std::uint8_t>(rsp.matrix_lidar_model);
        cached_lidar_frame.cols = rsp.matrix_lidar_cols;
        cached_lidar_frame.rows = rsp.matrix_lidar_rows;
        cached_lidar_frame.ranges_mm = rsp.matrix_lidar_ranges_mm;

        if (next_lidar_capture_us == 0) {
            next_lidar_capture_us = kMatrixLidarFramePeriodUs;
        }
        while (next_lidar_capture_us <= lidar_sim_time_us) {
            next_lidar_capture_us += kMatrixLidarFramePeriodUs;
        }
    };

    // Hoisted out of the main loop: re-zeroing sockaddr_in every iteration was 8+4
    // bytes of pointless writes. recvfrom() will overwrite it on success anyway.
    sockaddr_in peer{};
    socklen_t peer_len = sizeof(peer);
    // The previous code called logResourceSnapshot() on every 1-second timeout. With
    // non-blocking poll() the "no data" branch fires hundreds of times per second; gate
    // the diagnostic output to at most once per kIdleLogInterval.
    constexpr std::chrono::seconds kIdleLogInterval{1};
    auto last_idle_log = std::chrono::steady_clock::now() - kIdleLogInterval;
    // Reuse a single StateResponse buffer across responses. Each path fully overwrites
    // the fields it cares about, so we just zero the trailing array fields that aren't
    // unconditionally written. This avoids the ~1.6 KB zero-init that an in-branch
    // `StateResponse rsp{}` paid on every step / snapshot response.
    physics_sim::StateResponse rsp{};

    for (;;) {
        ssize_t n = 0;
        {
            const auto scope = serve_profiler.scope(0); // serve.recv
            // 5 ms poll wait: short enough that a freshly-arrived datagram is processed
            // promptly, long enough that the serve thread doesn't burn CPU spinning when
            // the controller is idle. POSIX poll(timeout) is one syscall whose kernel
            // wait is effectively free CPU.
            struct pollfd pfd{};
            pfd.fd = fd;
            pfd.events = POLLIN;
            const int poll_rc = ::poll(&pfd, 1, 5);
            if (poll_rc <= 0 || (pfd.revents & POLLIN) == 0) {
                (void)scope;
                const auto now = std::chrono::steady_clock::now();
                if (now - last_idle_log >= kIdleLogInterval) {
                    logResourceSnapshot();
                    last_idle_log = now;
                }
                continue;
            }
            peer_len = sizeof(peer);
            n = ::recvfrom(
                fd,
                reinterpret_cast<char*>(recv_buf.data()),
                recv_buf.size(),
                0,
                reinterpret_cast<sockaddr*>(&peer),
                &peer_len);
            (void)scope;
        }
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
                // poll() said readable but the datagram was already drained
                // (extremely rare race). Loop back and poll again.
                continue;
            }
            std::cerr << "[serve] recvfrom failed\n";
            ::close(fd);
            return 1;
        }

        if (n < 1) {
            continue;
        }

        const auto* bytes = reinterpret_cast<const std::byte*>(recv_buf.data());
        const std::uint8_t mtype = static_cast<std::uint8_t>(*bytes);

        if (mtype == static_cast<std::uint8_t>(physics_sim::MessageType::ConfigCommand)) {
            physics_sim::ConfigCommand cmd{};
            {
                const auto scope = serve_profiler.scope(1); // serve.decode_config
                if (!physics_sim::tryDecodeConfigCommand(bytes, static_cast<std::size_t>(n), cmd)) {
                    (void)scope;
                    logResourceSnapshot();
                    continue;
                }
                (void)scope;
            }
            {
                const auto scope = serve_profiler.scope(2); // serve.apply_config
                world.SetGravity(Vec3{cmd.gravity[0], cmd.gravity[1], cmd.gravity[2]});
                solver_iterations = cmd.solver_iterations > 0 ? cmd.solver_iterations : 8;
                configured = true;
                (void)scope;
            }

            physics_sim::ConfigAck ack{};
            ack.message_type = static_cast<std::uint8_t>(physics_sim::MessageType::ConfigAck);
            ack.body_count = world.GetBodyCount() - (world.HasTerrainHeightfield() ? 1u : 0u);
            ack.joint_count = world.GetServoJointCount();
            (void)::sendto(
                fd,
                &ack,
                physics_sim::kConfigAckBytes,
                0,
                reinterpret_cast<sockaddr*>(&peer),
                peer_len);
            logResourceSnapshot();
            continue;
        }

        if (mtype == static_cast<std::uint8_t>(physics_sim::MessageType::StateCorrection)) {
            physics_sim::StateCorrection correction{};
            {
                const auto scope = serve_profiler.scope(3); // serve.decode_correction
                if (!physics_sim::tryDecodeStateCorrection(bytes, static_cast<std::size_t>(n), correction)) {
                    (void)scope;
                    logResourceSnapshot();
                    continue;
                }
                (void)scope;
            }
            {
                const auto scope = serve_profiler.scope(4); // serve.apply_correction
                (void)ApplyStateCorrection(world, scene, correction, assimilation_state, terrain_patch, &serve_profiler);
                (void)scope;
            }
            cached_lidar_frame.valid = false;
            lidar_sim_time_us = std::max(lidar_sim_time_us + 1, next_lidar_capture_us);
            logResourceSnapshot();
            continue;
        }

        if (mtype == static_cast<std::uint8_t>(physics_sim::MessageType::StepCommand)) {
            if (!configured) {
                logResourceSnapshot();
                continue;
            }
            physics_sim::StepCommand step{};
            {
                const auto scope = serve_profiler.scope(5); // serve.decode_step
                if (!physics_sim::tryDecodeStepCommand(bytes, static_cast<std::size_t>(n), step)) {
                    (void)scope;
                    logResourceSnapshot();
                    continue;
                }
                (void)scope;
            }

            // sequence_id == 0: return current state without stepping or applying joint_targets.
            if (step.sequence_id == 0) {
                // Reset the reused buffer. Memset is faster than {}-init since it bypasses
                // the per-field default-init the aggregate would otherwise generate.
                std::memset(&rsp, 0, sizeof(rsp));
                rsp.message_type = static_cast<std::uint8_t>(physics_sim::MessageType::StateResponse);
                rsp.sequence_id = 0;

                const Body& chassis = world.GetBody(scene.body);
                rsp.body_position = {chassis.position.x, chassis.position.y, chassis.position.z};
                rsp.body_orientation = {
                    chassis.orientation.w,
                    chassis.orientation.x,
                    chassis.orientation.y,
                    chassis.orientation.z,
                };
                rsp.body_linear_velocity = {chassis.velocity.x, chassis.velocity.y, chassis.velocity.z};
                rsp.body_angular_velocity = {
                    chassis.angularVelocity.x,
                    chassis.angularVelocity.y,
                    chassis.angularVelocity.z,
                };

                for (std::size_t i = 0; i < wire_joints.size(); ++i) {
                    rsp.joint_angles[i] = world.GetServoJointAngle(wire_joints[i]);
                    rsp.joint_velocities[i] = 0.0f;
                }
                {
                    const auto scope = serve_profiler.scope(8); // serve.pack_response
                    FillFootContactsFromManifolds(world, scene, rsp.foot_contacts, rsp.foot_contact_normals);
                    FillObstacleFootprints(world, obstacle_body_ids, rsp);
                    refreshMatrixLidarFrame(rsp, world.GetBody(scene.body), 0.0f, false);
                    (void)scope;
                }
                {
                    const auto scope = serve_profiler.scope(9); // serve.send_response
                    (void)::sendto(
                        fd,
                        &rsp,
                        physics_sim::kStateResponseBytes,
                        0,
                        reinterpret_cast<sockaddr*>(&peer),
                        peer_len);
                    (void)scope;
                }
                logResourceSnapshot();
                continue;
            }

            if (step.dt_seconds <= 0.0f || !std::isfinite(step.dt_seconds)) {
                logResourceSnapshot();
                continue;
            }

            {
                const auto scope = serve_profiler.scope(6); // serve.apply_joint_targets
                for (std::size_t i = 0; i < wire_joints.size(); ++i) {
                    ServoJoint& sj = world.GetServoJointMutable(wire_joints[i]);
                    sj.targetAngle = step.joint_targets[i];
                }
                (void)scope;
            }

            for (std::size_t i = 0; i < wire_joints.size(); ++i) {
                prev_angles[i] = world.GetServoJointAngle(wire_joints[i]);
            }

            const int physics_substeps = std::clamp(
                static_cast<int>(std::ceil(step.dt_seconds / kServeMaxPhysicsSubstepSeconds)),
                1,
                kServeMaxPhysicsSubsteps);
            const float substep_dt = step.dt_seconds / static_cast<float>(physics_substeps);
            {
                const auto scope = serve_profiler.scope(7); // serve.physics_step
                for (int substep = 0; substep < physics_substeps; ++substep) {
                    world.Step(substep_dt, solver_iterations);
                }
                (void)scope;
            }

            std::memset(&rsp, 0, sizeof(rsp));
            rsp.message_type = static_cast<std::uint8_t>(physics_sim::MessageType::StateResponse);
            rsp.sequence_id = step.sequence_id;

            const Body& chassis = world.GetBody(scene.body);
            rsp.body_position = {chassis.position.x, chassis.position.y, chassis.position.z};
            rsp.body_orientation = {
                chassis.orientation.w,
                chassis.orientation.x,
                chassis.orientation.y,
                chassis.orientation.z,
            };
            rsp.body_linear_velocity = {chassis.velocity.x, chassis.velocity.y, chassis.velocity.z};
            rsp.body_angular_velocity = {
                chassis.angularVelocity.x,
                chassis.angularVelocity.y,
                chassis.angularVelocity.z,
            };

            const float inv_dt = 1.0f / step.dt_seconds;
            for (std::size_t i = 0; i < wire_joints.size(); ++i) {
                const float ang_after = world.GetServoJointAngle(wire_joints[i]);
                rsp.joint_angles[i] = ang_after;
                rsp.joint_velocities[i] = (ang_after - prev_angles[i]) * inv_dt;
                prev_angles[i] = ang_after;
            }

            {
                const auto scope = serve_profiler.scope(8); // serve.pack_response
                FillFootContactsFromManifolds(world, scene, rsp.foot_contacts, rsp.foot_contact_normals);
                FillObstacleFootprints(world, obstacle_body_ids, rsp);
                refreshMatrixLidarFrame(rsp, world.GetBody(scene.body), step.dt_seconds, true);
                (void)scope;
            }

            {
                const auto scope = serve_profiler.scope(9); // serve.send_response
                (void)::sendto(
                    fd,
                    &rsp,
                    physics_sim::kStateResponseBytes,
                    0,
                    reinterpret_cast<sockaddr*>(&peer),
                    peer_len);
                (void)scope;
            }

            {
                const auto scope = serve_profiler.scope(10); // serve.preview_emit
                preview_sim_time_s += step.dt_seconds;
                visual->begin_frame(preview_frame, preview_sim_time_s);
                visual->emit_terrain_patch(terrain_patch);
                EmitSceneBodies(*visual, world, scene);
                EmitObstacleBodies(*visual, world, obstacle_body_ids);
                visual->end_frame();
                ++preview_frame;
                (void)scope;
            }
            logResourceSnapshot();
            continue;
        }

        logResourceSnapshot();
    }
}

} // namespace minphys3d::demo
