#include "demo/scenes.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <system_error>
#include <thread>
#include <utility>
#include <vector>

#include "demo/frame_sink.hpp"
#include "minphys3d/core/body.hpp"
#include "minphys3d/core/world.hpp"

namespace minphys3d::demo {
namespace {

constexpr float kPi = 3.14159265358979323846f;

const char* SinkName(SinkKind sink_kind) {
    switch (sink_kind) {
        case SinkKind::Dummy:
            return "dummy";
        case SinkKind::Udp:
            return "udp";
    }
    return "unknown";
}

std::filesystem::path BuildDebugLogPath() {
    std::filesystem::path log_dir = "logs";
    const std::filesystem::path cwd_name = std::filesystem::current_path().filename();
    if (cwd_name != "hexapod-physics-sim") {
        log_dir = std::filesystem::path("hexapod-physics-sim") / log_dir;
    }
    std::error_code ec;
    std::filesystem::create_directories(log_dir, ec);
    if (ec) {
        return "hexapod-physics-sim.log";
    }
    return log_dir / "latest.log";
}

class RunLog {
public:
    explicit RunLog(std::filesystem::path path) : path_(std::move(path)) {
        file_ = std::fopen(path_.string().c_str(), "w");
    }

    ~RunLog() {
        if (file_ != nullptr) {
            std::fclose(file_);
        }
    }

    RunLog(const RunLog&) = delete;
    RunLog& operator=(const RunLog&) = delete;

    std::FILE* stream() const { return file_; }
    bool available() const { return file_ != nullptr; }
    const std::filesystem::path& path() const { return path_; }

private:
    std::filesystem::path path_;
    std::FILE* file_ = nullptr;
};

struct LegLinkIds {
    std::uint32_t coxa = 0;
    std::uint32_t femur = 0;
    std::uint32_t tibia = 0;
    std::uint32_t bodyToCoxaJoint = World::kInvalidJointId;
    std::uint32_t coxaToFemurJoint = World::kInvalidJointId;
    std::uint32_t femurToTibiaJoint = World::kInvalidJointId;
};

struct HexapodSceneObjects {
    std::uint32_t plane = 0;
    std::uint32_t body = 0;
    std::array<LegLinkIds, 6> legs{};
    std::vector<std::uint32_t> body_ids{};
};

struct HexapodLegSpec {
    Vec3 mountOffsetBody{};
    float mountAngleDegrees = 0.0f;
};

const std::array<HexapodLegSpec, 6> kHexapodLegSpecs{{
    {{0.063f, -0.007f, -0.0835f}, 143.0f},
    {{-0.063f, -0.007f, -0.0835f}, 217.0f},
    {{0.0815f, -0.007f, 0.0f}, 90.0f},
    {{-0.0815f, -0.007f, 0.0f}, 270.0f},
    {{0.063f, -0.007f, 0.0835f}, 37.0f},
    {{-0.063f, -0.007f, 0.0835f}, 323.0f},
}};

constexpr float kBodyMass = 2.4f;
constexpr float kCoxaMass = 0.055f;
constexpr float kFemurMass = 0.070f;
constexpr float kTibiaMass = 0.055f;
constexpr float kFootMass = 0.008f;

constexpr float kCoxaLength = 0.043f;
constexpr float kFemurLength = 0.060f;
constexpr float kTibiaLength = 0.104f;
constexpr float kBodyToBottom = 0.040f;
constexpr float kFootRadius = 0.018f;
constexpr float kHipMountOutboard = 0.010f;
constexpr float kCoxaRenderLength = kCoxaLength;
constexpr float kFemurRenderLength = kFemurLength;
constexpr float kTibiaRenderLength = kTibiaLength;

constexpr float kNeutralFemurAngle = -0.20f;
constexpr float kNeutralTibiaAngle = -1.00f;

Quat QuaternionFromBasis(const Vec3& x_axis, const Vec3& y_axis, const Vec3& z_axis) {
    const float m00 = x_axis.x;
    const float m01 = y_axis.x;
    const float m02 = z_axis.x;
    const float m10 = x_axis.y;
    const float m11 = y_axis.y;
    const float m12 = z_axis.y;
    const float m20 = x_axis.z;
    const float m21 = y_axis.z;
    const float m22 = z_axis.z;

    const float trace = m00 + m11 + m22;
    if (trace > 0.0f) {
        const float s = std::sqrt(trace + 1.0f) * 2.0f;
        return Normalize(Quat{
            0.25f * s,
            (m21 - m12) / s,
            (m02 - m20) / s,
            (m10 - m01) / s,
        });
    }
    if (m00 > m11 && m00 > m22) {
        const float s = std::sqrt(1.0f + m00 - m11 - m22) * 2.0f;
        return Normalize(Quat{
            (m21 - m12) / s,
            0.25f * s,
            (m01 + m10) / s,
            (m02 + m20) / s,
        });
    }
    if (m11 > m22) {
        const float s = std::sqrt(1.0f + m11 - m00 - m22) * 2.0f;
        return Normalize(Quat{
            (m02 - m20) / s,
            (m01 + m10) / s,
            0.25f * s,
            (m12 + m21) / s,
        });
    }
    const float s = std::sqrt(1.0f + m22 - m00 - m11) * 2.0f;
    return Normalize(Quat{
        (m10 - m01) / s,
        (m02 + m20) / s,
        (m12 + m21) / s,
        0.25f * s,
    });
}

Quat OrientationFromLongAxisAndHint(const Vec3& long_axis, const Vec3& hint_axis) {
    Vec3 x_axis{};
    if (!TryNormalize(long_axis, x_axis)) {
        return {};
    }
    Vec3 y_axis = hint_axis - Dot(hint_axis, x_axis) * x_axis;
    if (!TryNormalize(y_axis, y_axis)) {
        const Vec3 fallback = (std::abs(x_axis.y) < 0.9f) ? Vec3{0.0f, 1.0f, 0.0f} : Vec3{1.0f, 0.0f, 0.0f};
        y_axis = fallback - Dot(fallback, x_axis) * x_axis;
        if (!TryNormalize(y_axis, y_axis)) {
            return {};
        }
    }
    Vec3 z_axis = Normalize(Cross(x_axis, y_axis));
    y_axis = Normalize(Cross(z_axis, x_axis));
    return QuaternionFromBasis(x_axis, y_axis, z_axis);
}

Vec3 LegAxisFromMountAngleDegrees(float angle_degrees) {
    const float radians = angle_degrees * kPi / 180.0f;
    return Normalize(Vec3{std::sin(radians), 0.0f, std::cos(radians)});
}

Vec3 LegPitchDirection(const Vec3& leg_axis, float angle_radians) {
    return Normalize(leg_axis * std::cos(angle_radians) + Vec3{0.0f, 1.0f, 0.0f} * std::sin(angle_radians));
}

Body MakeBoxLinkBody(const Vec3& position, const Quat& orientation, const Vec3& half_extents, float mass) {
    Body body;
    body.shape = ShapeType::Box;
    body.position = position;
    body.orientation = orientation;
    body.halfExtents = half_extents;
    body.mass = mass;
    body.restitution = 0.02f;
    body.staticFriction = 0.65f;
    body.dynamicFriction = 0.45f;
    body.linearDamping = 2.0f;
    body.angularDamping = 10.0f;
    return body;
}

CompoundChild MakeCompoundBoxChild(const Vec3& local_position, const Vec3& half_extents, const Quat& local_orientation = {}) {
    CompoundChild child;
    child.shape = ShapeType::Box;
    child.localPosition = local_position;
    child.localOrientation = local_orientation;
    child.halfExtents = half_extents;
    return child;
}

CompoundChild MakeCompoundSphereChild(const Vec3& local_position, float radius) {
    CompoundChild child;
    child.shape = ShapeType::Sphere;
    child.localPosition = local_position;
    child.radius = radius;
    return child;
}

Vec3 CompoundChildWorldPosition(const Body& body, std::size_t child_index) {
    if (child_index >= body.compoundChildren.size()) {
        return body.position;
    }
    return body.position + Rotate(body.orientation, body.compoundChildren[child_index].localPosition);
}

struct ContactSummary {
    int manifolds = 0;
    int points = 0;
    float totalNormalImpulse = 0.0f;
    float maxPenetration = 0.0f;
};

struct LegContactSummary {
    ContactSummary coxa{};
    ContactSummary femur{};
    ContactSummary tibia{};
};

ContactSummary* FindLegContactSummary(
    std::array<LegContactSummary, 6>& summaries,
    const HexapodSceneObjects& scene,
    std::uint32_t body_id) {
    for (std::size_t leg_index = 0; leg_index < scene.legs.size(); ++leg_index) {
        const LegLinkIds& leg = scene.legs[leg_index];
        if (leg.coxa == body_id) {
            return &summaries[leg_index].coxa;
        }
        if (leg.femur == body_id) {
            return &summaries[leg_index].femur;
        }
        if (leg.tibia == body_id) {
            return &summaries[leg_index].tibia;
        }
    }
    return nullptr;
}

std::array<LegContactSummary, 6> SummarizeGroundContacts(const World& world, const HexapodSceneObjects& scene) {
    std::array<LegContactSummary, 6> summaries{};
    for (const Manifold& manifold : world.DebugManifolds()) {
        const bool a_is_plane = manifold.a == scene.plane;
        const bool b_is_plane = manifold.b == scene.plane;
        if (a_is_plane == b_is_plane) {
            continue;
        }

        const std::uint32_t body_id = a_is_plane ? manifold.b : manifold.a;
        ContactSummary* summary = FindLegContactSummary(summaries, scene, body_id);
        if (summary == nullptr) {
            continue;
        }

        ++summary->manifolds;
        summary->points += static_cast<int>(manifold.contacts.size());
        for (const Contact& contact : manifold.contacts) {
            summary->totalNormalImpulse += std::max(contact.normalImpulseSum, 0.0f);
            summary->maxPenetration = std::max(summary->maxPenetration, contact.penetration);
        }
    }
    return summaries;
}

Body MakeCompoundChassisBody(const Vec3& position, float mass) {
    Body body;
    body.shape = ShapeType::Compound;
    body.position = position;
    body.mass = mass;
    body.restitution = 0.0f;
    body.staticFriction = 0.60f;
    body.dynamicFriction = 0.45f;
    body.linearDamping = 2.0f;
    body.angularDamping = 16.0f;
    body.compoundChildren = {
        MakeCompoundBoxChild({0.0f, -0.024f, 0.0f}, {0.050f, 0.010f, 0.052f}),
        MakeCompoundBoxChild({0.0f, -0.024f, 0.070f}, {0.050f, 0.010f, 0.018f}),
        MakeCompoundBoxChild({0.0f, -0.024f, -0.070f}, {0.050f, 0.010f, 0.018f}),
        MakeCompoundBoxChild({0.067f, -0.024f, 0.0f}, {0.009f, 0.010f, 0.048f}),
        MakeCompoundBoxChild({-0.067f, -0.024f, 0.0f}, {0.009f, 0.010f, 0.048f}),
        MakeCompoundBoxChild({0.0f, -0.010f, 0.0f}, {0.072f, 0.008f, 0.030f}),
        MakeCompoundBoxChild({0.0f, 0.012f, 0.0f}, {0.038f, 0.016f, 0.042f}),
    };
    return body;
}

float ComputeStandingBodyHeight() {
    const Vec3 leg_axis = LegAxisFromMountAngleDegrees(kHexapodLegSpecs.front().mountAngleDegrees);
    const Vec3 femur_direction = LegPitchDirection(leg_axis, kNeutralFemurAngle);
    const Vec3 tibia_direction = LegPitchDirection(leg_axis, kNeutralFemurAngle + kNeutralTibiaAngle);
    const Vec3 foot_center_relative =
        kHexapodLegSpecs.front().mountOffsetBody
        + leg_axis * kCoxaLength
        + femur_direction * kFemurLength
        + tibia_direction * kTibiaLength
        + Vec3{0.0f, -kFootRadius, 0.0f};
    return std::max(kBodyToBottom, kFootRadius - foot_center_relative.y + 0.001f);
}

HexapodSceneObjects BuildHexapodScene(World& world) {
    HexapodSceneObjects scene{};

    Body plane;
    plane.shape = ShapeType::Plane;
    plane.planeNormal = {0.0f, 1.0f, 0.0f};
    plane.planeOffset = 0.0f;
    plane.restitution = 0.05f;
    plane.staticFriction = 0.9f;
    plane.dynamicFriction = 0.65f;
    scene.plane = world.CreateBody(plane);
    scene.body_ids.push_back(scene.plane);

    const float body_height = ComputeStandingBodyHeight();
    Body chassis = MakeCompoundChassisBody({0.0f, body_height, 0.0f}, kBodyMass);
    scene.body = world.CreateBody(chassis);
    scene.body_ids.push_back(scene.body);

    constexpr float kCoxaHalfLength = 0.5f * kCoxaRenderLength;
    constexpr float kFemurHalfLength = 0.5f * kFemurRenderLength;
    constexpr float kTibiaHalfLength = 0.5f * kTibiaRenderLength;

    for (std::size_t leg_index = 0; leg_index < kHexapodLegSpecs.size(); ++leg_index) {
        const HexapodLegSpec& leg_spec = kHexapodLegSpecs[leg_index];
        const Vec3 leg_axis = LegAxisFromMountAngleDegrees(leg_spec.mountAngleDegrees);
        const Vec3 lateral_axis = Normalize(Cross(leg_axis, Vec3{0.0f, 1.0f, 0.0f}));
        const Vec3 hip_anchor = chassis.position + leg_spec.mountOffsetBody + leg_axis * kHipMountOutboard;
        const Vec3 femur_direction = LegPitchDirection(leg_axis, kNeutralFemurAngle);
        const Vec3 tibia_direction = LegPitchDirection(leg_axis, kNeutralFemurAngle + kNeutralTibiaAngle);

        const Vec3 coxa_center = hip_anchor + leg_axis * kCoxaHalfLength;
        const Vec3 femur_anchor = hip_anchor + leg_axis * kCoxaLength;
        const Vec3 femur_center = femur_anchor + femur_direction * kFemurHalfLength;
        const Vec3 tibia_anchor = femur_anchor + femur_direction * kFemurLength;
        const Vec3 tibia_center = tibia_anchor + tibia_direction * kTibiaHalfLength;
        const Vec3 tibia_tip = tibia_anchor + tibia_direction * kTibiaLength;
        const Vec3 foot_center = tibia_tip + Vec3{0.0f, -kFootRadius, 0.0f};

        Body coxa = MakeBoxLinkBody(
            coxa_center,
            OrientationFromLongAxisAndHint(leg_axis, {0.0f, 1.0f, 0.0f}),
            {kCoxaHalfLength, 0.009f, 0.011f},
            kCoxaMass);
        const std::uint32_t coxa_id = world.CreateBody(coxa);

        Body femur = MakeBoxLinkBody(
            femur_center,
            OrientationFromLongAxisAndHint(femur_direction, lateral_axis),
            {kFemurHalfLength, 0.010f, 0.012f},
            kFemurMass);
        const std::uint32_t femur_id = world.CreateBody(femur);

        const Quat tibia_orientation = OrientationFromLongAxisAndHint(tibia_direction, lateral_axis);
        const Vec3 foot_local_offset = Rotate(Conjugate(tibia_orientation), foot_center - tibia_center);

        Body tibia;
        tibia.shape = ShapeType::Compound;
        tibia.position = tibia_center;
        tibia.orientation = tibia_orientation;
        tibia.mass = kTibiaMass + kFootMass;
        tibia.restitution = 0.0f;
        tibia.staticFriction = 0.60f;
        tibia.dynamicFriction = 0.45f;
        tibia.linearDamping = 2.0f;
        tibia.angularDamping = 10.0f;
        tibia.compoundChildren = {
            MakeCompoundBoxChild({0.0f, 0.0f, 0.0f}, {kTibiaHalfLength, 0.009f, 0.009f}),
            MakeCompoundSphereChild(foot_local_offset, kFootRadius),
        };
        const std::uint32_t tibia_id = world.CreateBody(tibia);

        // Servo limits: maxServoTorque clamps accumulated angular impulse in the joint solver.
        // Previous values (0.001 / 0.003) were far below tests/test_servo_chain_stability.cpp
        // (order 10–20) and could not hold the leg under chassis weight.
        const std::uint32_t body_to_coxa_joint = world.CreateServoJoint(
            scene.body,
            coxa_id,
            hip_anchor,
            {0.0f, 1.0f, 0.0f},
            0.0f,
            0.06f,
            8.0f,
            3.0f);

        const std::uint32_t coxa_to_femur_joint = world.CreateServoJoint(
            coxa_id,
            femur_id,
            femur_anchor,
            lateral_axis,
            0.0f,
            0.15f,
            12.0f,
            4.0f);

        const std::uint32_t femur_to_tibia_joint = world.CreateServoJoint(
            femur_id,
            tibia_id,
            tibia_anchor,
            lateral_axis,
            0.0f,
            0.15f,
            12.0f,
            3.5f);

        scene.legs[leg_index] = LegLinkIds{
            coxa_id,
            femur_id,
            tibia_id,
            body_to_coxa_joint,
            coxa_to_femur_joint,
            femur_to_tibia_joint,
        };
        scene.body_ids.push_back(coxa_id);
        scene.body_ids.push_back(femur_id);
        scene.body_ids.push_back(tibia_id);
    }

    return scene;
}

void EmitSceneBodies(FrameSink& sink, const World& world, const HexapodSceneObjects& scene) {
    for (const std::uint32_t body_id : scene.body_ids) {
        sink.emit_body(body_id, world.GetBody(body_id));
    }
}

void RelaxBuiltInHexapodServos(World& world, const HexapodSceneObjects& scene) {
    // The articulated hexapod never had explicit gait target updates; the live/zero-G presets are
    // static pose previews. Lower the shared P/D gains enough to stay stable without the old angle
    // snap, then add a uniform integral term so the preview can build holding torque under gravity
    // instead of slowly sagging onto the floor.
    constexpr float kServoGainScale = 0.54f;
    constexpr float kIntegralToPositionRatio = 0.25f;
    constexpr float kAngleStabilizationScale = 0.0f;
    const std::array<std::uint32_t, 18> servo_joint_ids{
        scene.legs[0].bodyToCoxaJoint,
        scene.legs[0].coxaToFemurJoint,
        scene.legs[0].femurToTibiaJoint,
        scene.legs[1].bodyToCoxaJoint,
        scene.legs[1].coxaToFemurJoint,
        scene.legs[1].femurToTibiaJoint,
        scene.legs[2].bodyToCoxaJoint,
        scene.legs[2].coxaToFemurJoint,
        scene.legs[2].femurToTibiaJoint,
        scene.legs[3].bodyToCoxaJoint,
        scene.legs[3].coxaToFemurJoint,
        scene.legs[3].femurToTibiaJoint,
        scene.legs[4].bodyToCoxaJoint,
        scene.legs[4].coxaToFemurJoint,
        scene.legs[4].femurToTibiaJoint,
        scene.legs[5].bodyToCoxaJoint,
        scene.legs[5].coxaToFemurJoint,
        scene.legs[5].femurToTibiaJoint,
    };
    for (const std::uint32_t joint_id : servo_joint_ids) {
        ServoJoint& joint = world.GetServoJointMutable(joint_id);
        joint.positionGain *= kServoGainScale;
        joint.dampingGain *= kServoGainScale;
        joint.integralGain = kIntegralToPositionRatio * joint.positionGain;
        joint.integralClamp = std::max(joint.integralClamp, 0.75f);
        joint.angleStabilizationScale = kAngleStabilizationScale;
    }
}

void RelaxZeroGravityHexapodServos(World& world, const HexapodSceneObjects& scene) {
    // The built-in hexapod gains are tuned to hold the chassis up against gravity + foot contacts.
    // In zero-G that same stiffness overdrives the free-floating articulated tree, so keep the pose
    // targets but scale the motor terms down for the weightless preview preset.
    constexpr float kZeroGServoScale = 0.25f;
    const std::array<std::uint32_t, 18> servo_joint_ids{
        scene.legs[0].bodyToCoxaJoint,
        scene.legs[0].coxaToFemurJoint,
        scene.legs[0].femurToTibiaJoint,
        scene.legs[1].bodyToCoxaJoint,
        scene.legs[1].coxaToFemurJoint,
        scene.legs[1].femurToTibiaJoint,
        scene.legs[2].bodyToCoxaJoint,
        scene.legs[2].coxaToFemurJoint,
        scene.legs[2].femurToTibiaJoint,
        scene.legs[3].bodyToCoxaJoint,
        scene.legs[3].coxaToFemurJoint,
        scene.legs[3].femurToTibiaJoint,
        scene.legs[4].bodyToCoxaJoint,
        scene.legs[4].coxaToFemurJoint,
        scene.legs[4].femurToTibiaJoint,
        scene.legs[5].bodyToCoxaJoint,
        scene.legs[5].coxaToFemurJoint,
        scene.legs[5].femurToTibiaJoint,
    };
    for (const std::uint32_t joint_id : servo_joint_ids) {
        ServoJoint& joint = world.GetServoJointMutable(joint_id);
        joint.maxServoTorque *= kZeroGServoScale;
        joint.positionGain *= kZeroGServoScale;
        joint.dampingGain *= kZeroGServoScale;
        joint.integralGain *= kZeroGServoScale;
        joint.angleStabilizationScale = 0.0f;
    }
}

int RunClassicDemo(
    SinkKind sink_kind,
    int frame_count,
    bool realtime,
    Vec3 gravity,
    const std::string& udp_host,
    int udp_port,
    DemoRunControl* control) {
    RunLog run_log(BuildDebugLogPath());
    World world(gravity);
#ifndef NDEBUG
    world.SetDebugLogStream(run_log.stream());
    world.SetBlockSolveDebugLogging(false);
#endif

    std::cout << "[hexapod-physics-sim] starting demo scene model=default"
              << " sink=" << SinkName(sink_kind) << " udp=" << udp_host << ":" << udp_port
              << " realtime_playback=" << (realtime ? "true" : "false") << " gravity=(" << gravity.x << "," << gravity.y
              << "," << gravity.z << ")";
    if (run_log.available()) {
        std::cout << " log=" << run_log.path().string();
    } else {
        std::cout << " log=stderr (failed to open log file)";
    }
    std::cout << "\n";

    Body plane;
    plane.shape = ShapeType::Plane;
    plane.planeNormal = {0.0f, 1.0f, 0.0f};
    plane.planeOffset = 0.0f;
    plane.restitution = 0.05f;
    plane.staticFriction = 0.9f;
    plane.dynamicFriction = 0.65f;
    const std::uint32_t plane_id = world.CreateBody(plane);

    Body box_a;
    box_a.shape = ShapeType::Box;
    box_a.position = {0.0f, 0.9f, 0.0f};
    box_a.halfExtents = {0.6f, 0.25f, 0.5f};
    box_a.mass = 3.0f;
    box_a.restitution = 0.05f;
    box_a.staticFriction = 0.8f;
    box_a.dynamicFriction = 0.55f;
    const std::uint32_t box_a_id = world.CreateBody(box_a);

    Body box_b;
    box_b.shape = ShapeType::Box;
    box_b.position = {0.1f, 2.1f, 0.0f};
    box_b.velocity = {0.0f, -0.2f, 0.0f};
    box_b.halfExtents = {0.5f, 0.25f, 0.4f};
    box_b.mass = 2.0f;
    box_b.restitution = 0.03f;
    box_b.staticFriction = 0.78f;
    box_b.dynamicFriction = 0.5f;
    box_b.orientation = Normalize(Quat{0.9914f, 0.0f, 0.0f, 0.1305f});
    const std::uint32_t box_b_id = world.CreateBody(box_b);

    Body sphere;
    sphere.shape = ShapeType::Sphere;
    sphere.position = {1.25f, 2.8f, 0.0f};
    sphere.velocity = {-3.2f, -0.2f, 0.1f};
    sphere.radius = 0.35f;
    sphere.mass = 1.0f;
    sphere.restitution = 0.25f;
    const std::uint32_t sphere_id = world.CreateBody(sphere);

    world.CreateDistanceJoint(
        box_b_id,
        sphere_id,
        world.GetBody(box_b_id).position + Vec3{0.0f, 0.25f, 0.0f},
        world.GetBody(sphere_id).position,
        0.2f,
        0.15f);

    world.CreateHingeJoint(
        box_a_id,
        box_b_id,
        world.GetBody(box_a_id).position + Vec3{0.0f, 0.25f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        true,
        -0.5f,
        0.5f,
        true,
        0.5f,
        0.2f);

    std::unique_ptr<FrameSink> sink = MakeFrameSink(sink_kind, udp_host, udp_port);

    constexpr float dt = 1.0f / 60.0f;
    const DemoSteadyClock::time_point t0 = DemoSteadyClock::now();
    for (int frame = 0; frame < frame_count; ++frame) {
        world.Step(dt, 16);

        sink->begin_frame(frame, static_cast<float>(frame + 1) * dt);
        sink->emit_body(plane_id, world.GetBody(plane_id));
        sink->emit_body(box_a_id, world.GetBody(box_a_id));
        sink->emit_body(box_b_id, world.GetBody(box_b_id));
        sink->emit_body(sphere_id, world.GetBody(sphere_id));
        sink->end_frame();

        CooperativeYield(control);
        if (CooperativeCancelled(control)) {
            std::cout << "[hexapod-physics-sim] demo scene model=default cancelled mid-run\n";
            return 0;
        }

        if (run_log.available()) {
            const Body& a = world.GetBody(box_a_id);
            const Body& b = world.GetBody(box_b_id);
            const Body& s = world.GetBody(sphere_id);
            std::fprintf(run_log.stream(),
                         "frame=%d A=(%.6f, %.6f, %.6f) B=(%.6f, %.6f, %.6f) S=(%.6f, %.6f, %.6f)\n",
                         frame,
                         a.position.x,
                         a.position.y,
                         a.position.z,
                         b.position.x,
                         b.position.y,
                         b.position.z,
                         s.position.x,
                         s.position.y,
                         s.position.z);
        }

        PaceRealtimeOuterFrame(PaceRealtimeEnabled(realtime, control), t0, frame, dt);
    }

    std::cout << "[hexapod-physics-sim] completed demo scene model=default frames=" << frame_count
              << " sink=" << SinkName(sink_kind);
    if (run_log.available()) {
        std::cout << " log=" << run_log.path().string();
    }
    std::cout << "\n";

    return 0;
}

int RunHexapodDemo(
    SinkKind sink_kind,
    int frame_count,
    bool realtime,
    Vec3 gravity,
    const std::string& udp_host,
    int udp_port,
    DemoRunControl* control) {
    RunLog run_log(BuildDebugLogPath());
    World world(gravity);
#ifndef NDEBUG
    world.SetDebugLogStream(run_log.stream());
    world.SetBlockSolveDebugLogging(false);
#endif

    std::cout << "[hexapod-physics-sim] starting demo scene model=hexapod"
              << " sink=" << SinkName(sink_kind) << " udp=" << udp_host << ":" << udp_port
              << " realtime_playback=" << (realtime ? "true" : "false") << " gravity=(" << gravity.x << "," << gravity.y
              << "," << gravity.z << ")";
    if (run_log.available()) {
        std::cout << " log=" << run_log.path().string();
    } else {
        std::cout << " log=stderr (failed to open log file)";
    }
    std::cout << "\n";

    const HexapodSceneObjects scene = BuildHexapodScene(world);
    RelaxBuiltInHexapodServos(world, scene);
    std::unique_ptr<FrameSink> sink = MakeFrameSink(sink_kind, udp_host, udp_port);

    // The built-in hexapod presets are static pose previews. Keep a small number of servo position
    // passes so the leg anchors do not drift under load while leaving per-joint angle snap disabled;
    // the snap term was the part that injected energy and blew the rig apart.
    minphys3d::JointSolverConfig joint_cfg = world.GetJointSolverConfig();
    joint_cfg.servoPositionPasses = 8;
    joint_cfg.hingeAnchorBiasFactor = 0.8f;
    joint_cfg.hingeAnchorDampingFactor = 0.2f;
    world.SetJointSolverConfig(joint_cfg);

    // Zero-G still needs an extra downscale because there is no gravity/contact damping at all.
    if (minphys3d::LengthSquared(gravity) < 1.0e-4f) {
        RelaxZeroGravityHexapodServos(world, scene);
    }

    constexpr float dt = 1.0f / 60.0f;
    constexpr int kPhysicsSubstepsPerFrame = 4;
    constexpr int kSolverIterations = 48;
    const DemoSteadyClock::time_point t0 = DemoSteadyClock::now();
    for (int frame = 0; frame < frame_count; ++frame) {
        for (int substep = 0; substep < kPhysicsSubstepsPerFrame; ++substep) {
            world.Step(dt / static_cast<float>(kPhysicsSubstepsPerFrame), kSolverIterations);
        }

        sink->begin_frame(frame, static_cast<float>(frame + 1) * dt);
        EmitSceneBodies(*sink, world, scene);
        sink->end_frame();

        CooperativeYield(control);
        if (CooperativeCancelled(control)) {
            std::cout << "[hexapod-physics-sim] demo scene model=hexapod cancelled mid-run\n";
            return 0;
        }

        if (run_log.available()) {
            const Body& chassis = world.GetBody(scene.body);
            const std::array<LegContactSummary, 6> contact_summary = SummarizeGroundContacts(world, scene);
            std::fprintf(run_log.stream(),
                         "frame=%d body=(%.6f, %.6f, %.6f) body_vel=(%.6f, %.6f, %.6f) body_rot=(%.6f, %.6f, %.6f, %.6f) body_ang_vel=(%.6f, %.6f, %.6f)\n",
                         frame,
                         chassis.position.x,
                         chassis.position.y,
                         chassis.position.z,
                         chassis.velocity.x,
                         chassis.velocity.y,
                         chassis.velocity.z,
                         chassis.orientation.w,
                         chassis.orientation.x,
                         chassis.orientation.y,
                         chassis.orientation.z,
                         chassis.angularVelocity.x,
                         chassis.angularVelocity.y,
                         chassis.angularVelocity.z);
            for (std::size_t leg_index = 0; leg_index < scene.legs.size(); ++leg_index) {
                const LegLinkIds& leg = scene.legs[leg_index];
                const float coxa_angle = world.GetServoJointAngle(leg.bodyToCoxaJoint);
                const float femur_angle = world.GetServoJointAngle(leg.coxaToFemurJoint);
                const float tibia_angle = world.GetServoJointAngle(leg.femurToTibiaJoint);
                const ServoJoint& coxa_joint = world.GetServoJoint(leg.bodyToCoxaJoint);
                const ServoJoint& femur_joint = world.GetServoJoint(leg.coxaToFemurJoint);
                const ServoJoint& tibia_joint = world.GetServoJoint(leg.femurToTibiaJoint);
                const Body& tibia = world.GetBody(leg.tibia);
                const Vec3 foot = CompoundChildWorldPosition(tibia, 1);
                const LegContactSummary& contacts = contact_summary[leg_index];
                std::fprintf(run_log.stream(),
                             "frame=%d leg=%zu coxa_angle=%.6f coxa_error=%.6f femur_angle=%.6f femur_error=%.6f tibia_angle=%.6f tibia_error=%.6f coxa_manifolds=%d coxa_points=%d coxa_impulse=%.6f coxa_pen=%.6f femur_manifolds=%d femur_points=%d femur_impulse=%.6f femur_pen=%.6f tibia_manifolds=%d tibia_points=%d tibia_impulse=%.6f tibia_pen=%.6f foot_y=%.6f\n",
                             frame,
                             leg_index,
                             coxa_angle,
                             coxa_angle - coxa_joint.targetAngle,
                             femur_angle,
                             femur_angle - femur_joint.targetAngle,
                             tibia_angle,
                             tibia_angle - tibia_joint.targetAngle,
                             contacts.coxa.manifolds,
                             contacts.coxa.points,
                             contacts.coxa.totalNormalImpulse,
                             contacts.coxa.maxPenetration,
                             contacts.femur.manifolds,
                             contacts.femur.points,
                             contacts.femur.totalNormalImpulse,
                             contacts.femur.maxPenetration,
                             contacts.tibia.manifolds,
                             contacts.tibia.points,
                             contacts.tibia.totalNormalImpulse,
                             contacts.tibia.maxPenetration,
                             foot.y);
            }
        }

        PaceRealtimeOuterFrame(PaceRealtimeEnabled(realtime, control), t0, frame, dt);
    }

    std::cout << "[hexapod-physics-sim] completed demo scene model=hexapod frames=" << frame_count
              << " sink=" << SinkName(sink_kind);
    if (run_log.available()) {
        std::cout << " log=" << run_log.path().string();
    }
    std::cout << "\n";

    return 0;
}

const Vec3 kDemoEarthGravity{0.0f, -9.81f, 0.0f};

int RunModelDemo(
    SinkKind sink_kind,
    SceneModel model,
    int frame_count,
    bool realtime,
    Vec3 gravity,
    const std::string& udp_host,
    int udp_port,
    DemoRunControl* control) {
    if (model == SceneModel::Hexapod) {
        return RunHexapodDemo(sink_kind, frame_count, realtime, gravity, udp_host, udp_port, control);
    }
    return RunClassicDemo(sink_kind, frame_count, realtime, gravity, udp_host, udp_port, control);
}

} // namespace

int RunDefaultScene(SinkKind sink_kind, SceneModel model) {
    return RunModelDemo(sink_kind, model, 120, false, kDemoEarthGravity, "127.0.0.1", 9870, nullptr);
}

int RunRealTimeDefaultScene(SinkKind sink_kind, SceneModel model) {
    return RunModelDemo(sink_kind, model, 1200, true, kDemoEarthGravity, "127.0.0.1", 9870, nullptr);
}

int RunPhysicsDemo(
    SinkKind sink_kind,
    SceneModel model,
    int frame_count,
    bool realtime_playback,
    Vec3 gravity,
    const std::string& udp_host,
    int udp_port,
    DemoRunControl* run_control) {
    return RunModelDemo(sink_kind, model, frame_count, realtime_playback, gravity, udp_host, udp_port, run_control);
}

} // namespace minphys3d::demo
