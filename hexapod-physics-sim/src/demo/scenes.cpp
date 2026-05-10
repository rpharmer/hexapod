#include "demo/scenes.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <system_error>
#include <thread>
#include <utility>
#include <vector>

#include "demo/frame_sink.hpp"
#include "demo/process_resource_diagnostics.hpp"
#include "minphys3d/core/body.hpp"
#include "minphys3d/core/world.hpp"
#include "minphys3d/demo/hexapod_scene.hpp"
#include "minphys3d/demo/hexapod_stability.hpp"
#include "hexapod_dynamics_constants.hpp"
#include "physics_sim_protocol.hpp"

namespace minphys3d::demo {
namespace {

constexpr Real kPi = 3.14159265358979323846;

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

struct HexapodLegSpec {
    Vec3 mountOffsetBody{};
    Real mountAngleDegrees = 0.0;
};

const std::array<HexapodLegSpec, 6> kHexapodLegSpecs{{
    {{0.063, -0.007, -0.0835}, 143.0},
    {{-0.063, -0.007, -0.0835}, 217.0},
    {{0.0815, -0.007, 0.0}, 90.0},
    {{-0.0815, -0.007, 0.0}, 270.0},
    {{0.063, -0.007, 0.0835}, 37.0},
    {{-0.063, -0.007, 0.0835}, 323.0},
}};

constexpr Real kBodyMass = static_cast<float>(hexapod_dynamics::kBodyMassKg);
constexpr Real kCoxaMass = static_cast<float>(hexapod_dynamics::kCoxaMassKg);
constexpr Real kFemurMass = static_cast<float>(hexapod_dynamics::kFemurMassKg);
constexpr Real kTibiaMass = static_cast<float>(hexapod_dynamics::kTibiaMassKg);
constexpr Real kFootMass = static_cast<float>(hexapod_dynamics::kFootMassKg);

constexpr Real kCoxaLength = 0.043;
constexpr Real kFemurLength = 0.060;
constexpr Real kTibiaLength = static_cast<float>(hexapod_dynamics::kTibiaLinkLengthM);
constexpr Real kBodyToBottom = 0.040;
constexpr Real kHipMountOutboard = 0.010;
constexpr Real kCoxaRenderLength = kCoxaLength;
constexpr Real kFemurRenderLength = kFemurLength;
constexpr Real kTibiaRenderLength = kTibiaLength;


// Shared servo profile for every leg joint. All six legs get the same max torque so the
// built-in preset stays symmetric and the solver/constraint behavior is not biased by joint type.
constexpr Real kHexapodServoMaxTorque = static_cast<float>(hexapod_dynamics::kServoMaxTorqueNm);
constexpr Real kHexapodServoPositionGain = static_cast<float>(hexapod_dynamics::kServoOmegaN);
constexpr Real kHexapodServoDampingGain = static_cast<float>(hexapod_dynamics::kServoZeta);
// Joint velocity envelope. The original 8 rad/s value was being pinned every stride during
// fast tripod transitions (max_joint_vel_radps = 7.99954, max_foot_speed_mps ≈ 8 × 0.2 m).
// Raising to 10 rad/s gives the controller's commanded swing speeds genuine headroom and
// removes the silent-saturation symptom. Real digital hobby/metal-gear servos in this size
// class run 8–12 rad/s no-load — 10 keeps the constraint plausible.
constexpr Real kHexapodServoMaxSpeed = 10.0;

Quat QuaternionFromBasis(const Vec3& x_axis, const Vec3& y_axis, const Vec3& z_axis) {
    const Real m00 = x_axis.x;
    const Real m01 = y_axis.x;
    const Real m02 = z_axis.x;
    const Real m10 = x_axis.y;
    const Real m11 = y_axis.y;
    const Real m12 = z_axis.y;
    const Real m20 = x_axis.z;
    const Real m21 = y_axis.z;
    const Real m22 = z_axis.z;

    const Real trace = m00 + m11 + m22;
    if (trace > 0.0) {
        const Real s = std::sqrt(trace + 1.0) * 2.0;
        return Normalize(Quat{
            0.25 * s,
            (m21 - m12) / s,
            (m02 - m20) / s,
            (m10 - m01) / s,
        });
    }
    if (m00 > m11 && m00 > m22) {
        const Real s = std::sqrt(1.0 + m00 - m11 - m22) * 2.0;
        return Normalize(Quat{
            (m21 - m12) / s,
            0.25 * s,
            (m01 + m10) / s,
            (m02 + m20) / s,
        });
    }
    if (m11 > m22) {
        const Real s = std::sqrt(1.0 + m11 - m00 - m22) * 2.0;
        return Normalize(Quat{
            (m02 - m20) / s,
            (m01 + m10) / s,
            0.25 * s,
            (m12 + m21) / s,
        });
    }
    const Real s = std::sqrt(1.0 + m22 - m00 - m11) * 2.0;
    return Normalize(Quat{
        (m10 - m01) / s,
        (m02 + m20) / s,
        (m12 + m21) / s,
        0.25 * s,
    });
}

Quat OrientationFromLongAxisAndHint(const Vec3& long_axis, const Vec3& hint_axis) {
    Vec3 x_axis{};
    if (!TryNormalize(long_axis, x_axis)) {
        return {};
    }
    Vec3 y_axis = hint_axis - Dot(hint_axis, x_axis) * x_axis;
    if (!TryNormalize(y_axis, y_axis)) {
        const Vec3 fallback = (std::abs(x_axis.y) < 0.9) ? Vec3{0.0, 1.0, 0.0} : Vec3{1.0, 0.0, 0.0};
        y_axis = fallback - Dot(fallback, x_axis) * x_axis;
        if (!TryNormalize(y_axis, y_axis)) {
            return {};
        }
    }
    Vec3 z_axis = Normalize(Cross(x_axis, y_axis));
    y_axis = Normalize(Cross(z_axis, x_axis));
    return QuaternionFromBasis(x_axis, y_axis, z_axis);
}

Vec3 LegAxisFromMountAngleDegrees(Real angle_degrees) {
    const Real radians = angle_degrees * kPi / 180.0;
    return Normalize(Vec3{std::sin(radians), 0.0, std::cos(radians)});
}

Vec3 LegPitchDirection(const Vec3& leg_axis, Real angle_radians) {
    return Normalize(leg_axis * std::cos(angle_radians) + Vec3{0.0, 1.0, 0.0} * std::sin(angle_radians));
}

Body MakeBoxLinkBody(const Vec3& position, const Quat& orientation, const Vec3& half_extents, Real mass) {
    Body body;
    body.shape = ShapeType::Box;
    body.position = position;
    body.orientation = orientation;
    body.halfExtents = half_extents;
    body.mass = mass;
    body.restitution = 0.02;
    body.staticFriction = 0.65;
    body.dynamicFriction = 0.45;
    body.linearDamping = 2.0;
    body.angularDamping = 10.0;
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

CompoundChild MakeCompoundSphereChild(const Vec3& local_position, Real radius) {
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

Body MakeCompoundChassisBody(const Vec3& position, Real mass) {
    Body body;
    body.shape = ShapeType::Compound;
    body.position = position;
    body.mass = mass;
    body.restitution = 0.0;
    body.staticFriction = 0.60;
    body.dynamicFriction = 0.45;
    body.linearDamping = 2.0;
    body.angularDamping = 16.0;
    body.compoundChildren = {
        MakeCompoundBoxChild({0.0, -0.024, 0.0}, {0.050, 0.010, 0.052}),
        MakeCompoundBoxChild({0.0, -0.024, 0.070}, {0.050, 0.010, 0.018}),
        MakeCompoundBoxChild({0.0, -0.024, -0.070}, {0.050, 0.010, 0.018}),
        MakeCompoundBoxChild({0.067, -0.024, 0.0}, {0.009, 0.010, 0.048}),
        MakeCompoundBoxChild({-0.067, -0.024, 0.0}, {0.009, 0.010, 0.048}),
        MakeCompoundBoxChild({0.0, -0.010, 0.0}, {0.072, 0.008, 0.030}),
        MakeCompoundBoxChild({0.0, 0.012, 0.0}, {0.038, 0.016, 0.042}),
    };
    return body;
}

} // unnamed namespace (detail helpers)

void SyncBuiltInHexapodServoTargets(World& world, const HexapodSceneObjects& scene) {
    // Built geometry uses neutral femur/tibia angles, but `CreateServoJoint` defaults `targetAngle` to 0.
    // Align targets with the measured hinge angles so PD + integral start from the assembled pose instead
    // of fighting a static offset (which reads as "weak" motors and lets the chassis settle low).
    for (const LegLinkIds& leg : scene.legs) {
        const std::array<std::uint32_t, 3> joint_ids{
            leg.bodyToCoxaJoint,
            leg.coxaToFemurJoint,
            leg.femurToTibiaJoint,
        };
        for (const std::uint32_t joint_id : joint_ids) {
            ServoJoint& joint = world.GetServoJointMutable(joint_id);
            joint.targetAngle = world.GetServoJointAngle(joint_id);
            joint.integralAccum = 0.0;
        }
    }
}

Real ComputeStandingBodyHeight() {
    const Vec3 leg_axis = LegAxisFromMountAngleDegrees(kHexapodLegSpecs.front().mountAngleDegrees);
    const Vec3 femur_direction = LegPitchDirection(leg_axis, physics_sim::kAssemblyFemurPitchRad);
    const Vec3 tibia_direction =
        LegPitchDirection(leg_axis, physics_sim::kAssemblyFemurPitchRad + physics_sim::kAssemblyTibiaPitchRad);
    const Vec3 foot_center_relative =
        kHexapodLegSpecs.front().mountOffsetBody
        + leg_axis * kCoxaLength
        + femur_direction * kFemurLength
        + tibia_direction * kTibiaLength;
    // Extra clearance so the first contact frames do not start with feet intersecting the plane when
    // identical servos ramp holding torque (slightly conservative over analytic foot height).
    constexpr Real kSpawnHeightMargin = 0.002;
    return std::max(kBodyToBottom, physics_sim::kHexapodFootRadiusM - foot_center_relative.y + 0.001) +
           kSpawnHeightMargin;
}

HexapodSceneObjects BuildHexapodScene(World& world) {
    HexapodSceneObjects scene{};

    Body plane;
    plane.shape = ShapeType::Plane;
    plane.planeNormal = {0.0, 1.0, 0.0};
    plane.planeOffset = 0.0;
    plane.restitution = 0.05;
    plane.staticFriction = 0.9;
    plane.dynamicFriction = 0.65;
    plane.collisionMask = ~std::uint32_t(0x0002);
    scene.plane = world.CreateBody(plane);
    scene.body_ids.push_back(scene.plane);

    const Real body_height = ComputeStandingBodyHeight();
    Body chassis = MakeCompoundChassisBody({0.0, body_height, 0.0}, kBodyMass);
    chassis.collisionGroup = 0x0002;
    scene.body = world.CreateBody(chassis);
    scene.body_ids.push_back(scene.body);

    constexpr Real kCoxaHalfLength = 0.5 * kCoxaRenderLength;
    constexpr Real kFemurHalfLength = 0.5 * kFemurRenderLength;
    constexpr Real kTibiaHalfLength = 0.5 * kTibiaRenderLength;

    for (std::size_t leg_index = 0; leg_index < kHexapodLegSpecs.size(); ++leg_index) {
        const HexapodLegSpec& leg_spec = kHexapodLegSpecs[leg_index];
        const Vec3 leg_axis = LegAxisFromMountAngleDegrees(leg_spec.mountAngleDegrees);
        const Vec3 lateral_axis = Normalize(Cross(leg_axis, Vec3{0.0, 1.0, 0.0}));
        const Vec3 hip_anchor = chassis.position + leg_spec.mountOffsetBody + leg_axis * kHipMountOutboard;
        const Vec3 femur_direction = LegPitchDirection(leg_axis, physics_sim::kAssemblyFemurPitchRad);
        const Vec3 tibia_direction =
            LegPitchDirection(leg_axis, physics_sim::kAssemblyFemurPitchRad + physics_sim::kAssemblyTibiaPitchRad);

        const Vec3 coxa_center = hip_anchor + leg_axis * kCoxaHalfLength;
        const Vec3 femur_anchor = hip_anchor + leg_axis * kCoxaLength;
        const Vec3 femur_center = femur_anchor + femur_direction * kFemurHalfLength;
        const Vec3 tibia_anchor = femur_anchor + femur_direction * kFemurLength;
        const Vec3 tibia_center = tibia_anchor + tibia_direction * kTibiaHalfLength;
        const Vec3 tibia_tip = tibia_anchor + tibia_direction * kTibiaLength;
        const Vec3 foot_center = tibia_tip;

        Body coxa = MakeBoxLinkBody(
            coxa_center,
            OrientationFromLongAxisAndHint(leg_axis, {0.0, 1.0, 0.0}),
            {kCoxaHalfLength, 0.009, 0.011},
            kCoxaMass);
        const std::uint32_t coxa_id = world.CreateBody(coxa);

        Body femur = MakeBoxLinkBody(
            femur_center,
            OrientationFromLongAxisAndHint(femur_direction, lateral_axis),
            {kFemurHalfLength, 0.010, 0.012},
            kFemurMass);
        const std::uint32_t femur_id = world.CreateBody(femur);

        const Quat tibia_orientation = OrientationFromLongAxisAndHint(tibia_direction, lateral_axis);
        const Vec3 foot_local_offset = Rotate(Conjugate(tibia_orientation), foot_center - tibia_center);

        Body tibia;
        tibia.shape = ShapeType::Compound;
        tibia.position = tibia_center;
        tibia.orientation = tibia_orientation;
        tibia.mass = kTibiaMass + kFootMass;
        tibia.restitution = 0.0;
        tibia.staticFriction = 0.60;
        tibia.dynamicFriction = 0.45;
        tibia.linearDamping = 2.0;
        tibia.angularDamping = 10.0;
        tibia.compoundChildren = {
            MakeCompoundBoxChild({0.0, 0.0, 0.0}, {kTibiaHalfLength, 0.009, 0.009}),
            MakeCompoundSphereChild(foot_local_offset, physics_sim::kHexapodFootRadiusM),
        };
        const std::uint32_t tibia_id = world.CreateBody(tibia);

        const std::uint32_t body_to_coxa_joint = world.CreateServoJoint(
            scene.body,
            coxa_id,
            hip_anchor,
            {0.0, 1.0, 0.0},
            0.0,
            kHexapodServoMaxTorque,
            kHexapodServoPositionGain,
            kHexapodServoDampingGain,
            0.0,
            0.5,
            0.0,
            1.0,
            kHexapodServoMaxSpeed,
            0.5);

        const std::uint32_t coxa_to_femur_joint = world.CreateServoJoint(
            coxa_id,
            femur_id,
            femur_anchor,
            lateral_axis,
            0.0,
            kHexapodServoMaxTorque,
            kHexapodServoPositionGain,
            kHexapodServoDampingGain,
            0.0,
            0.5,
            0.0,
            1.0,
            kHexapodServoMaxSpeed,
            0.5);

        const std::uint32_t femur_to_tibia_joint = world.CreateServoJoint(
            femur_id,
            tibia_id,
            tibia_anchor,
            lateral_axis,
            0.0,
            kHexapodServoMaxTorque,
            kHexapodServoPositionGain,
            kHexapodServoDampingGain,
            0.0,
            0.5,
            0.0,
            1.0,
            kHexapodServoMaxSpeed,
            0.5);

        // Geometric invariant: the femur body only rotates around the lateral axis, so
        // the lateral axis in world space is identical for coxa_to_femur and femur_to_tibia.
        // Let the tibia joint copy {axisA,t1,t2} from the femur joint's prep instead of
        // recomputing from femur.orientation each substep.
        world.GetServoJointMutable(femur_to_tibia_joint).masterAxisJointIdx = coxa_to_femur_joint;

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

    SyncBuiltInHexapodServoTargets(world, scene);
    return scene;
}

void EmitSceneBodies(FrameSink& sink, const World& world, const HexapodSceneObjects& scene) {
    for (const std::uint32_t body_id : scene.body_ids) {
        sink.emit_body(body_id, world.GetBody(body_id));
    }
}

void RelaxBuiltInHexapodServos(World& world, const HexapodSceneObjects& scene) {
    // Servo gains are now omega_n / zeta (mass-normalized by the soft constraint formulation),
    // so no linear gain scaling is needed. Position-pass stabilization is disabled to avoid
    // double stabilization with the velocity-level soft constraint motor.
    constexpr Real kIntegralToPositionRatio = 0.0;
    constexpr Real kAngleStabilizationScale = 0.0;
    for (const std::uint32_t joint_id : HexapodServoJointIds(scene)) {
        ServoJoint& joint = world.GetServoJointMutable(joint_id);
        joint.integralGain = kIntegralToPositionRatio * joint.positionGain;
        joint.integralClamp = std::max(joint.integralClamp, 0.75);
        joint.angleStabilizationScale = kAngleStabilizationScale;
    }
}

void RelaxZeroGravityHexapodServos(World& world, const HexapodSceneObjects& scene) {
    // The built-in hexapod gains are tuned to hold the chassis up against gravity + foot contacts.
    // In zero-G that same stiffness overdrives the free-floating articulated tree, so keep the pose
    // targets but scale the motor terms down for the weightless preview preset.
    constexpr Real kZeroGServoScale = 0.25;
    for (const std::uint32_t joint_id : HexapodServoJointIds(scene)) {
        ServoJoint& joint = world.GetServoJointMutable(joint_id);
        joint.maxServoTorque *= kZeroGServoScale;
        joint.positionGain *= kZeroGServoScale;
        joint.dampingGain *= kZeroGServoScale;
        joint.integralGain *= kZeroGServoScale;
        joint.angleStabilizationScale = 0.0;
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

    ProcessResourceDiagnosticsState resource_diag{};
    std::FILE* resource_out = run_log.available() ? run_log.stream() : stderr;
    MaybeLogProcessResourceSnapshot(resource_out, resource_diag);

    Body plane;
    plane.shape = ShapeType::Plane;
    plane.planeNormal = {0.0, 1.0, 0.0};
    plane.planeOffset = 0.0;
    plane.restitution = 0.05;
    plane.staticFriction = 0.9;
    plane.dynamicFriction = 0.65;
    const std::uint32_t plane_id = world.CreateBody(plane);

    Body box_a;
    box_a.shape = ShapeType::Box;
    box_a.position = {0.0, 0.9, 0.0};
    box_a.halfExtents = {0.6, 0.25, 0.5};
    box_a.mass = 3.0;
    box_a.restitution = 0.05;
    box_a.staticFriction = 0.8;
    box_a.dynamicFriction = 0.55;
    const std::uint32_t box_a_id = world.CreateBody(box_a);

    Body box_b;
    box_b.shape = ShapeType::Box;
    box_b.position = {0.1, 2.1, 0.0};
    box_b.velocity = {0.0, -0.2, 0.0};
    box_b.halfExtents = {0.5, 0.25, 0.4};
    box_b.mass = 2.0;
    box_b.restitution = 0.03;
    box_b.staticFriction = 0.78;
    box_b.dynamicFriction = 0.5;
    box_b.orientation = Normalize(Quat{0.9914, 0.0, 0.0, 0.1305});
    const std::uint32_t box_b_id = world.CreateBody(box_b);

    Body sphere;
    sphere.shape = ShapeType::Sphere;
    sphere.position = {1.25, 2.8, 0.0};
    sphere.velocity = {-3.2, -0.2, 0.1};
    sphere.radius = 0.35;
    sphere.mass = 1.0;
    sphere.restitution = 0.25;
    const std::uint32_t sphere_id = world.CreateBody(sphere);

    world.CreateDistanceJoint(
        box_b_id,
        sphere_id,
        world.GetBody(box_b_id).position + Vec3{0.0, 0.25, 0.0},
        world.GetBody(sphere_id).position,
        0.2,
        0.15);

    world.CreateHingeJoint(
        box_a_id,
        box_b_id,
        world.GetBody(box_a_id).position + Vec3{0.0, 0.25, 0.0},
        {0.0, 1.0, 0.0},
        true,
        -0.5,
        0.5,
        true,
        0.5,
        0.2);

    std::unique_ptr<FrameSink> sink = MakeFrameSink(sink_kind, udp_host, udp_port);

    constexpr Real dt = 1.0 / 60.0;
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
        MaybeLogProcessResourceSnapshot(resource_out, resource_diag);

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
    DemoRunControl* control,
    int solver_iterations) {
    RunLog run_log(BuildDebugLogPath());
    World world(gravity);
#ifndef NDEBUG
    world.SetDebugLogStream(run_log.stream());
    world.SetBlockSolveDebugLogging(false);
#endif

    std::cout << "[hexapod-physics-sim] starting demo scene model=hexapod"
              << " sink=" << SinkName(sink_kind) << " udp=" << udp_host << ":" << udp_port
              << " solver_iterations=" << std::max(1, solver_iterations)
              << " realtime_playback=" << (realtime ? "true" : "false") << " gravity=(" << gravity.x << "," << gravity.y
              << "," << gravity.z << ")";
    if (run_log.available()) {
        std::cout << " log=" << run_log.path().string();
    } else {
        std::cout << " log=stderr (failed to open log file)";
    }
    std::cout << "\n";

    ProcessResourceDiagnosticsState resource_diag{};
    std::FILE* resource_out = run_log.available() ? run_log.stream() : stderr;
    MaybeLogProcessResourceSnapshot(resource_out, resource_diag);

    const HexapodSceneObjects scene = BuildHexapodScene(world);
    RelaxBuiltInHexapodServos(world, scene);
    ApplyHexapodPoseHoldStabilityTuning(world, scene);
    std::unique_ptr<FrameSink> sink = MakeFrameSink(sink_kind, udp_host, udp_port);

    // Zero-G still needs an extra downscale because there is no gravity/contact damping at all.
    if (minphys3d::LengthSquared(gravity) < 1.0e-4) {
        RelaxZeroGravityHexapodServos(world, scene);
    }

    constexpr Real dt = 1.0 / 60.0;
    const int kPhysicsSubstepsPerFrame = kHexapodPoseHoldBenchmarkSubstepsPerFrame;
    const int effective_solver_iterations = std::max(1, solver_iterations);
    std::array<Real, 18> previous_angles{};
    {
        std::size_t angle_index = 0;
        for (const std::uint32_t joint_id : HexapodServoJointIds(scene)) {
            previous_angles[angle_index++] = world.GetServoJointAngle(joint_id);
        }
    }
    HexapodStandingStats standing_stats{};
    const DemoSteadyClock::time_point t0 = DemoSteadyClock::now();
    for (int frame = 0; frame < frame_count; ++frame) {
        for (int substep = 0; substep < kPhysicsSubstepsPerFrame; ++substep) {
            world.Step(dt / static_cast<float>(kPhysicsSubstepsPerFrame), effective_solver_iterations);
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
            const std::array<HexapodLegContactRollup, 6> contact_summary = SummarizeHexapodGroundContacts(world, scene);
            const Real body_roll_rad = BodyRollRadStability(chassis.orientation);
            const Real body_pitch_rad = BodyPitchRadStability(chassis.orientation);
            int total_contact_manifolds = 0;
            int total_contact_points = 0;
            for (const HexapodLegContactRollup& leg_contacts : contact_summary) {
                total_contact_manifolds += leg_contacts.coxa.manifolds + leg_contacts.femur.manifolds + leg_contacts.tibia.manifolds;
                total_contact_points += leg_contacts.coxa.points + leg_contacts.femur.points + leg_contacts.tibia.points;
            }
            Real max_joint_speed_rad_s = 0.0;
            std::fprintf(run_log.stream(),
                         "frame=%d body=(%.6f, %.6f, %.6f) body_height=%.6f body_roll_deg=%.6f body_pitch_deg=%.6f body_vel=(%.6f, %.6f, %.6f) body_rot=(%.6f, %.6f, %.6f, %.6f) body_ang_vel=(%.6f, %.6f, %.6f) contact_manifolds=%d contact_points=%d\n",
                         frame,
                         chassis.position.x,
                         chassis.position.y,
                         chassis.position.z,
                         chassis.position.y,
                         body_roll_rad * 180.0 / kPi,
                         body_pitch_rad * 180.0 / kPi,
                         chassis.velocity.x,
                         chassis.velocity.y,
                         chassis.velocity.z,
                         chassis.orientation.w,
                         chassis.orientation.x,
                         chassis.orientation.y,
                         chassis.orientation.z,
                         chassis.angularVelocity.x,
                         chassis.angularVelocity.y,
                         chassis.angularVelocity.z,
                         total_contact_manifolds,
                         total_contact_points);
            for (std::size_t leg_index = 0; leg_index < scene.legs.size(); ++leg_index) {
                const LegLinkIds& leg = scene.legs[leg_index];
                const Real coxa_angle = world.GetServoJointAngle(leg.bodyToCoxaJoint);
                const Real femur_angle = world.GetServoJointAngle(leg.coxaToFemurJoint);
                const Real tibia_angle = world.GetServoJointAngle(leg.femurToTibiaJoint);
                const Real coxa_speed = (coxa_angle - previous_angles[leg_index * 3 + 0]) / dt;
                const Real femur_speed = (femur_angle - previous_angles[leg_index * 3 + 1]) / dt;
                const Real tibia_speed = (tibia_angle - previous_angles[leg_index * 3 + 2]) / dt;
                max_joint_speed_rad_s = std::max(max_joint_speed_rad_s,
                                                 std::max(std::abs(coxa_speed), std::max(std::abs(femur_speed), std::abs(tibia_speed))));
                const ServoJoint& coxa_joint = world.GetServoJoint(leg.bodyToCoxaJoint);
                const ServoJoint& femur_joint = world.GetServoJoint(leg.coxaToFemurJoint);
                const ServoJoint& tibia_joint = world.GetServoJoint(leg.femurToTibiaJoint);
                const Body& tibia = world.GetBody(leg.tibia);
                const Vec3 foot = CompoundChildWorldPosition(tibia, 1);
                const HexapodLegContactRollup& contacts = contact_summary[leg_index];
                std::fprintf(run_log.stream(),
                             "frame=%d leg=%zu coxa_angle=%.6f coxa_speed=%.6f coxa_error=%.6f femur_angle=%.6f femur_speed=%.6f femur_error=%.6f tibia_angle=%.6f tibia_speed=%.6f tibia_error=%.6f coxa_manifolds=%d coxa_points=%d coxa_impulse=%.6f coxa_pen=%.6f femur_manifolds=%d femur_points=%d femur_impulse=%.6f femur_pen=%.6f tibia_manifolds=%d tibia_points=%d tibia_impulse=%.6f tibia_pen=%.6f foot_y=%.6f\n",
                             frame,
                             leg_index,
                             coxa_angle,
                             coxa_speed,
                             coxa_angle - coxa_joint.targetAngle,
                             femur_angle,
                             femur_speed,
                             femur_angle - femur_joint.targetAngle,
                             tibia_angle,
                             tibia_speed,
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
                previous_angles[leg_index * 3 + 0] = coxa_angle;
                previous_angles[leg_index * 3 + 1] = femur_angle;
                previous_angles[leg_index * 3 + 2] = tibia_angle;
            }
            UpdateHexapodStandingStats(standing_stats,
                                chassis,
                                body_roll_rad,
                                body_pitch_rad,
                                total_contact_manifolds,
                                total_contact_points,
                                max_joint_speed_rad_s);
        } else {
            const Body& chassis = world.GetBody(scene.body);
            const Real body_roll_rad = BodyRollRadStability(chassis.orientation);
            const Real body_pitch_rad = BodyPitchRadStability(chassis.orientation);
            std::array<HexapodLegContactRollup, 6> contact_summary = SummarizeHexapodGroundContacts(world, scene);
            int total_contact_manifolds = 0;
            int total_contact_points = 0;
            for (const HexapodLegContactRollup& leg_contacts : contact_summary) {
                total_contact_manifolds += leg_contacts.coxa.manifolds + leg_contacts.femur.manifolds + leg_contacts.tibia.manifolds;
                total_contact_points += leg_contacts.coxa.points + leg_contacts.femur.points + leg_contacts.tibia.points;
            }
            Real max_joint_speed_rad_s = 0.0;
            for (std::size_t leg_index = 0; leg_index < scene.legs.size(); ++leg_index) {
                const LegLinkIds& leg = scene.legs[leg_index];
                const Real coxa_angle = world.GetServoJointAngle(leg.bodyToCoxaJoint);
                const Real femur_angle = world.GetServoJointAngle(leg.coxaToFemurJoint);
                const Real tibia_angle = world.GetServoJointAngle(leg.femurToTibiaJoint);
                const Real coxa_speed = (coxa_angle - previous_angles[leg_index * 3 + 0]) / dt;
                const Real femur_speed = (femur_angle - previous_angles[leg_index * 3 + 1]) / dt;
                const Real tibia_speed = (tibia_angle - previous_angles[leg_index * 3 + 2]) / dt;
                max_joint_speed_rad_s = std::max(max_joint_speed_rad_s,
                                                 std::max(std::abs(coxa_speed), std::max(std::abs(femur_speed), std::abs(tibia_speed))));
                previous_angles[leg_index * 3 + 0] = coxa_angle;
                previous_angles[leg_index * 3 + 1] = femur_angle;
                previous_angles[leg_index * 3 + 2] = tibia_angle;
            }
            UpdateHexapodStandingStats(standing_stats,
                                chassis,
                                body_roll_rad,
                                body_pitch_rad,
                                total_contact_manifolds,
                                total_contact_points,
                                max_joint_speed_rad_s);
        }
        MaybeLogProcessResourceSnapshot(resource_out, resource_diag);

        PaceRealtimeOuterFrame(PaceRealtimeEnabled(realtime, control), t0, frame, dt);
    }

    const Body& final_chassis = world.GetBody(scene.body);
    std::cout << "[hexapod-physics-sim] completed demo scene model=hexapod frames=" << frame_count
              << " sink=" << SinkName(sink_kind)
              << " solver_iterations=" << effective_solver_iterations
              << " final_body_height=" << final_chassis.position.y
              << " min_body_height=" << standing_stats.minBodyHeight
              << " max_body_height=" << standing_stats.maxBodyHeight
              << " max_abs_roll_deg=" << standing_stats.maxAbsRollDeg
              << " max_abs_pitch_deg=" << standing_stats.maxAbsPitchDeg
              << " max_joint_speed_rad_s=" << standing_stats.maxJointSpeedRadS
              << " min_contact_manifolds=" << standing_stats.minContactManifolds
              << " min_contact_points=" << standing_stats.minContactPoints;
    if (run_log.available()) {
        std::cout << " log=" << run_log.path().string();
    }
    std::cout << "\n";

    return 0;
}

const Vec3 kDemoEarthGravity{0.0, -9.81, 0.0};

int RunModelDemo(
    SinkKind sink_kind,
    SceneModel model,
    int frame_count,
    bool realtime,
    Vec3 gravity,
    const std::string& udp_host,
    int udp_port,
    DemoRunControl* control,
    int solver_iterations) {
    if (model == SceneModel::Hexapod) {
        return RunHexapodDemo(sink_kind, frame_count, realtime, gravity, udp_host, udp_port, control, solver_iterations);
    }
    return RunClassicDemo(sink_kind, frame_count, realtime, gravity, udp_host, udp_port, control);
}

int RunDefaultScene(SinkKind sink_kind, SceneModel model) {
    return RunModelDemo(
        sink_kind, model, 120, false, kDemoEarthGravity, "127.0.0.1", 9870, nullptr, kHexapodPoseHoldBenchmarkSolverIterations);
}

int RunRealTimeDefaultScene(SinkKind sink_kind, SceneModel model) {
    return RunModelDemo(
        sink_kind, model, 1200, true, kDemoEarthGravity, "127.0.0.1", 9870, nullptr, kHexapodPoseHoldBenchmarkSolverIterations);
}

int RunPhysicsDemo(
    SinkKind sink_kind,
    SceneModel model,
    int frame_count,
    bool realtime_playback,
    Vec3 gravity,
    const std::string& udp_host,
    int udp_port,
    DemoRunControl* run_control,
    int solver_iterations) {
    return RunModelDemo(
        sink_kind, model, frame_count, realtime_playback, gravity, udp_host, udp_port, run_control, solver_iterations);
}

} // namespace minphys3d::demo
