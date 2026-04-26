#include "physics_sim_bridge.hpp"

#include "geometry_config.hpp"
#include "body_controller.hpp"
#include "physics_sim_joint_wire_mapping.hpp"
#include "leg_ik.hpp"
#include "motion_intent_utils.hpp"
#include "logger.hpp"
#include "physics_sim_protocol.hpp"
#include "imu_physics.hpp"
#include "matrix_lidar_physics.hpp"
#include "types.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <string>

#if defined(__linux__) || defined(__APPLE__)
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#else
#error "PhysicsSimBridge requires POSIX sockets"
#endif

namespace {

constexpr double kHalfPi = 1.57079632679489661923;
struct HexapodLegSpec {
    Vec3 mountOffsetBody{};
    float mountAngleDegrees = 0.0f;
};

constexpr std::array<HexapodLegSpec, 6> kHexapodLegSpecs{{
    {{0.063f, -0.007f, -0.0835f}, 143.0f},
    {{-0.063f, -0.007f, -0.0835f}, 217.0f},
    {{0.0815f, -0.007f, 0.0f}, 90.0f},
    {{-0.0815f, -0.007f, 0.0f}, 270.0f},
    {{0.063f, -0.007f, 0.0835f}, 37.0f},
    {{-0.063f, -0.007f, 0.0835f}, 323.0f},
}};

/// Maps sim world/body vectors into hexapod-server coordinates so the spawned chassis forward
/// direction (sim local/world -Z) becomes server +X, left becomes +Y, and up stays +Z:
/// (x,y,z)_svr = (-z_s, x_s, y_s).
Vec3 simVecToServer(float x, float y, float z) {
    return Vec3{-static_cast<double>(z), static_cast<double>(x), static_cast<double>(y)};
}

Vec3 normalizeVec3(const Vec3& value) {
    const double norm = vecNorm(value);
    if (norm <= 1e-9) {
        return Vec3{};
    }
    return Vec3{value.x / norm, value.y / norm, value.z / norm};
}

Vec3 LegAxisFromMountAngleDegrees(float angle_degrees) {
    const float radians = angle_degrees * static_cast<float>(3.14159265358979323846) / 180.0f;
    return normalizeVec3(Vec3{std::sin(radians), 0.0f, std::cos(radians)});
}

Vec3 LegPitchDirection(const Vec3& leg_axis, float angle_radians) {
    return normalizeVec3(leg_axis * std::cos(angle_radians) + Vec3{0.0f, 1.0f, 0.0f} * std::sin(angle_radians));
}

double computeStandingBodyHeightM() {
    const Vec3 leg_axis = LegAxisFromMountAngleDegrees(kHexapodLegSpecs.front().mountAngleDegrees);
    const Vec3 femur_direction = LegPitchDirection(leg_axis, physics_sim::kAssemblyFemurPitchRad);
    const Vec3 tibia_direction =
        LegPitchDirection(leg_axis, physics_sim::kAssemblyFemurPitchRad + physics_sim::kAssemblyTibiaPitchRad);
    const Vec3 foot_center_relative =
        kHexapodLegSpecs.front().mountOffsetBody
        + leg_axis * 0.043f
        + femur_direction * 0.060f
        + tibia_direction * 0.104f
        + Vec3{0.0f, -0.018f, 0.0f};
    constexpr double kSpawnHeightMargin = 0.002;
    return std::max(0.04, 0.018 - static_cast<double>(foot_center_relative.y) + 0.001) + kSpawnHeightMargin;
}

Vec3 computeFootInBodyFrame(const LegState& leg_state, const LegGeometry& leg_geometry) {
    const double q1 = leg_state.joint_state[COXA].pos_rad.value;
    const double q2 = leg_state.joint_state[FEMUR].pos_rad.value;
    const double q3 = leg_state.joint_state[TIBIA].pos_rad.value;

    const double rho = leg_geometry.femurLength.value * std::cos(q2) +
                       leg_geometry.tibiaLength.value * std::cos(q2 + q3);
    const double z_plane = leg_geometry.femurLength.value * std::sin(q2) +
                           leg_geometry.tibiaLength.value * std::sin(q2 + q3);
    const double x_leg = leg_geometry.coxaLength.value + rho;
    const double y_leg = -std::sin(q1) * z_plane;
    const double z_leg = std::cos(q1) * z_plane;

    const Vec3 foot_leg_local{x_leg, y_leg, z_leg};
    const double c = std::cos(leg_geometry.mountAngle.value);
    const double s = std::sin(leg_geometry.mountAngle.value);
    const Vec3 foot_body_relative{
        c * foot_leg_local.x - s * foot_leg_local.y,
        s * foot_leg_local.x + c * foot_leg_local.y,
        foot_leg_local.z
    };
    return leg_geometry.bodyCoxaOffset + foot_body_relative;
}

struct Mat3d {
    double m[3][3]{};
};

Mat3d quatWxyzToMat3(double w, double x, double y, double z) {
    const double n = w * w + x * x + y * y + z * z;
    const double s = n > 0.0 ? 2.0 / n : 0.0;
    const double wx = s * w * x;
    const double wy = s * w * y;
    const double wz = s * w * z;
    const double xx = s * x * x;
    const double xy = s * x * y;
    const double xz = s * x * z;
    const double yy = s * y * y;
    const double yz = s * y * z;
    const double zz = s * z * z;
    Mat3d r{};
    r.m[0][0] = 1.0 - (yy + zz);
    r.m[0][1] = xy - wz;
    r.m[0][2] = xz + wy;
    r.m[1][0] = xy + wz;
    r.m[1][1] = 1.0 - (xx + zz);
    r.m[1][2] = yz - wx;
    r.m[2][0] = xz - wy;
    r.m[2][1] = yz + wx;
    r.m[2][2] = 1.0 - (xx + yy);
    return r;
}

Mat3d matMul(const Mat3d& a, const Mat3d& b) {
    Mat3d c{};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            c.m[i][j] = a.m[i][0] * b.m[0][j] + a.m[i][1] * b.m[1][j] + a.m[i][2] * b.m[2][j];
        }
    }
    return c;
}

// Fixed sim->server change-of-basis matrix matching `simVecToServer`.
Mat3d frameSimToServer() {
    Mat3d r{};
    r.m[0][0] = 0.0;
    r.m[0][1] = 0.0;
    r.m[0][2] = -1.0;
    r.m[1][0] = 1.0;
    r.m[1][1] = 0.0;
    r.m[1][2] = 0.0;
    r.m[2][0] = 0.0;
    r.m[2][1] = 1.0;
    r.m[2][2] = 0.0;
    return r;
}

Mat3d matTranspose(const Mat3d& a) {
    Mat3d t{};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            t.m[i][j] = a.m[j][i];
        }
    }
    return t;
}

Vec3 matMulVec(const Mat3d& a, const Vec3& v) {
    return Vec3{
        a.m[0][0] * v.x + a.m[0][1] * v.y + a.m[0][2] * v.z,
        a.m[1][0] * v.x + a.m[1][1] * v.y + a.m[1][2] * v.z,
        a.m[2][0] * v.x + a.m[2][1] * v.y + a.m[2][2] * v.z,
    };
}

EulerAnglesRad3 matToRollPitchYawZyx(const Mat3d& r) {
    const double sy = std::sqrt(r.m[0][0] * r.m[0][0] + r.m[1][0] * r.m[1][0]);
    const bool singular = sy < 1e-8;
    double roll{};
    double pitch{};
    double yaw{};
    if (!singular) {
        roll = std::atan2(r.m[2][1], r.m[2][2]);
        pitch = std::atan2(-r.m[2][0], sy);
        yaw = std::atan2(r.m[1][0], r.m[0][0]);
    } else {
        roll = std::atan2(-r.m[1][2], r.m[1][1]);
        pitch = std::atan2(-r.m[2][0], sy);
        yaw = 0.0;
    }
    return EulerAnglesRad3{roll, pitch, yaw};
}

PhysicsSimObstacleFootprint obstacleFootprintToServer(const physics_sim::ObstacleFootprint& in) {
    PhysicsSimObstacleFootprint out{};
    out.center_x_m = -static_cast<double>(in.center_z);
    out.center_y_m = static_cast<double>(in.center_x);
    out.half_extent_x_m = static_cast<double>(in.half_extent_x);
    out.half_extent_y_m = static_cast<double>(in.half_extent_z);
    out.yaw_rad = static_cast<double>(in.yaw_rad) + kHalfPi;
    return out;
}

JointTargets buildStandingServoTargets() {
    BodyController body_controller{};
    LegIK leg_ik{};
    RobotState est{};
    SafetyState safety{};
    GaitState gait{};
    MotionIntent intent = makeMotionIntent(RobotMode::STAND, GaitType::TRIPOD, computeStandingBodyHeightM());
    const BodyTwist cmd_twist{};
    const LegTargets stance_targets =
        body_controller.update(est, intent, gait, safety, cmd_twist, nullptr);
    return leg_ik.solve(est, stance_targets, safety);
}

physics_sim::StateCorrection buildStandingResetCorrection() {
    physics_sim::StateCorrection correction{};
    correction.message_type = static_cast<std::uint8_t>(physics_sim::MessageType::StateCorrection);
    correction.sequence_id = 0;
    correction.timestamp_us = now_us().value;
    correction.flags = physics_sim::kStateCorrectionPoseValid |
                       physics_sim::kStateCorrectionTwistValid |
                       physics_sim::kStateCorrectionHardReset;
    correction.correction_strength = 1.0f;

    correction.body_position = {0.0f, static_cast<float>(computeStandingBodyHeightM()), 0.0f};
    correction.body_orientation = {1.0f, 0.0f, 0.0f, 0.0f};
    correction.body_linear_velocity = {0.0f, 0.0f, 0.0f};
    correction.body_angular_velocity = {0.0f, 0.0f, 0.0f};
    return correction;
}

double maxAbsJointVelocityRadps(const physics_sim::StateResponse& rsp) {
    double max_vel = 0.0;
    for (const float joint_vel : rsp.joint_velocities) {
        max_vel = std::max(max_vel, std::abs(static_cast<double>(joint_vel)));
    }
    return max_vel;
}

} // namespace

PhysicsSimBridge::PhysicsSimBridge(std::string host,
                                   int port,
                                   int bus_loop_period_us,
                                   int physics_solver_iterations,
                                   std::shared_ptr<logging::AsyncLogger> logger)
    : host_(std::move(host)),
      port_(port),
      bus_loop_period_us_(bus_loop_period_us),
      physics_solver_iterations_(physics_solver_iterations > 0 ? physics_solver_iterations : 8),
      logger_(std::move(logger)) {}

PhysicsSimBridge::~PhysicsSimBridge() {
    if (sock_ >= 0) {
        ::close(sock_);
        sock_ = -1;
    }
}

void PhysicsSimBridge::setLastError(BridgeError err, BridgeFailurePhase phase) {
    last_result_.error = err;
    last_result_.phase = phase;
}

bool PhysicsSimBridge::recvWithTimeout(void* buf, std::size_t len, int timeout_ms) {
    struct pollfd pfd {};
    pfd.fd = sock_;
    pfd.events = POLLIN;
    const int pr = ::poll(&pfd, 1, timeout_ms);
    if (pr <= 0) {
        setLastError(BridgeError::Timeout, BridgeFailurePhase::CommandResponse);
        return false;
    }
    const ssize_t n = ::recv(sock_, reinterpret_cast<char*>(buf), len, 0);
    if (n != static_cast<ssize_t>(len)) {
        setLastError(BridgeError::TransportFailure, BridgeFailurePhase::CommandResponse);
        return false;
    }
    return true;
}

bool PhysicsSimBridge::init() {
    if (initialized_) {
        return true;
    }
    if (logger_) {
        LOG_INFO(logger_, "PhysicsSimBridge init host=", host_, " port=", port_,
                 " solver_iterations=", physics_solver_iterations_);
    }
    sock_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_ < 0) {
        setLastError(BridgeError::TransportFailure, BridgeFailurePhase::Initialization);
        return false;
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<std::uint16_t>(port_));
    if (::inet_pton(AF_INET, host_.c_str(), &addr.sin_addr) != 1) {
        setLastError(BridgeError::TransportFailure, BridgeFailurePhase::Initialization);
        ::close(sock_);
        sock_ = -1;
        return false;
    }

    if (::connect(sock_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
        setLastError(BridgeError::TransportFailure, BridgeFailurePhase::Initialization);
        ::close(sock_);
        sock_ = -1;
        return false;
    }

    physics_sim::ConfigCommand cfg{};
    cfg.gravity = {0.0f, -9.81f, 0.0f};
    cfg.solver_iterations = physics_solver_iterations_;

    if (::send(sock_, &cfg, physics_sim::kConfigCommandBytes, 0) !=
        static_cast<ssize_t>(physics_sim::kConfigCommandBytes)) {
        setLastError(BridgeError::TransportFailure, BridgeFailurePhase::Initialization);
        ::close(sock_);
        sock_ = -1;
        return false;
    }
    if (logger_) {
        LOG_DEBUG(logger_, "PhysicsSimBridge sent ConfigCommand");
    }

    alignas(physics_sim::ConfigAck) std::byte ack_buf[sizeof(physics_sim::ConfigAck)]{};
    if (!recvWithTimeout(ack_buf, physics_sim::kConfigAckBytes, 500)) {
        if (logger_) {
            LOG_ERROR(logger_, "PhysicsSim: ConfigAck timeout or short read");
        }
        ::close(sock_);
        sock_ = -1;
        return false;
    }
    physics_sim::ConfigAck ack{};
    if (!physics_sim::tryDecodeConfigAck(ack_buf, physics_sim::kConfigAckBytes, ack)) {
        setLastError(BridgeError::ProtocolFailure, BridgeFailurePhase::CommandDecode);
        ::close(sock_);
        sock_ = -1;
        return false;
    }
    if (logger_) {
        LOG_DEBUG(logger_, "PhysicsSimBridge received ConfigAck body_count=", ack.body_count,
                  " joint_count=", ack.joint_count);
    }
    if (ack.joint_count < 18) {
        setLastError(BridgeError::ProtocolFailure, BridgeFailurePhase::Initialization);
        if (logger_) {
            LOG_ERROR(logger_, "PhysicsSim: unexpected joint_count=", ack.joint_count);
        }
        ::close(sock_);
        sock_ = -1;
        return false;
    }

    physics_sim::StepCommand peek{};
    peek.sequence_id = 0;
    peek.dt_seconds = 1.0e-3f;

    if (::send(sock_, &peek, physics_sim::kStepCommandBytes, 0) !=
        static_cast<ssize_t>(physics_sim::kStepCommandBytes)) {
        setLastError(BridgeError::TransportFailure, BridgeFailurePhase::Initialization);
        ::close(sock_);
        sock_ = -1;
        return false;
    }

    alignas(physics_sim::StateResponse) std::byte peek_buf[sizeof(physics_sim::StateResponse)]{};
    if (!recvWithTimeout(peek_buf, physics_sim::kStateResponseBytes, 500)) {
        if (logger_) {
            LOG_ERROR(logger_, "PhysicsSim: peek StateResponse timeout");
        }
        setLastError(BridgeError::Timeout, BridgeFailurePhase::Initialization);
        ::close(sock_);
        sock_ = -1;
        return false;
    }
    physics_sim::StateResponse peek_rsp{};
    if (!physics_sim::tryDecodeStateResponse(peek_buf, physics_sim::kStateResponseBytes, peek_rsp)) {
        setLastError(BridgeError::ProtocolFailure, BridgeFailurePhase::CommandDecode);
        ::close(sock_);
        sock_ = -1;
        return false;
    }
    if (peek_rsp.sequence_id != 0) {
        setLastError(BridgeError::ProtocolFailure, BridgeFailurePhase::Initialization);
        if (logger_) {
            LOG_ERROR(logger_, "PhysicsSim: peek sequence mismatch");
        }
        ::close(sock_);
        sock_ = -1;
        return false;
    }

    const HexapodGeometry& geometry = geometry_config::activeHexapodGeometry();
    for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
        const ServoCalibration& cal = geometry.legGeometry[leg].servo;
        const int base = static_cast<int>(leg) * 3;
        const float sim_c = peek_rsp.joint_angles[static_cast<std::size_t>(base + 0)];
        const float sim_f = peek_rsp.joint_angles[static_cast<std::size_t>(base + 1)];
        const float sim_t = peek_rsp.joint_angles[static_cast<std::size_t>(base + 2)];
        pending_targets_.leg_states[leg] =
            physics_sim_joint_wire_mapping::servoLegFromSimWireAngles(cal, static_cast<int>(leg), sim_c, sim_f, sim_t);
    }

    pending_targets_ = buildStandingServoTargets();
    last_motion_trace_log_us_ = TimePointUs{};
    last_motion_trace_sample_us_ = TimePointUs{};
    last_motion_trace_foot_positions_body_.fill(Vec3{});
    have_motion_trace_foot_positions_ = false;
    initialized_ = true;
    if (!sendStateCorrection(buildStandingResetCorrection())) {
        if (logger_) {
            LOG_WARN(logger_, "PhysicsSimBridge failed to send standing reset correction");
        }
        ::close(sock_);
        sock_ = -1;
        initialized_ = false;
        return false;
    }
    if (logger_) {
        LOG_INFO(logger_, "PhysicsSimBridge seeded standing joint targets and reset sim pose after connect");
    }

    last_result_.error = BridgeError::None;
    return true;
}

bool PhysicsSimBridge::write(const JointTargets& in) {
    if (!initialized_) {
        return false;
    }
    // RobotRuntime seeds joint_targets to zero; freshness rejects also write empty targets.
    // Do not overwrite primed sim pose until control sends a non-trivial command.
    bool all_zero = true;
    for (const LegState& leg : in.leg_states) {
        for (const JointState& js : leg.joint_state) {
            if (js.pos_rad.value != 0.0 || js.vel_radps.value != 0.0) {
                all_zero = false;
                break;
            }
        }
        if (!all_zero) {
            break;
        }
    }
    if (all_zero) {
        pending_targets_ = buildStandingServoTargets();
        return true;
    }
    pending_targets_ = in;
    return true;
}

bool PhysicsSimBridge::sendStateCorrection(const physics_sim::StateCorrection& correction) {
    if (!initialized_) {
        return false;
    }

    const ssize_t sent = ::send(sock_, &correction, physics_sim::kStateCorrectionBytes, 0);
    if (sent != static_cast<ssize_t>(physics_sim::kStateCorrectionBytes)) {
        setLastError(BridgeError::TransportFailure, BridgeFailurePhase::CommandTransport);
        if (logger_) {
            LOG_WARN(logger_, "PhysicsSimBridge failed to send StateCorrection (sent=", sent, ")");
        }
        return false;
    }

    if (logger_) {
        LOG_DEBUG(logger_,
                  "PhysicsSimBridge sent StateCorrection seq=",
                  correction.sequence_id,
                  " flags=",
                  static_cast<unsigned>(correction.flags),
                  " strength=",
                  correction.correction_strength);
    }
    return true;
}

bool PhysicsSimBridge::read(RobotState& out) {
    if (!initialized_) {
        return false;
    }

    const HexapodGeometry& geometry = geometry_config::activeHexapodGeometry();

    physics_sim::StepCommand step{};
    step.dt_seconds = static_cast<float>(bus_loop_period_us_) * 1.0e-6f;
    static std::uint32_t seq = 1;
    step.sequence_id = seq++;

    for (int leg = 0; leg < kNumLegs; ++leg) {
        const ServoCalibration& cal = geometry.legGeometry[leg].servo;
        const int base = leg * 3;
        physics_sim_joint_wire_mapping::simWireTargetsFromServoLeg(
            cal,
            leg,
            pending_targets_.leg_states[static_cast<std::size_t>(leg)],
            step.joint_targets[static_cast<std::size_t>(base + 0)],
            step.joint_targets[static_cast<std::size_t>(base + 1)],
            step.joint_targets[static_cast<std::size_t>(base + 2)]);
    }

    if (::send(sock_, &step, physics_sim::kStepCommandBytes, 0) !=
        static_cast<ssize_t>(physics_sim::kStepCommandBytes)) {
        setLastError(BridgeError::TransportFailure, BridgeFailurePhase::CommandTransport);
        return false;
    }

    alignas(physics_sim::StateResponse) std::byte rsp_buf[sizeof(physics_sim::StateResponse)]{};
    if (!recvWithTimeout(rsp_buf, physics_sim::kStateResponseBytes, 500)) {
        if (logger_) {
            LOG_ERROR(logger_, "PhysicsSim: StateResponse timeout");
        }
        return false;
    }
    physics_sim::StateResponse rsp{};
    if (!physics_sim::tryDecodeStateResponse(rsp_buf, physics_sim::kStateResponseBytes, rsp)) {
        setLastError(BridgeError::ProtocolFailure, BridgeFailurePhase::CommandDecode);
        return false;
    }
    if (rsp.sequence_id != step.sequence_id) {
        setLastError(BridgeError::ProtocolFailure, BridgeFailurePhase::CommandDecode);
        return false;
    }

    {
        std::vector<PhysicsSimObstacleFootprint> footprints{};
        const std::size_t obstacle_count =
            std::min<std::size_t>(rsp.obstacle_count, rsp.obstacles.size());
        footprints.reserve(obstacle_count);
        for (std::size_t i = 0; i < obstacle_count; ++i) {
            footprints.push_back(obstacleFootprintToServer(rsp.obstacles[i]));
        }
        const std::lock_guard<std::mutex> lock(obstacle_mutex_);
        latest_obstacle_footprints_ = std::move(footprints);
    }

    out = RobotState{};
    out.timestamp_us = now_us();
    out.sample_id = static_cast<std::uint64_t>(step.sequence_id);
    out.bus_ok = true;
    out.valid = true;
    out.has_valid_flag = true;
    out.has_body_twist_state = true;
    out.has_power_state = true;
    out.voltage = 12.0f;
    out.current = 1.0f;

    for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
        const ServoCalibration& cal = geometry.legGeometry[leg].servo;
        const int base = static_cast<int>(leg) * 3;
        const float sim_c = rsp.joint_angles[static_cast<std::size_t>(base + 0)];
        const float sim_f = rsp.joint_angles[static_cast<std::size_t>(base + 1)];
        const float sim_t = rsp.joint_angles[static_cast<std::size_t>(base + 2)];

        out.leg_states[leg] =
            physics_sim_joint_wire_mapping::servoLegFromSimWireAngles(cal, leg, sim_c, sim_f, sim_t);
        const double v_c = static_cast<double>(rsp.joint_velocities[static_cast<std::size_t>(base + 0)]);
        const double v_f = static_cast<double>(rsp.joint_velocities[static_cast<std::size_t>(base + 1)]);
        const double v_t = static_cast<double>(rsp.joint_velocities[static_cast<std::size_t>(base + 2)]);
        // Servo positions are affine in mechanical joint angles:
        // servo = sign * joint + offset, and sim wire carries the mechanical joint directly.
        out.leg_states[leg].joint_state[COXA].vel_radps =
            AngularRateRadPerSec{cal.coxaSign * v_c};
        out.leg_states[leg].joint_state[FEMUR].vel_radps =
            AngularRateRadPerSec{cal.femurSign * v_f};
        out.leg_states[leg].joint_state[TIBIA].vel_radps =
            AngularRateRadPerSec{cal.tibiaSign * v_t};
    }

    for (std::size_t i = 0; i < kNumLegs; ++i) {
        out.foot_contacts[i] = rsp.foot_contacts[i] != 0;
    }

    imuSampleFromPhysicsStateResponse(rsp, out.timestamp_us, out.imu);
    out.has_imu = out.imu.valid;

    matrixLidarFromPhysicsStateResponse(rsp, out.timestamp_us, out);

    const Mat3d R_ws = frameSimToServer();
    const Mat3d R_sim = quatWxyzToMat3(
        rsp.body_orientation[0],
        rsp.body_orientation[1],
        rsp.body_orientation[2],
        rsp.body_orientation[3]);
    // Convert the sim-world rotation into server-world coordinates with a proper
    // change-of-basis: R_srv = C * R_sim * C^-1, where C maps sim vectors into
    // the server frame used by locomotion/navigation (+X forward, +Y left, +Z up).
    const Mat3d R_srv = matMul(R_ws, matMul(R_sim, matTranspose(R_ws)));
    out.body_twist_state.twist_pos_rad = matToRollPitchYawZyx(R_srv);

    const Vec3 w_srv_world = simVecToServer(
        rsp.body_angular_velocity[0], rsp.body_angular_velocity[1], rsp.body_angular_velocity[2]);
    const Vec3 w_srv_body = matMulVec(matTranspose(R_srv), w_srv_world);
    out.body_twist_state.twist_vel_radps =
        AngularVelocityRadPerSec3{w_srv_body.x, w_srv_body.y, w_srv_body.z};

    const Vec3 v_srv_world = simVecToServer(
        rsp.body_linear_velocity[0], rsp.body_linear_velocity[1], rsp.body_linear_velocity[2]);
    // Physics sim exposes world-frame chassis velocities. Convert them into the server body frame
    // so downstream foot planning can safely blend them with body-frame command twists.
    const Vec3 v_srv_body = matMulVec(matTranspose(R_srv), v_srv_world);
    out.body_twist_state.body_trans_mps = VelocityMps3{v_srv_body.x, v_srv_body.y, v_srv_body.z};

    const Vec3 p_srv = simVecToServer(rsp.body_position[0], rsp.body_position[1], rsp.body_position[2]);
    out.body_twist_state.body_trans_m = PositionM3{p_srv.x, p_srv.y, p_srv.z};

    const TimePointUs trace_now = out.timestamp_us;
    const bool have_trace_dt = !last_motion_trace_sample_us_.isZero() &&
                               trace_now.value > last_motion_trace_sample_us_.value;
    const double trace_dt_s = have_trace_dt
                                  ? static_cast<double>(trace_now.value - last_motion_trace_sample_us_.value) * 1.0e-6
                                  : 0.0;
    const double max_joint_vel_radps = maxAbsJointVelocityRadps(rsp);

    double max_foot_speed_mps = 0.0;
    std::array<Vec3, kNumLegs> current_foot_positions_body{};
    for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
        current_foot_positions_body[leg] =
            computeFootInBodyFrame(out.leg_states[leg], geometry.legGeometry[leg]);
        if (have_motion_trace_foot_positions_ && trace_dt_s > 0.0) {
            const Vec3 delta = current_foot_positions_body[leg] - last_motion_trace_foot_positions_body_[leg];
            max_foot_speed_mps = std::max(max_foot_speed_mps, vecNorm(delta) / trace_dt_s);
        }
    }

    const bool motion_trace_ready = max_joint_vel_radps > 0.02 || max_foot_speed_mps > 0.002;
    const bool should_log_motion_trace =
        motion_trace_ready &&
        (last_motion_trace_log_us_.isZero() ||
         trace_now.value - last_motion_trace_log_us_.value >= 1000000ULL);
    if (should_log_motion_trace && logger_) {
        LOG_INFO(logger_,
                 "PhysicsSimBridge motion trace max_joint_vel_radps=",
                 max_joint_vel_radps,
                 " max_foot_speed_mps=",
                 max_foot_speed_mps,
                 " sample_dt_us=",
                 have_trace_dt ? (trace_now.value - last_motion_trace_sample_us_.value) : 0ULL,
                 " log_age_us=",
                 last_motion_trace_log_us_.isZero() ? 0ULL : (trace_now.value - last_motion_trace_log_us_.value));
        last_motion_trace_log_us_ = trace_now;
    }
    last_motion_trace_foot_positions_body_ = current_foot_positions_body;
    last_motion_trace_sample_us_ = trace_now;
    have_motion_trace_foot_positions_ = true;

    last_result_.error = BridgeError::None;
    return true;
}

std::optional<BridgeCommandResultMetadata> PhysicsSimBridge::last_bridge_result() const {
    return last_result_;
}

std::vector<PhysicsSimObstacleFootprint> PhysicsSimBridge::latestObstacleFootprints() const {
    const std::lock_guard<std::mutex> lock(obstacle_mutex_);
    return latest_obstacle_footprints_;
}
