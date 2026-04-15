#include "physics_sim_bridge.hpp"

#include "geometry_config.hpp"
#include "physics_sim_joint_wire_mapping.hpp"
#include "logger.hpp"
#include "physics_sim_protocol.hpp"
#include "types.hpp"

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

/// Maps sim world (Y-up) vectors into hexapod-server world (Z-up): (x,y,z)_svr = (x_s, z_s, y_s).
/// Sim +Y is up; server +Z is up (so +Y_sim must map to +Z_svr, not -Z).
Vec3 simVecToServer(float x, float y, float z) {
    return Vec3{static_cast<double>(x), static_cast<double>(z), static_cast<double>(y)};
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

// Fixed world-frame rotation R_ws * v_sim = v_server (Y-up sim -> Z-up server, same as simVecToServer).
Mat3d frameSimToServer() {
    Mat3d r{};
    r.m[0][0] = 1.0;
    r.m[0][1] = 0.0;
    r.m[0][2] = 0.0;
    r.m[1][0] = 0.0;
    r.m[1][1] = 0.0;
    r.m[1][2] = 1.0;
    r.m[2][0] = 0.0;
    r.m[2][1] = 1.0;
    r.m[2][2] = 0.0;
    return r;
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

    initialized_ = true;
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
        return true;
    }
    pending_targets_ = in;
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

    const Mat3d R_ws = frameSimToServer();
    const Mat3d R_sim = quatWxyzToMat3(
        rsp.body_orientation[0],
        rsp.body_orientation[1],
        rsp.body_orientation[2],
        rsp.body_orientation[3]);
    // Convert the sim-world rotation into server-world coordinates by changing basis on
    // both the source and destination side. A one-sided multiply leaves the upright sim
    // pose looking like a tipped server pose.
    const Mat3d R_srv = matMul(R_ws, matMul(R_sim, R_ws));
    out.body_twist_state.twist_pos_rad = matToRollPitchYawZyx(R_srv);

    const Vec3 w_srv = simVecToServer(
        rsp.body_angular_velocity[0], rsp.body_angular_velocity[1], rsp.body_angular_velocity[2]);
    out.body_twist_state.twist_vel_radps =
        AngularVelocityRadPerSec3{w_srv.x, w_srv.y, w_srv.z};

    const Vec3 v_srv = simVecToServer(
        rsp.body_linear_velocity[0], rsp.body_linear_velocity[1], rsp.body_linear_velocity[2]);
    out.body_twist_state.body_trans_mps = VelocityMps3{v_srv.x, v_srv.y, v_srv.z};

    const Vec3 p_srv = simVecToServer(rsp.body_position[0], rsp.body_position[1], rsp.body_position[2]);
    out.body_twist_state.body_trans_m = PositionM3{p_srv.x, p_srv.y, p_srv.z};

    last_result_.error = BridgeError::None;
    return true;
}

std::optional<BridgeCommandResultMetadata> PhysicsSimBridge::last_bridge_result() const {
    return last_result_;
}
