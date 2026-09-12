#include "hardware/physics_sim_bridge.hpp"
#include "math_types.hpp"
#include "physics_sim_protocol.hpp"
#include "types.hpp"

#include <atomic>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <iostream>
#include <thread>

#if defined(__linux__) || defined(__APPLE__)
#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#else
#error "test_physics_sim_bridge_frame_conversion requires POSIX sockets"
#endif

namespace {

bool expect(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

bool nearlyEqual(double lhs, double rhs, double eps = 1e-6) {
    return std::abs(lhs - rhs) <= eps;
}

constexpr float kHalfSqrt2 = 0.70710678118f;

class UdpPhysicsSimStub {
public:
    UdpPhysicsSimStub() {
        sock_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (sock_ < 0) {
            std::perror("socket");
            ok_.store(false);
            return;
        }

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
        addr.sin_port = htons(0);
        if (::bind(sock_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
            std::perror("bind");
            ok_.store(false);
            return;
        }

        socklen_t addr_len = sizeof(addr);
        if (::getsockname(sock_, reinterpret_cast<sockaddr*>(&addr), &addr_len) != 0) {
            std::perror("getsockname");
            ok_.store(false);
            return;
        }
        port_ = ntohs(addr.sin_port);

        timeval tv{};
        tv.tv_sec = 2;
        tv.tv_usec = 0;
        (void)::setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    }

    ~UdpPhysicsSimStub() {
        if (thread_.joinable()) {
            thread_.join();
        }
        if (sock_ >= 0) {
            ::close(sock_);
        }
    }

    bool ok() const { return ok_.load(); }
    std::uint16_t port() const { return port_; }

    void setSolverStatus(physics_sim::SolverStatus status) {
        solver_status_.store(status);
    }

    void start() {
        thread_ = std::thread([this]() { run(); });
    }

private:
    void run() {
        while (ok_.load()) {
            std::array<std::byte, 4096> buf{};
            sockaddr_in peer{};
            socklen_t peer_len = sizeof(peer);
            const ssize_t n = ::recvfrom(
                sock_,
                reinterpret_cast<char*>(buf.data()),
                buf.size(),
                0,
                reinterpret_cast<sockaddr*>(&peer),
                &peer_len);
            if (n < 0) {
                ok_.store(false);
                return;
            }

            const auto* bytes = reinterpret_cast<const std::byte*>(buf.data());
            const std::uint8_t mtype = static_cast<std::uint8_t>(*bytes);

            if (mtype == static_cast<std::uint8_t>(physics_sim::MessageType::ConfigCommand)) {
                physics_sim::ConfigCommand cmd{};
                if (!physics_sim::tryDecodeConfigCommand(bytes, static_cast<std::size_t>(n), cmd)) {
                    ok_.store(false);
                    return;
                }

                physics_sim::ConfigAck ack{};
                ack.message_type = static_cast<std::uint8_t>(physics_sim::MessageType::ConfigAck);
                ack.body_count = 1;
                ack.joint_count = 18;
                if (::sendto(sock_, &ack, physics_sim::kConfigAckBytes, 0,
                             reinterpret_cast<sockaddr*>(&peer), peer_len) !=
                    static_cast<ssize_t>(physics_sim::kConfigAckBytes)) {
                    ok_.store(false);
                    return;
                }
                continue;
            }

            if (mtype == static_cast<std::uint8_t>(physics_sim::MessageType::StepCommand)) {
                physics_sim::StepCommand step{};
                if (!physics_sim::tryDecodeStepCommand(bytes, static_cast<std::size_t>(n), step)) {
                    ok_.store(false);
                    return;
                }

                physics_sim::StateResponse rsp{};
                rsp.message_type = static_cast<std::uint8_t>(physics_sim::MessageType::StateResponse);
                rsp.sequence_id = step.sequence_id;
                rsp.body_position = {0.03125f, 0.151162f, -0.0625f};
                // Yaw the body -90 deg in server space (sim +Y axis) so the bridge must rotate
                // world-frame chassis velocities back into body-frame coordinates.
                rsp.body_orientation = {kHalfSqrt2, 0.0f, kHalfSqrt2, 0.0f};
                // Sim +X maps to server world +Y. With the -90 deg yaw above, that should become
                // body-frame -X after the bridge applies R^T to the world velocity.
                rsp.body_linear_velocity = {0.4f, 0.0f, 0.0f};
                rsp.body_angular_velocity = {0.0f, 0.2f, 0.0f};
                rsp.joint_angles.fill(0.0f);
                rsp.joint_velocities.fill(0.0f);
                rsp.solver_status = solver_status_.load();
                rsp.solver_iterations = 17;
                rsp.solver_primal_residual = 1.0e-7f;
                rsp.solver_dual_residual = 2.0e-7f;
                rsp.solver_complementarity_residual = 3.0e-9f;
                rsp.solver_ncp_dual_residual = 4.0e-8f;
                rsp.solver_ncp_complementarity_residual = 5.0e-9f;
                rsp.solver_cone_residual = 6.0e-10f;
                rsp.solver_peak_normal_impulse = 0.011f;
                rsp.solver_peak_friction_impulse = 0.012f;
                rsp.solver_peak_structural_impulse = 0.013f;
                rsp.solver_peak_actuator_impulse = 0.014f;
                rsp.solver_peak_servo_torque_utilization = 0.75f;
                rsp.solver_preintegration_linear_speed = 0.21f;
                rsp.solver_preintegration_angular_speed = 0.31f;
                rsp.solver_mechanical_energy_delta = -0.41f;
                rsp.solver_actuator_work = 0.51f;
                rsp.solver_admm_rho = 4.2f;
                rsp.solver_delassus_condition_estimate = 23.0f;
                rsp.solver_contact_manifold_count = 6;
                rsp.solver_contact_constraint_count = 6;
                rsp.solver_warm_start_reset_count = 7;
                rsp.solver_retry_count = 8;
                rsp.solver_rollback_count = 9;
                rsp.solver_held_state_count = 10;
                rsp.solver_unsupported_island_count = 11;
                rsp.solver_worst_contact_id = 0x123456789abcdef0ULL;
                rsp.solver_contact_set_signature = 0xfedcba9876543210ULL;
                // Non-uniform pattern so tests prove `StateResponse::foot_contacts` maps to `RobotState`.
                rsp.foot_contacts[0] = 1;
                rsp.foot_contacts[1] = 0;
                rsp.foot_contacts[2] = 1;
                rsp.foot_contacts[3] = 0;
                rsp.foot_contacts[4] = 1;
                rsp.foot_contacts[5] = 0;
                for (auto& normal : rsp.foot_contact_normals) {
                    normal = {0.0f, 1.0f, 0.0f};
                }

                if (::sendto(sock_, &rsp, physics_sim::kStateResponseBytes, 0,
                             reinterpret_cast<sockaddr*>(&peer), peer_len) !=
                    static_cast<ssize_t>(physics_sim::kStateResponseBytes)) {
                    ok_.store(false);
                    return;
                }
                continue;
            }

            if (mtype == static_cast<std::uint8_t>(physics_sim::MessageType::StateCorrection)) {
                physics_sim::StateCorrection correction{};
                if (!physics_sim::tryDecodeStateCorrection(bytes, static_cast<std::size_t>(n), correction)) {
                    ok_.store(false);
                    return;
                }
                continue;
            }

            ok_.store(false);
            return;
        }
    }

    int sock_{-1};
    std::uint16_t port_{0};
    std::atomic<bool> ok_{true};
    std::atomic<physics_sim::SolverStatus> solver_status_{physics_sim::SolverStatus::Healthy};
    std::thread thread_{};
};

} // namespace

int main() {
    UdpPhysicsSimStub stub{};
    if (!expect(stub.ok(), "stub socket should initialize")) {
        return EXIT_FAILURE;
    }

    stub.start();

    PhysicsSimBridge bridge("127.0.0.1", stub.port(), 1000, 8, nullptr);
    if (!expect(bridge.init(), "bridge init should succeed against the stub sim")) {
        return EXIT_FAILURE;
    }

    JointTargets cmd{};
    for (int leg = 0; leg < kNumLegs; ++leg) {
        cmd.leg_states[leg].joint_state[COXA].pos_rad = AngleRad{0.0};
        cmd.leg_states[leg].joint_state[FEMUR].pos_rad = AngleRad{0.0};
        cmd.leg_states[leg].joint_state[TIBIA].pos_rad = AngleRad{0.0};
    }
    if (!expect(bridge.write(cmd), "bridge write should accept a neutral target")) {
        return EXIT_FAILURE;
    }

    RobotState out{};
    if (!expect(bridge.read(out), "bridge read should succeed against the stub sim")) {
        return EXIT_FAILURE;
    }
    const auto healthy_solver = bridge.latestSolverTelemetry();
    if (!expect(healthy_solver.has_value(), "healthy response should publish solver telemetry")
        || !expect(healthy_solver->status == physics_sim::SolverStatus::Healthy,
                   "healthy response should preserve solver status")
        || !expect(healthy_solver->iterations == 17,
                   "solver iteration telemetry should cross the bridge")
        || !expect(nearlyEqual(healthy_solver->ncp_dual_residual, 4.0e-8, 1.0e-12),
                   "physical NCP residual telemetry should cross the bridge")
        || !expect(nearlyEqual(healthy_solver->peak_servo_torque_utilization, 0.75),
                   "actuator utilization telemetry should cross the bridge")
        || !expect(nearlyEqual(healthy_solver->preintegration_angular_speed, 0.31),
                   "pre-integration speed telemetry should cross the bridge")
        || !expect(healthy_solver->contact_constraint_count == 6,
                   "contact count telemetry should cross the bridge")
        || !expect(healthy_solver->rollback_count == 9,
                   "rollback telemetry should cross the bridge")
        || !expect(healthy_solver->worst_contact_id == 0x123456789abcdef0ULL,
                   "worst-contact telemetry should cross the bridge")
        || !expect(healthy_solver->contact_set_signature == 0xfedcba9876543210ULL,
                   "contact-set signature telemetry should cross the bridge")) {
        return EXIT_FAILURE;
    }

    constexpr std::array<bool, kNumLegs> kExpectedFootContacts{
        true, false, true, false, true, false};
    for (int leg = 0; leg < kNumLegs; ++leg) {
        if (!expect(out.foot_contacts[static_cast<std::size_t>(leg)] ==
                         kExpectedFootContacts[static_cast<std::size_t>(leg)],
                    "foot_contacts bools should come from physics StateResponse")) {
            return EXIT_FAILURE;
        }
    }

    if (!expect(nearlyEqual(out.body_twist_state.twist_pos_rad.x, 0.0),
                "sim yaw pose should map to near-zero roll") ||
        !expect(nearlyEqual(out.body_twist_state.twist_pos_rad.y, 0.0),
                "sim yaw pose should map to near-zero pitch") ||
        !expect(nearlyEqual(out.body_twist_state.twist_pos_rad.z, -1.57079632679, 1e-5),
                "sim +Y rotation should map to -90deg server yaw") ||
        !expect(nearlyEqual(out.body_twist_state.body_trans_m.x, 0.0625, 1e-6),
                "sim -z translation should map to server +x") ||
        !expect(nearlyEqual(out.body_twist_state.body_trans_m.y, 0.03125, 1e-6),
                "sim x translation should map to server +y") ||
        !expect(nearlyEqual(out.body_twist_state.body_trans_m.z, 0.151162, 1e-6),
                "sim y translation should map to server z") ||
        !expect(nearlyEqual(out.body_twist_state.body_trans_mps.x, -0.4, 1e-5),
                "world velocity aligned with body forward should become body-frame -x") ||
        !expect(nearlyEqual(out.body_twist_state.body_trans_mps.y, 0.0, 1e-5),
                "body-frame velocity should cancel world-frame yaw offset on y") ||
        !expect(nearlyEqual(out.body_twist_state.body_trans_mps.z, 0.0, 1e-5),
                "body-frame velocity should preserve zero vertical component") ||
        !expect(nearlyEqual(out.body_twist_state.twist_vel_radps.z, 0.2, 1e-5),
                "world yaw rate should become body-frame yaw rate")) {
        return EXIT_FAILURE;
    }

    if (!expect(out.has_imu && out.imu.valid, "physics bridge should populate IMU from StateResponse") ||
        !expect(nearlyEqual(vecNorm(out.imu.accel_mps2), 9.80665, 1e-3),
                "IMU specific-force magnitude should be ~1g for upright stub pose")) {
        return EXIT_FAILURE;
    }

    if (!expect(!out.has_matrix_lidar,
                "stub StateResponse should leave matrix_lidar disabled (matrix_lidar_valid=0)")) {
        return EXIT_FAILURE;
    }

    stub.setSolverStatus(physics_sim::SolverStatus::HeldLastGood);
    if (!expect(bridge.write(cmd), "bridge should continue sending after a held solver sample") ||
        !expect(!bridge.read(out), "held solver samples must not be published as valid state") ||
        !expect(bridge.last_bridge_result().has_value() &&
                    bridge.last_bridge_result()->error == BridgeError::Unsupported,
                "held solver sample should expose an unsupported bridge result")) {
        return EXIT_FAILURE;
    }
    const auto held_solver = bridge.latestSolverTelemetry();
    if (!expect(held_solver.has_value(), "held response should retain solver telemetry")
        || !expect(held_solver->status == physics_sim::SolverStatus::HeldLastGood,
                   "held response should preserve solver status before rejection")) {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
