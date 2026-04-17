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

    void start() {
        thread_ = std::thread([this]() { run(); });
    }

private:
    void run() {
        std::uint8_t handled = 0;
        while (handled < 3 && ok_.load()) {
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
                ++handled;
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
                rsp.body_position = {0.0f, 0.151162f, 0.0f};
                rsp.body_orientation = {1.0f, 0.0f, 0.0f, 0.0f};
                rsp.body_linear_velocity = {0.0f, 0.0f, 0.0f};
                rsp.body_angular_velocity = {0.0f, 0.0f, 0.0f};
                rsp.joint_angles.fill(0.0f);
                rsp.joint_velocities.fill(0.0f);
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
                ++handled;
                continue;
            }

            ok_.store(false);
            return;
        }
    }

    int sock_{-1};
    std::uint16_t port_{0};
    std::atomic<bool> ok_{true};
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
                "upright sim pose should map to near-zero roll") ||
        !expect(nearlyEqual(out.body_twist_state.twist_pos_rad.y, 0.0),
                "upright sim pose should map to near-zero pitch") ||
        !expect(nearlyEqual(out.body_twist_state.twist_pos_rad.z, 0.0),
                "upright sim pose should map to near-zero yaw") ||
        !expect(nearlyEqual(out.body_twist_state.body_trans_m.x, 0.0),
                "sim x translation should map through unchanged") ||
        !expect(nearlyEqual(out.body_twist_state.body_trans_m.y, 0.0),
                "sim z translation should map through unchanged") ||
        !expect(nearlyEqual(out.body_twist_state.body_trans_m.z, 0.151162, 1e-6),
                "sim y translation should map to server z")) {
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

    return EXIT_SUCCESS;
}
