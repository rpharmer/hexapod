#include "hardware/physics_sim_bridge.hpp"
#include "physics_sim_protocol.hpp"

#include <atomic>
#include <array>
#include <chrono>
#include <cerrno>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>

#if defined(__linux__) || defined(__APPLE__)
#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#else
#error "test_physics_sim_bridge_state_correction requires POSIX sockets"
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

    bool receivedCorrection() const { return correction_received_.load(); }
    bool receivedStep() const { return step_received_.load(); }
    std::size_t stepCount() const {
        const std::lock_guard<std::mutex> lock(step_mutex_);
        return step_history_.size();
    }
    physics_sim::StateCorrection lastCorrection() const {
        const std::lock_guard<std::mutex> lock(correction_mutex_);
        return last_correction_;
    }
    physics_sim::StepCommand lastStep() const {
        const std::lock_guard<std::mutex> lock(step_mutex_);
        return last_step_;
    }
    physics_sim::StepCommand stepAt(const std::size_t index) const {
        const std::lock_guard<std::mutex> lock(step_mutex_);
        return step_history_.at(index);
    }

private:
    void run() {
        std::uint8_t handled = 0;
        while (handled < 8 && ok_.load()) {
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
                {
                    const std::lock_guard<std::mutex> lock(step_mutex_);
                    last_step_ = step;
                    step_history_.push_back(step);
                }
                step_received_.store(true);

                physics_sim::StateResponse rsp{};
                rsp.message_type = static_cast<std::uint8_t>(physics_sim::MessageType::StateResponse);
                rsp.sequence_id = step.sequence_id;
                rsp.body_position = {0.0f, 0.0f, 0.0f};
                rsp.body_orientation = {1.0f, 0.0f, 0.0f, 0.0f};
                rsp.body_linear_velocity = {0.0f, 0.0f, 0.0f};
                rsp.body_angular_velocity = {0.0f, 0.0f, 0.0f};
                rsp.joint_angles = step.joint_targets;
                rsp.joint_velocities.fill(0.0f);
                if (::sendto(sock_, &rsp, physics_sim::kStateResponseBytes, 0,
                             reinterpret_cast<sockaddr*>(&peer), peer_len) !=
                    static_cast<ssize_t>(physics_sim::kStateResponseBytes)) {
                    ok_.store(false);
                    return;
                }
                ++handled;
                continue;
            }

            if (mtype == static_cast<std::uint8_t>(physics_sim::MessageType::StateCorrection)) {
                physics_sim::StateCorrection correction{};
                if (!physics_sim::tryDecodeStateCorrection(bytes, static_cast<std::size_t>(n), correction)) {
                    ok_.store(false);
                    return;
                }
                {
                    const std::lock_guard<std::mutex> lock(correction_mutex_);
                    last_correction_ = correction;
                }
                correction_received_.store(true);
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
    std::atomic<bool> correction_received_{false};
    std::atomic<bool> step_received_{false};
    mutable std::mutex correction_mutex_{};
    mutable std::mutex step_mutex_{};
    physics_sim::StateCorrection last_correction_{};
    physics_sim::StepCommand last_step_{};
    std::vector<physics_sim::StepCommand> step_history_{};
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

    physics_sim::StateCorrection correction{};
    correction.message_type = static_cast<std::uint8_t>(physics_sim::MessageType::StateCorrection);
    correction.sequence_id = 77;
    correction.timestamp_us = 4242;
    correction.flags = physics_sim::kStateCorrectionPoseValid |
                       physics_sim::kStateCorrectionTwistValid |
                       physics_sim::kStateCorrectionContactValid |
                       physics_sim::kStateCorrectionTerrainValid |
                       physics_sim::kStateCorrectionHardReset;
    correction.correction_strength = 1.0f;
    correction.body_position = {0.25f, 0.50f, -0.125f};
    correction.body_orientation = {0.9238795f, 0.0f, 0.3826834f, 0.0f};
    correction.body_linear_velocity = {0.1f, -0.2f, 0.3f};
    correction.body_angular_velocity = {-0.4f, 0.5f, -0.6f};
    for (std::size_t i = 0; i < correction.foot_contact_phase.size(); ++i) {
        correction.foot_contact_phase[i] =
            static_cast<std::uint8_t>(physics_sim::ContactPhase::ConfirmedStance);
        correction.foot_contact_confidence[i] = 0.9f;
        correction.foot_ground_height_m[i] = 0.12f + static_cast<float>(i) * 0.01f;
        correction.foot_ground_confidence[i] = 0.8f;
    }
    correction.terrain_normal = {0.0f, 0.0f, 1.0f};
    correction.terrain_height_m = 0.14f;

    if (!expect(bridge.sendStateCorrection(correction), "bridge should send state correction")) {
        return EXIT_FAILURE;
    }

    physics_sim::StateCorrection got{};
    bool saw_expected_correction = false;
    for (int i = 0; i < 200; ++i) {
        if (stub.receivedCorrection()) {
            got = stub.lastCorrection();
            if (got.sequence_id == correction.sequence_id) {
                saw_expected_correction = true;
                break;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds{5});
    }

    if (!expect(saw_expected_correction, "stub should receive the explicit state correction")) {
        return EXIT_FAILURE;
    }
    if (!expect(got.message_type == static_cast<std::uint8_t>(physics_sim::MessageType::StateCorrection),
                "state correction message type should be preserved") ||
        !expect(got.sequence_id == correction.sequence_id, "sequence id should round-trip") ||
        !expect(got.timestamp_us == correction.timestamp_us, "timestamp should round-trip") ||
        !expect((got.flags & physics_sim::kStateCorrectionHardReset) != 0,
                "hard reset flag should be preserved") ||
        !expect((got.flags & physics_sim::kStateCorrectionContactValid) != 0,
                "contact valid flag should be preserved") ||
        !expect(nearlyEqual(got.correction_strength, correction.correction_strength),
                "correction strength should round-trip")) {
        return EXIT_FAILURE;
    }

    if (!expect(nearlyEqual(got.body_position[0], correction.body_position[0]) &&
                    nearlyEqual(got.body_position[1], correction.body_position[1]) &&
                    nearlyEqual(got.body_position[2], correction.body_position[2]),
                "body position should round-trip") ||
        !expect(nearlyEqual(got.body_orientation[0], correction.body_orientation[0]) &&
                    nearlyEqual(got.body_orientation[1], correction.body_orientation[1]) &&
                    nearlyEqual(got.body_orientation[2], correction.body_orientation[2]) &&
                    nearlyEqual(got.body_orientation[3], correction.body_orientation[3]),
                "body orientation should round-trip") ||
        !expect(nearlyEqual(got.body_linear_velocity[0], correction.body_linear_velocity[0]) &&
                    nearlyEqual(got.body_linear_velocity[1], correction.body_linear_velocity[1]) &&
                    nearlyEqual(got.body_linear_velocity[2], correction.body_linear_velocity[2]),
                "body linear velocity should round-trip") ||
        !expect(nearlyEqual(got.body_angular_velocity[0], correction.body_angular_velocity[0]) &&
                    nearlyEqual(got.body_angular_velocity[1], correction.body_angular_velocity[1]) &&
                    nearlyEqual(got.body_angular_velocity[2], correction.body_angular_velocity[2]),
                "body angular velocity should round-trip")) {
        return EXIT_FAILURE;
    }

    RobotState out{};
    if (!expect(bridge.read(out), "bridge should establish a feedback sample before forwarding testing")) {
        return EXIT_FAILURE;
    }
    const std::size_t baseline_step_count = stub.stepCount();

    JointTargets aggressive{};
    aggressive.leg_states[0].joint_state[COXA].pos_rad = AngleRad{1.0};
    if (!expect(bridge.write(aggressive), "bridge should accept aggressive joint targets")) {
        return EXIT_FAILURE;
    }

    if (!expect(bridge.read(out), "bridge read should succeed after joint target write")) {
        return EXIT_FAILURE;
    }

    for (int i = 0; i < 200; ++i) {
        if (stub.stepCount() >= baseline_step_count + 1) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds{5});
    }

    if (!expect(stub.stepCount() >= baseline_step_count + 1, "stub should receive a post-write step command")) {
        return EXIT_FAILURE;
    }

    const physics_sim::StepCommand step = stub.stepAt(stub.stepCount() - 1);
    const physics_sim::StepCommand prior_step = stub.stepAt(baseline_step_count - 1);
    const double first_joint_delta =
        std::abs(static_cast<double>(step.joint_targets[0]) - static_cast<double>(prior_step.joint_targets[0]));
    if (!expect(first_joint_delta > 0.0, "forwarded step command should still move the joint")) {
        return EXIT_FAILURE;
    }
    if (first_joint_delta <= 0.1) {
        std::cerr << "Observed first joint delta: " << first_joint_delta << '\n';
    }
    if (!expect(first_joint_delta > 0.1,
                "physics sim bridge should forward the commanded joint target without extra rate limiting")) {
        return EXIT_FAILURE;
    }

    JointTargets zero_targets{};
    if (!expect(bridge.write(zero_targets), "bridge should accept an all-zero target as a standing hold")) {
        return EXIT_FAILURE;
    }
    if (!expect(bridge.read(out), "bridge read should succeed after zero target write")) {
        return EXIT_FAILURE;
    }
    for (int i = 0; i < 200; ++i) {
        if (stub.stepCount() >= baseline_step_count + 2) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds{5});
    }
    if (!expect(stub.stepCount() >= baseline_step_count + 2,
                "stub should receive a step command after zero target write")) {
        return EXIT_FAILURE;
    }
    const physics_sim::StepCommand hold_step = stub.stepAt(stub.stepCount() - 1);
    const double hold_joint_delta =
        std::abs(static_cast<double>(hold_step.joint_targets[0]) - static_cast<double>(prior_step.joint_targets[0]));
    if (!expect(hold_joint_delta < 0.05,
                "all-zero bridge writes should resolve to a standing hold target")) {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
