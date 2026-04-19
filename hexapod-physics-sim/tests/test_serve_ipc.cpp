// Integration: fork hexapod-physics-sim --serve and exchange Config + Step over UDP.
// argv[1] must be the path to the hexapod-physics-sim executable (CTest passes $<TARGET_FILE:...>).
// argv[2], when provided, is a scene file to append in serve mode and should yield obstacle footprints.

#include "physics_sim_protocol.hpp"

#include <chrono>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <iostream>
#include <thread>

#if defined(__linux__)
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>
#endif

namespace {

bool sendAll(int fd, const void* data, std::size_t len) {
    const auto* p = reinterpret_cast<const char*>(data);
    std::size_t off = 0;
    while (off < len) {
        const ssize_t n = ::send(fd, p + off, len - off, 0);
        if (n <= 0) {
            return false;
        }
        off += static_cast<std::size_t>(n);
    }
    return true;
}

bool recvAll(int fd, void* data, std::size_t len) {
    auto* p = reinterpret_cast<char*>(data);
    std::size_t off = 0;
    while (off < len) {
        const ssize_t n = ::recv(fd, p + off, len - off, 0);
        if (n <= 0) {
            return false;
        }
        off += static_cast<std::size_t>(n);
    }
    return true;
}

int openUdpSocketWithRetry() {
    for (int attempt = 0; attempt < 5; ++attempt) {
        const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (fd >= 0) {
            return fd;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }
    return -1;
}

bool sendCorrection(int fd, const physics_sim::StateCorrection& correction) {
    return sendAll(fd, &correction, physics_sim::kStateCorrectionBytes);
}

float YawAboutSimUp(const std::array<float, 4>& q) {
    const float siny_cosp = 2.0f * (q[0] * q[2] + q[1] * q[3]);
    const float cosy_cosp = 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]);
    return std::atan2(siny_cosp, cosy_cosp);
}

std::array<float, 4> QuaternionFromYaw(float yaw_rad) {
    const float half = 0.5f * yaw_rad;
    return {std::cos(half), 0.0f, std::sin(half), 0.0f};
}

std::uint16_t MinFiniteLidarMm(const physics_sim::StateResponse& rsp) {
    std::uint16_t best = physics_sim::kMatrixLidarInvalidMm;
    for (std::uint16_t mm : rsp.matrix_lidar_ranges_mm) {
        if (mm == physics_sim::kMatrixLidarInvalidMm) {
            continue;
        }
        if (best == physics_sim::kMatrixLidarInvalidMm || mm < best) {
            best = mm;
        }
    }
    return best;
}

} // namespace

int main(int argc, char** argv) {
#if !defined(__linux__)
    std::cerr << "test_serve_ipc: Linux-only (fork/exec)\n";
    return 0;
#else
    if (argc < 2) {
        std::cerr << "usage: test_serve_ipc PATH_TO_hexapod-physics-sim [SCENE_FILE]\n";
        return 0;
    }

    constexpr int kPort = 20977;
    const char* sim_exe = argv[1];
    const char* scene_file = argc >= 3 ? argv[2] : nullptr;
    const bool expect_obstacles = scene_file != nullptr;

    pid_t pid = ::fork();
    if (pid < 0) {
        std::cerr << "fork failed\n";
        return 2;
    }
    if (pid == 0) {
        const std::string port_str = std::to_string(kPort);
        if (scene_file != nullptr) {
            ::execl(sim_exe,
                    sim_exe,
                    "--serve",
                    "--serve-port",
                    port_str.c_str(),
                    "--scene-file",
                    scene_file,
                    nullptr);
        } else {
            ::execl(sim_exe, sim_exe, "--serve", "--serve-port", port_str.c_str(), nullptr);
        }
        std::perror("execl");
        _exit(127);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    int fd = openUdpSocketWithRetry();
    if (fd < 0) {
        std::cerr << "socket failed\n";
        std::perror("socket");
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return 3;
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<std::uint16_t>(kPort));
    if (::inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr) != 1) {
        ::close(fd);
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return 4;
    }
    if (::connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
        std::cerr << "connect failed\n";
        ::close(fd);
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return 5;
    }

    physics_sim::ConfigCommand cfg{};
    cfg.message_type = static_cast<std::uint8_t>(physics_sim::MessageType::ConfigCommand);
    cfg.gravity = {0.0f, -9.81f, 0.0f};
    cfg.solver_iterations = 16;
    if (!sendAll(fd, &cfg, physics_sim::kConfigCommandBytes)) {
        std::cerr << "send ConfigCommand failed\n";
        ::close(fd);
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return 6;
    }

    alignas(physics_sim::ConfigAck) char ack_buf[sizeof(physics_sim::ConfigAck)]{};
    if (!recvAll(fd, ack_buf, physics_sim::kConfigAckBytes)) {
        std::cerr << "ConfigAck recv failed\n";
        ::close(fd);
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return 7;
    }
    physics_sim::ConfigAck ack{};
    if (!physics_sim::tryDecodeConfigAck(ack_buf, physics_sim::kConfigAckBytes, ack)) {
        std::cerr << "ConfigAck invalid\n";
        ::close(fd);
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return 7;
    }
    if (ack.joint_count < 18) {
        std::cerr << "unexpected joint_count " << ack.joint_count << "\n";
        ::close(fd);
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return 8;
    }

    physics_sim::StepCommand peek{};
    peek.message_type = static_cast<std::uint8_t>(physics_sim::MessageType::StepCommand);
    peek.sequence_id = 0;
    peek.dt_seconds = 1.0f;
    if (!sendAll(fd, &peek, physics_sim::kStepCommandBytes)) {
        std::cerr << "send peek StepCommand failed\n";
        ::close(fd);
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return 9;
    }
    alignas(physics_sim::StateResponse) char peek_buf[sizeof(physics_sim::StateResponse)]{};
    if (!recvAll(fd, peek_buf, physics_sim::kStateResponseBytes)) {
        std::cerr << "peek StateResponse recv failed\n";
        ::close(fd);
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return 10;
    }
    physics_sim::StateResponse peek_rsp{};
    if (!physics_sim::tryDecodeStateResponse(peek_buf, physics_sim::kStateResponseBytes, peek_rsp)) {
        std::cerr << "peek StateResponse invalid\n";
        ::close(fd);
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return 10;
    }
    if (peek_rsp.sequence_id != 0) {
        std::cerr << "peek sequence mismatch\n";
        ::close(fd);
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return 11;
    }
    if (peek_rsp.body_position[1] < 0.05f) {
        std::cerr << "peek expected chassis above ground\n";
        ::close(fd);
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return 12;
    }
    if (peek_rsp.matrix_lidar_valid != 1) {
        std::cerr << "peek expected matrix_lidar_valid\n";
        ::close(fd);
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return 12;
    }
    if (peek_rsp.matrix_lidar_cols != 64 || peek_rsp.matrix_lidar_rows != 8) {
        std::cerr << "peek unexpected matrix lidar dimensions\n";
        ::close(fd);
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return 12;
    }
    {
        int finite_mm = 0;
        for (std::uint16_t mm : peek_rsp.matrix_lidar_ranges_mm) {
            if (mm != 0xFFFF && mm >= 100 && mm <= 5000) {
                ++finite_mm;
            }
        }
        if (finite_mm < 32) {
            std::cerr << "peek expected many finite matrix lidar returns (floor/obstacles)\n";
            ::close(fd);
            ::kill(pid, SIGTERM);
            ::waitpid(pid, nullptr, 0);
            return 12;
        }
    }
    if (expect_obstacles && peek_rsp.obstacle_count == 0) {
        std::cerr << "peek expected obstacle footprints from appended scene\n";
        ::close(fd);
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return 12;
    }

    const std::array<float, 3> base_pos = peek_rsp.body_position;
    const std::array<float, 4> base_ori = peek_rsp.body_orientation;
    const std::uint16_t base_min_lidar = MinFiniteLidarMm(peek_rsp);

    physics_sim::StateCorrection lift{};
    lift.message_type = static_cast<std::uint8_t>(physics_sim::MessageType::StateCorrection);
    lift.sequence_id = 1;
    lift.timestamp_us = 1000;
    lift.flags = physics_sim::kStateCorrectionPoseValid |
                 physics_sim::kStateCorrectionTwistValid |
                 physics_sim::kStateCorrectionHardReset;
    lift.correction_strength = 1.0f;
    lift.body_position = base_pos;
    lift.body_position[1] += 0.25f;
    lift.body_orientation = base_ori;
    lift.body_linear_velocity = {0.0f, 0.0f, 0.0f};
    lift.body_angular_velocity = {0.0f, 0.0f, 0.0f};
    if (!sendCorrection(fd, lift)) {
        std::cerr << "send lift correction failed\n";
        ::close(fd);
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return 13;
    }

    physics_sim::StepCommand step{};
    step.message_type = static_cast<std::uint8_t>(physics_sim::MessageType::StepCommand);
    step.sequence_id = 42;
    step.dt_seconds = 1.0f / 240.0f;
    for (float& j : step.joint_targets) {
        j = 0.0f;
    }
    if (!sendAll(fd, &step, physics_sim::kStepCommandBytes)) {
        std::cerr << "send StepCommand failed\n";
        ::close(fd);
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return 14;
    }

    alignas(physics_sim::StateResponse) char rsp_buf[sizeof(physics_sim::StateResponse)]{};
    if (!recvAll(fd, rsp_buf, physics_sim::kStateResponseBytes)) {
        std::cerr << "StateResponse recv failed\n";
        ::close(fd);
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return 15;
    }
    physics_sim::StateResponse rsp{};
    if (!physics_sim::tryDecodeStateResponse(rsp_buf, physics_sim::kStateResponseBytes, rsp)) {
        std::cerr << "StateResponse invalid\n";
        ::close(fd);
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return 15;
    }
    if (rsp.sequence_id != 42) {
        std::cerr << "sequence mismatch after lift\n";
        ::close(fd);
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return 16;
    }
    if (rsp.body_position[1] < base_pos[1] + 0.20f) {
        std::cerr << "lift expected chassis to move upward\n";
        ::close(fd);
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return 17;
    }
    for (std::uint8_t contact : rsp.foot_contacts) {
        if (contact != 0) {
            std::cerr << "lift expected airborne robot to have no raw foot contacts\n";
            ::close(fd);
            ::kill(pid, SIGTERM);
            ::waitpid(pid, nullptr, 0);
            return 17;
        }
    }

    physics_sim::StateCorrection contact{};
    contact.message_type = static_cast<std::uint8_t>(physics_sim::MessageType::StateCorrection);
    contact.sequence_id = 2;
    contact.timestamp_us = 2000;
    contact.flags = physics_sim::kStateCorrectionContactValid;
    contact.correction_strength = 1.0f;
    for (std::size_t i = 0; i < contact.foot_contact_phase.size(); ++i) {
        contact.foot_contact_phase[i] = static_cast<std::uint8_t>(physics_sim::ContactPhase::ConfirmedStance);
        contact.foot_contact_confidence[i] = 1.0f;
    }
    if (!sendCorrection(fd, contact)) {
        std::cerr << "send contact correction failed\n";
        ::close(fd);
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return 18;
    }

    step.sequence_id = 43;
    if (!sendAll(fd, &step, physics_sim::kStepCommandBytes)) {
        std::cerr << "send contact StepCommand failed\n";
        ::close(fd);
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return 19;
    }
    if (!recvAll(fd, rsp_buf, physics_sim::kStateResponseBytes)) {
        std::cerr << "contact StateResponse recv failed\n";
        ::close(fd);
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return 20;
    }
    if (!physics_sim::tryDecodeStateResponse(rsp_buf, physics_sim::kStateResponseBytes, rsp)) {
        std::cerr << "contact StateResponse invalid\n";
        ::close(fd);
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return 20;
    }
    if (rsp.sequence_id != 43) {
        std::cerr << "sequence mismatch after contact correction\n";
        ::close(fd);
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return 21;
    }
    for (std::uint8_t contact_bit : rsp.foot_contacts) {
        if (contact_bit != 0) {
            std::cerr << "contact correction must not force raw manifold contacts\n";
            ::close(fd);
            ::kill(pid, SIGTERM);
            ::waitpid(pid, nullptr, 0);
            return 21;
        }
    }
    if (rsp.body_position[1] < base_pos[1] + 0.20f) {
        std::cerr << "contact correction should not collapse the airborne chassis\n";
        ::close(fd);
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return 21;
    }

    physics_sim::StateCorrection yaw{};
    yaw.message_type = static_cast<std::uint8_t>(physics_sim::MessageType::StateCorrection);
    yaw.sequence_id = 3;
    yaw.timestamp_us = 3000;
    yaw.flags = physics_sim::kStateCorrectionPoseValid;
    yaw.correction_strength = 0.75f;
    yaw.body_position = rsp.body_position;
    const float current_yaw = YawAboutSimUp(rsp.body_orientation);
    const float target_yaw = current_yaw + 0.60f;
    yaw.body_orientation = QuaternionFromYaw(target_yaw);
    if (!sendCorrection(fd, yaw)) {
        std::cerr << "send yaw correction failed\n";
        ::close(fd);
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return 22;
    }

    step.sequence_id = 44;
    if (!sendAll(fd, &step, physics_sim::kStepCommandBytes)) {
        std::cerr << "send yaw StepCommand failed\n";
        ::close(fd);
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return 23;
    }
    if (!recvAll(fd, rsp_buf, physics_sim::kStateResponseBytes)) {
        std::cerr << "yaw StateResponse recv failed\n";
        ::close(fd);
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return 24;
    }
    if (!physics_sim::tryDecodeStateResponse(rsp_buf, physics_sim::kStateResponseBytes, rsp)) {
        std::cerr << "yaw StateResponse invalid\n";
        ::close(fd);
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return 24;
    }
    const float blended_yaw = YawAboutSimUp(rsp.body_orientation);
    const float yaw_delta = std::atan2(std::sin(blended_yaw - current_yaw), std::cos(blended_yaw - current_yaw));
    if (!(std::abs(yaw_delta) > 0.03f && std::abs(yaw_delta) < 0.60f)) {
        std::cerr << "yaw correction should blend, delta=" << yaw_delta << "\n";
        ::close(fd);
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return 25;
    }

    physics_sim::StateCorrection reset{};
    reset.message_type = static_cast<std::uint8_t>(physics_sim::MessageType::StateCorrection);
    reset.sequence_id = 4;
    reset.timestamp_us = 4000;
    reset.flags = physics_sim::kStateCorrectionPoseValid |
                  physics_sim::kStateCorrectionTwistValid |
                  physics_sim::kStateCorrectionTerrainValid |
                  physics_sim::kStateCorrectionHardReset;
    reset.correction_strength = 1.0f;
    reset.body_position = base_pos;
    reset.body_orientation = base_ori;
    reset.body_linear_velocity = {0.0f, 0.0f, 0.0f};
    reset.body_angular_velocity = {0.0f, 0.0f, 0.0f};
    reset.terrain_height_m = base_pos[1];
    reset.terrain_normal = {0.0f, 1.0f, 0.0f};
    if (!sendCorrection(fd, reset)) {
        std::cerr << "send reset correction failed\n";
        ::close(fd);
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return 26;
    }

    step.sequence_id = 45;
    if (!sendAll(fd, &step, physics_sim::kStepCommandBytes)) {
        std::cerr << "send reset StepCommand failed\n";
        ::close(fd);
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return 27;
    }
    if (!recvAll(fd, rsp_buf, physics_sim::kStateResponseBytes)) {
        std::cerr << "reset StateResponse recv failed\n";
        ::close(fd);
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return 28;
    }
    if (!physics_sim::tryDecodeStateResponse(rsp_buf, physics_sim::kStateResponseBytes, rsp)) {
        std::cerr << "reset StateResponse invalid\n";
        ::close(fd);
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return 28;
    }
    if (rsp.sequence_id != 45) {
        std::cerr << "sequence mismatch after reset\n";
        ::close(fd);
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return 29;
    }
    if (std::abs(rsp.body_position[1] - base_pos[1]) > 0.05f) {
        std::cerr << "reset should restore the chassis height\n";
        ::close(fd);
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return 30;
    }
    if (std::abs(std::atan2(std::sin(YawAboutSimUp(rsp.body_orientation) - YawAboutSimUp(base_ori)),
                            std::cos(YawAboutSimUp(rsp.body_orientation) - YawAboutSimUp(base_ori)))) > 0.10f) {
        std::cerr << "reset should restore the heading\n";
        ::close(fd);
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return 30;
    }
    if (expect_obstacles && rsp.obstacle_count == 0) {
        std::cerr << "reset expected obstacle footprints from appended scene\n";
        ::close(fd);
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return 31;
    }

    physics_sim::StateCorrection terrain{};
    terrain.message_type = static_cast<std::uint8_t>(physics_sim::MessageType::StateCorrection);
    terrain.sequence_id = 0;
    terrain.timestamp_us = 5000;
    terrain.flags = physics_sim::kStateCorrectionTerrainValid;
    terrain.correction_strength = 0.85f;
    terrain.terrain_height_m = base_pos[1] + 0.14f;
    terrain.terrain_normal = {0.0f, 1.0f, 0.0f};
    if (!sendCorrection(fd, terrain)) {
        std::cerr << "send terrain correction failed\n";
        ::close(fd);
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return 32;
    }

    if (!sendAll(fd, &peek, physics_sim::kStepCommandBytes)) {
        std::cerr << "send terrain peek StepCommand failed\n";
        ::close(fd);
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return 33;
    }
    if (!recvAll(fd, peek_buf, physics_sim::kStateResponseBytes)) {
        std::cerr << "terrain peek StateResponse recv failed\n";
        ::close(fd);
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return 34;
    }
    if (!physics_sim::tryDecodeStateResponse(peek_buf, physics_sim::kStateResponseBytes, peek_rsp)) {
        std::cerr << "terrain peek StateResponse invalid\n";
        ::close(fd);
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return 34;
    }
    const std::uint16_t terrain_min_lidar = MinFiniteLidarMm(peek_rsp);
    if (terrain_min_lidar == physics_sim::kMatrixLidarInvalidMm ||
        terrain_min_lidar >= base_min_lidar) {
        std::cerr << "terrain correction should bring some lidar returns closer (base="
                  << base_min_lidar << " terrain=" << terrain_min_lidar << ")\n";
        ::close(fd);
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return 35;
    }

    ::close(fd);
    ::kill(pid, SIGTERM);
    int st = 0;
    ::waitpid(pid, &st, 0);
    std::cout << "test_serve_ipc ok body_y=" << rsp.body_position[1] << "\n";
    return 0;
#endif
}
