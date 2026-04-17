// Integration: fork hexapod-physics-sim --serve and exchange Config + Step over UDP.
// argv[1] must be the path to the hexapod-physics-sim executable (CTest passes $<TARGET_FILE:...>).
// argv[2], when provided, is a scene file to append in serve mode and should yield obstacle footprints.

#include "physics_sim_protocol.hpp"

#include <chrono>
#include <csignal>
#include <cstdlib>
#include <cstring>
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

    int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        std::cerr << "socket failed\n";
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
    if (expect_obstacles && peek_rsp.obstacle_count == 0) {
        std::cerr << "peek expected obstacle footprints from appended scene\n";
        ::close(fd);
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return 12;
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
        return 13;
    }

    alignas(physics_sim::StateResponse) char rsp_buf[sizeof(physics_sim::StateResponse)]{};
    if (!recvAll(fd, rsp_buf, physics_sim::kStateResponseBytes)) {
        std::cerr << "StateResponse recv failed\n";
        ::close(fd);
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return 14;
    }
    physics_sim::StateResponse rsp{};
    if (!physics_sim::tryDecodeStateResponse(rsp_buf, physics_sim::kStateResponseBytes, rsp)) {
        std::cerr << "StateResponse invalid\n";
        ::close(fd);
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return 14;
    }
    if (rsp.sequence_id != 42) {
        std::cerr << "sequence mismatch\n";
        ::close(fd);
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return 15;
    }
    if (expect_obstacles && rsp.obstacle_count == 0) {
        std::cerr << "step expected obstacle footprints from appended scene\n";
        ::close(fd);
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return 16;
    }

    ::close(fd);
    ::kill(pid, SIGTERM);
    int st = 0;
    ::waitpid(pid, &st, 0);
    std::cout << "test_serve_ipc ok body_y=" << rsp.body_position[1] << "\n";
    return 0;
#endif
}
