// Integration: fork hexapod-physics-sim --serve --sink udp and verify IPC replies still work.

#include "physics_sim_protocol.hpp"

#include <chrono>
#include <cmath>
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

} // namespace

int main(int argc, char** argv) {
#if !defined(__linux__)
    std::cerr << "test_serve_ipc_preview: Linux-only (fork/exec)\n";
    return 0;
#else
    if (argc < 2) {
        std::cerr << "usage: test_serve_ipc_preview PATH_TO_hexapod-physics-sim\n";
        return 0;
    }

    constexpr int kServePort = 20978;
    constexpr int kPreviewPort = 20979;
    const char* sim_exe = argv[1];

    pid_t pid = ::fork();
    if (pid < 0) {
        std::cerr << "fork failed\n";
        return 2;
    }
    if (pid == 0) {
        const std::string serve_port_str = std::to_string(kServePort);
        const std::string preview_port_str = std::to_string(kPreviewPort);
        ::execl(sim_exe,
                sim_exe,
                "--serve",
                "--serve-port",
                serve_port_str.c_str(),
                "--sink",
                "udp",
                "--udp-host",
                "127.0.0.1",
                "--udp-port",
                preview_port_str.c_str(),
                "--serve-preview-stride",
                "1",
                nullptr);
        std::perror("execl");
        _exit(127);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    int fd = openUdpSocketWithRetry();
    if (fd < 0) {
        std::cerr << "socket failed\n";
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return 3;
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<std::uint16_t>(kServePort));
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
    cfg.gravity = {0.0, -9.81, 0.0};
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
        return 8;
    }
    if (ack.joint_count < 18) {
        std::cerr << "unexpected joint_count " << ack.joint_count << "\n";
        ::close(fd);
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return 9;
    }

    physics_sim::StepCommand step{};
    step.message_type = static_cast<std::uint8_t>(physics_sim::MessageType::StepCommand);
    step.sequence_id = 7;
    step.dt_seconds = 1.0 / 60.0;
    for (float& target : step.joint_targets) {
        target = 0.05;
    }
    if (!sendAll(fd, &step, physics_sim::kStepCommandBytes)) {
        std::cerr << "send StepCommand failed\n";
        ::close(fd);
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return 10;
    }

    alignas(physics_sim::StateResponse) char rsp_buf[sizeof(physics_sim::StateResponse)]{};
    if (!recvAll(fd, rsp_buf, physics_sim::kStateResponseBytes)) {
        std::cerr << "StateResponse recv failed\n";
        ::close(fd);
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return 11;
    }
    physics_sim::StateResponse rsp{};
    if (!physics_sim::tryDecodeStateResponse(rsp_buf, physics_sim::kStateResponseBytes, rsp)) {
        std::cerr << "StateResponse invalid\n";
        ::close(fd);
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return 12;
    }
    if (rsp.sequence_id != step.sequence_id) {
        std::cerr << "sequence mismatch\n";
        ::close(fd);
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return 13;
    }
    if (!std::isfinite(rsp.body_position[1]) || rsp.body_position[1] < 0.05) {
        std::cerr << "expected chassis above ground\n";
        ::close(fd);
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return 14;
    }
    if (rsp.matrix_lidar_valid != 1) {
        std::cerr << "expected matrix lidar in serve preview mode\n";
        ::close(fd);
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return 15;
    }

    ::close(fd);
    ::kill(pid, SIGTERM);
    ::waitpid(pid, nullptr, 0);
    std::cout << "test_serve_ipc_preview ok body_y=" << rsp.body_position[1] << "\n";
    return 0;
#endif
}
