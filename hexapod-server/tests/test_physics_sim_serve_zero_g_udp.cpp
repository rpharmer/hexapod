// Fork hexapod-physics-sim --serve, zero gravity, peek wire angles, then:
//  (1) algebraic round-trip via physics_sim_joint_wire_mapping + default geometry
//  (2) many hold-steps: commanded angles == peek; joint drift should stay tiny in 0g
//
// Usage: test_physics_sim_serve_zero_g_udp PATH_TO_hexapod-physics-sim
// Or set HEXAPOD_PHYSICS_SIM_EXE to the same path.

#include "geometry_config.hpp"
#include "physics_sim_joint_wire_mapping.hpp"
#include "physics_sim_protocol.hpp"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

#if defined(__linux__)
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>
#endif

namespace {

bool send_all(int fd, const void* data, std::size_t len) {
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

bool recv_all(int fd, void* data, std::size_t len) {
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

double wrap_pi(double a) {
    return std::atan2(std::sin(a), std::cos(a));
}

} // namespace

int main(int argc, char** argv) {
#if !defined(__linux__)
    std::cerr << "test_physics_sim_serve_zero_g_udp: Linux-only\n";
    return 0;
#else
    const char* sim_exe = nullptr;
    if (argc >= 2 && argv[1][0] != '\0') {
        sim_exe = argv[1];
    } else {
        sim_exe = std::getenv("HEXAPOD_PHYSICS_SIM_EXE");
    }
    if (sim_exe == nullptr || sim_exe[0] == '\0') {
        std::cout << "skip test_physics_sim_serve_zero_g_udp (pass sim path or HEXAPOD_PHYSICS_SIM_EXE)\n";
        return 0;
    }

    const int kPort = 20000 + (static_cast<int>(::getpid()) % 8000);
    const HexapodGeometry geo = geometry_config::buildDefaultHexapodGeometry();

    pid_t pid = ::fork();
    if (pid < 0) {
        std::cerr << "fork failed\n";
        return 2;
    }
    if (pid == 0) {
        const std::string port_str = std::to_string(kPort);
        ::execl(sim_exe, sim_exe, "--serve", "--serve-port", port_str.c_str(), nullptr);
        std::perror("execl");
        _exit(127);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(250));

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
    cfg.gravity = {0.0f, 0.0f, 0.0f};
    cfg.solver_iterations = 24;
    if (!send_all(fd, &cfg, physics_sim::kConfigCommandBytes)) {
        std::cerr << "send ConfigCommand failed\n";
        ::close(fd);
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return 6;
    }

    alignas(physics_sim::ConfigAck) char ack_buf[sizeof(physics_sim::ConfigAck)]{};
    if (!recv_all(fd, ack_buf, physics_sim::kConfigAckBytes)) {
        std::cerr << "ConfigAck recv failed\n";
        ::close(fd);
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return 7;
    }
    physics_sim::ConfigAck ack{};
    if (!physics_sim::tryDecodeConfigAck(ack_buf, physics_sim::kConfigAckBytes, ack) || ack.joint_count < 18) {
        std::cerr << "ConfigAck invalid\n";
        ::close(fd);
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return 7;
    }

    physics_sim::StepCommand peek{};
    peek.message_type = static_cast<std::uint8_t>(physics_sim::MessageType::StepCommand);
    peek.sequence_id = 0;
    peek.dt_seconds = 1.0f;
    if (!send_all(fd, &peek, physics_sim::kStepCommandBytes)) {
        std::cerr << "peek send failed\n";
        ::close(fd);
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return 8;
    }
    alignas(physics_sim::StateResponse) char peek_buf[sizeof(physics_sim::StateResponse)]{};
    if (!recv_all(fd, peek_buf, physics_sim::kStateResponseBytes)) {
        std::cerr << "peek recv failed\n";
        ::close(fd);
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return 9;
    }
    physics_sim::StateResponse peek_rsp{};
    if (!physics_sim::tryDecodeStateResponse(peek_buf, physics_sim::kStateResponseBytes, peek_rsp) ||
        peek_rsp.sequence_id != 0) {
        std::cerr << "peek decode failed\n";
        ::close(fd);
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return 9;
    }

    float max_peek_abs = 0.0f;
    for (const float angle : peek_rsp.joint_angles) {
        max_peek_abs = std::max(max_peek_abs, std::abs(angle));
    }
    constexpr float kPeekZeroEps = 2.0e-4f;
    if (max_peek_abs > kPeekZeroEps) {
        std::cerr << "peek relaxed wire angles expected near zero max=" << max_peek_abs << '\n';
        ::close(fd);
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return 10;
    }

    double max_algebra_err = 0.0;
    for (int leg = 0; leg < kNumLegs; ++leg) {
        const ServoCalibration& cal = geo.legGeometry[static_cast<std::size_t>(leg)].servo;
        const int b = leg * 3;
        const float a0 = peek_rsp.joint_angles[static_cast<std::size_t>(b + 0)];
        const float a1 = peek_rsp.joint_angles[static_cast<std::size_t>(b + 1)];
        const float a2 = peek_rsp.joint_angles[static_cast<std::size_t>(b + 2)];
        const LegState servo =
            physics_sim_joint_wire_mapping::servoLegFromSimWireAngles(cal, leg, a0, a1, a2);
        float r0 = 0.0f;
        float r1 = 0.0f;
        float r2 = 0.0f;
        physics_sim_joint_wire_mapping::simWireTargetsFromServoLeg(cal, leg, servo, r0, r1, r2);
        max_algebra_err = std::max(max_algebra_err, std::abs(wrap_pi(static_cast<double>(r0 - a0))));
        max_algebra_err = std::max(max_algebra_err, std::abs(wrap_pi(static_cast<double>(r1 - a1))));
        max_algebra_err = std::max(max_algebra_err, std::abs(wrap_pi(static_cast<double>(r2 - a2))));
    }
    constexpr double kAlgebraEps = 2.0e-5;
    if (max_algebra_err > kAlgebraEps) {
        std::cerr << "peek wire angles do not round-trip mapping max_err=" << max_algebra_err << '\n';
        ::close(fd);
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return 11;
    }

    constexpr double kOffsetEps = 2.0e-5;
    for (int leg = 0; leg < kNumLegs; ++leg) {
        const ServoCalibration& cal = geo.legGeometry[static_cast<std::size_t>(leg)].servo;
        const LegState joint =
            physics_sim_joint_wire_mapping::jointMechanicalFromSimWireAngles(cal, leg, 0.0f, 0.0f, 0.0f);
        if (std::abs(joint.joint_state[COXA].pos_rad.value - physics_sim::kWireZeroCoxaMechanicalRad) > kOffsetEps ||
            std::abs(joint.joint_state[FEMUR].pos_rad.value - physics_sim::kWireZeroFemurMechanicalRad) > kOffsetEps ||
            std::abs(joint.joint_state[TIBIA].pos_rad.value - physics_sim::kWireZeroTibiaMechanicalRad) > kOffsetEps) {
            std::cerr << "zero-wire mapping mismatch leg=" << leg << " got=("
                      << joint.joint_state[COXA].pos_rad.value << ","
                      << joint.joint_state[FEMUR].pos_rad.value << ","
                      << joint.joint_state[TIBIA].pos_rad.value << ")\n";
            ::close(fd);
            ::kill(pid, SIGTERM);
            ::waitpid(pid, nullptr, 0);
            return 12;
        }
    }

    constexpr int kHoldSteps = 120;
    constexpr float kDt = 1.0f / 240.0f;
    float max_hold_drift = 0.0f;
    std::uint32_t seq = 1;
    for (int i = 0; i < kHoldSteps; ++i) {
        physics_sim::StepCommand step{};
        step.message_type = static_cast<std::uint8_t>(physics_sim::MessageType::StepCommand);
        step.sequence_id = seq++;
        step.dt_seconds = kDt;
        step.joint_targets = peek_rsp.joint_angles;
        if (!send_all(fd, &step, physics_sim::kStepCommandBytes)) {
            std::cerr << "step send failed\n";
            ::close(fd);
            ::kill(pid, SIGTERM);
            ::waitpid(pid, nullptr, 0);
            return 12;
        }
        alignas(physics_sim::StateResponse) char rsp_raw[sizeof(physics_sim::StateResponse)]{};
        if (!recv_all(fd, rsp_raw, physics_sim::kStateResponseBytes)) {
            std::cerr << "step recv failed\n";
            ::close(fd);
            ::kill(pid, SIGTERM);
            ::waitpid(pid, nullptr, 0);
            return 13;
        }
        physics_sim::StateResponse rsp{};
        if (!physics_sim::tryDecodeStateResponse(rsp_raw, physics_sim::kStateResponseBytes, rsp) ||
            rsp.sequence_id != step.sequence_id) {
            std::cerr << "step rsp bad\n";
            ::close(fd);
            ::kill(pid, SIGTERM);
            ::waitpid(pid, nullptr, 0);
            return 13;
        }
        for (std::size_t j = 0; j < rsp.joint_angles.size(); ++j) {
            const float d = static_cast<float>(
                std::abs(wrap_pi(static_cast<double>(rsp.joint_angles[j] - peek_rsp.joint_angles[j]))));
            max_hold_drift = std::max(max_hold_drift, d);
        }
    }

    constexpr float kMaxHoldDrift = 0.03f;
    if (max_hold_drift > kMaxHoldDrift) {
        std::cerr << "zero-g hold drift too large max=" << max_hold_drift << '\n';
        ::close(fd);
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return 14;
    }

    ::close(fd);
    ::kill(pid, SIGTERM);
    int st = 0;
    ::waitpid(pid, &st, 0);

    std::cout << "test_physics_sim_serve_zero_g_udp ok max_peek_abs=" << max_peek_abs
              << " max_algebra_err=" << max_algebra_err
              << " max_hold_drift=" << max_hold_drift << '\n';
    return 0;
#endif
}
