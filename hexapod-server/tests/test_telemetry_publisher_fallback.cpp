#include "control/telemetry_publisher.hpp"

#include <arpa/inet.h>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <optional>
#include <string>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

namespace {

bool expect(const bool condition, const std::string& message)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

std::optional<double> extractNumberAfter(const std::string& text, const std::string& key)
{
    const std::size_t key_pos = text.find(key);
    if (key_pos == std::string::npos) {
        return std::nullopt;
    }
    const std::size_t start = key_pos + key.size();
    const std::size_t end = text.find_first_of(",}", start);
    if (end == std::string::npos || end <= start) {
        return std::nullopt;
    }

    try {
        return std::stod(text.substr(start, end - start));
    } catch (...) {
        return std::nullopt;
    }
}

std::optional<bool> extractBoolAfter(const std::string& text, const std::string& key)
{
    const std::size_t key_pos = text.find(key);
    if (key_pos == std::string::npos) {
        return std::nullopt;
    }
    const std::size_t start = key_pos + key.size();
    if (text.compare(start, 4, "true") == 0) {
        return true;
    }
    if (text.compare(start, 5, "false") == 0) {
        return false;
    }
    return std::nullopt;
}

std::optional<std::string> recvPacket(const int socket_fd)
{
    char buffer[16384];
    const ssize_t bytes = ::recvfrom(socket_fd, buffer, sizeof(buffer), 0, nullptr, nullptr);
    if (bytes <= 0) {
        return std::nullopt;
    }
    return std::string(buffer, static_cast<std::size_t>(bytes));
}

bool closeEnough(const double lhs, const double rhs, const double tol = 1e-6)
{
    return std::abs(lhs - rhs) <= tol;
}

telemetry::ControlStepTelemetry makeStep(const uint64_t timestamp_us,
                                         const double speed_mps,
                                         const double heading_rad,
                                         const double yaw_rate_radps,
                                         const double estimated_vx_mps = 0.0,
                                         const double estimated_vy_mps = 0.0)
{
    telemetry::ControlStepTelemetry step{};
    step.timestamp_us = TimePointUs{timestamp_us};
    step.status.active_mode = RobotMode::WALK;
    step.motion_intent.speed_mps = LinearRateMps{speed_mps};
    step.motion_intent.heading_rad = AngleRad{heading_rad};
    step.estimated_state.body_pose_state.body_trans_mps.x = estimated_vx_mps;
    step.estimated_state.body_pose_state.body_trans_mps.y = estimated_vy_mps;
    step.estimated_state.body_pose_state.angular_velocity_radps.z = yaw_rate_radps;
    return step;
}

bool testFallbackIntegratorHoldsPoseOnTimestampRegressionAndLongPause()
{
    const int recv_socket = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (!expect(recv_socket >= 0, "should create UDP receiver socket")) {
        return false;
    }

    sockaddr_in recv_addr{};
    recv_addr.sin_family = AF_INET;
    recv_addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    recv_addr.sin_port = 0;
    if (!expect(::bind(recv_socket, reinterpret_cast<const sockaddr*>(&recv_addr), sizeof(recv_addr)) == 0,
                "should bind UDP receiver")) {
        ::close(recv_socket);
        return false;
    }

    timeval timeout{};
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;
    (void)::setsockopt(recv_socket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    socklen_t addr_len = sizeof(recv_addr);
    if (!expect(::getsockname(recv_socket, reinterpret_cast<sockaddr*>(&recv_addr), &addr_len) == 0,
                "should read bound UDP port")) {
        ::close(recv_socket);
        return false;
    }
    const int recv_port = ntohs(recv_addr.sin_port);

    telemetry::TelemetryPublisherConfig cfg{};
    cfg.enabled = true;
    cfg.udp_host = "127.0.0.1";
    cfg.udp_port = recv_port;
    auto publisher = telemetry::makeUdpTelemetryPublisher(cfg, nullptr);

    publisher->publishControlStep(makeStep(1'000'000, 1.0, 0.0, 0.0, 1.0, 0.0));
    auto packet = recvPacket(recv_socket);
    if (!expect(packet.has_value(), "should receive packet for first sample")) {
        ::close(recv_socket);
        return false;
    }

    publisher->publishControlStep(makeStep(1'500'000, 1.0, 0.0, 0.0, 1.0, 0.0));
    packet = recvPacket(recv_socket);
    const auto x_after_motion = packet ? extractNumberAfter(*packet, "\"x_m\":") : std::nullopt;
    if (!expect(packet.has_value() && x_after_motion.has_value(),
                "should decode fallback x pose after nominal integration")) {
        ::close(recv_socket);
        return false;
    }
    if (!expect(closeEnough(*x_after_motion, 0.25),
                "dt clamp should cap position integration at 0.25 m")) {
        ::close(recv_socket);
        return false;
    }

    publisher->publishControlStep(makeStep(1'400'000, 1.0, 0.0, 0.0, 1.0, 0.0));
    packet = recvPacket(recv_socket);
    const auto x_after_regression = packet ? extractNumberAfter(*packet, "\"x_m\":") : std::nullopt;
    const auto reset_after_regression = packet ? extractBoolAfter(*packet, "\"pose_reset\":") : std::nullopt;
    if (!expect(packet.has_value() && x_after_regression.has_value(),
                "should decode fallback x pose after timestamp regression")) {
        ::close(recv_socket);
        return false;
    }
    if (!expect(closeEnough(*x_after_regression, *x_after_motion),
                "timestamp regression should preserve fallback pose continuity")) {
        ::close(recv_socket);
        return false;
    }
    if (!expect(reset_after_regression.has_value() && *reset_after_regression,
                "timestamp regression should publish pose_reset flag")) {
        ::close(recv_socket);
        return false;
    }

    publisher->publishControlStep(makeStep(2'900'000, 1.0, 0.0, 0.0, 1.0, 0.0));
    packet = recvPacket(recv_socket);
    const auto x_after_pause = packet ? extractNumberAfter(*packet, "\"x_m\":") : std::nullopt;
    const auto reset_after_pause = packet ? extractBoolAfter(*packet, "\"pose_reset\":") : std::nullopt;
    if (!expect(packet.has_value() && x_after_pause.has_value(),
                "should decode fallback x pose after long pause")) {
        ::close(recv_socket);
        return false;
    }
    if (!expect(closeEnough(*x_after_pause, *x_after_motion),
                "long pause should preserve fallback pose continuity")) {
        ::close(recv_socket);
        return false;
    }
    if (!expect(reset_after_pause.has_value() && *reset_after_pause,
                "long pause should publish pose_reset flag")) {
        ::close(recv_socket);
        return false;
    }

    ::close(recv_socket);
    return true;
}

bool testFallbackIntegratorIgnoresNonFiniteInputsAndResetsOnIdle()
{
    const int recv_socket = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (!expect(recv_socket >= 0, "should create UDP receiver socket")) {
        return false;
    }

    sockaddr_in recv_addr{};
    recv_addr.sin_family = AF_INET;
    recv_addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    recv_addr.sin_port = 0;
    if (!expect(::bind(recv_socket, reinterpret_cast<const sockaddr*>(&recv_addr), sizeof(recv_addr)) == 0,
                "should bind UDP receiver")) {
        ::close(recv_socket);
        return false;
    }

    timeval timeout{};
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;
    (void)::setsockopt(recv_socket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    socklen_t addr_len = sizeof(recv_addr);
    if (!expect(::getsockname(recv_socket, reinterpret_cast<sockaddr*>(&recv_addr), &addr_len) == 0,
                "should read bound UDP port")) {
        ::close(recv_socket);
        return false;
    }
    const int recv_port = ntohs(recv_addr.sin_port);

    telemetry::TelemetryPublisherConfig cfg{};
    cfg.enabled = true;
    cfg.udp_host = "127.0.0.1";
    cfg.udp_port = recv_port;
    auto publisher = telemetry::makeUdpTelemetryPublisher(cfg, nullptr);

    publisher->publishControlStep(makeStep(1'000'000, 1.0, 0.0, 1.0));
    auto packet = recvPacket(recv_socket);
    if (!expect(packet.has_value(), "should receive packet for first sample")) {
        ::close(recv_socket);
        return false;
    }

    telemetry::ControlStepTelemetry non_finite = makeStep(1'100'000, NAN, NAN, NAN);
    publisher->publishControlStep(non_finite);
    packet = recvPacket(recv_socket);
    const auto x_after_non_finite = packet ? extractNumberAfter(*packet, "\"x_m\":") : std::nullopt;
    const auto yaw_after_non_finite = packet ? extractNumberAfter(*packet, "\"yaw_rad\":") : std::nullopt;
    if (!expect(packet.has_value() && x_after_non_finite.has_value() && yaw_after_non_finite.has_value(),
                "should decode fallback pose after non-finite input")) {
        ::close(recv_socket);
        return false;
    }
    if (!expect(closeEnough(*x_after_non_finite, 0.0), "non-finite inputs should not advance x pose")) {
        ::close(recv_socket);
        return false;
    }
    if (!expect(closeEnough(*yaw_after_non_finite, 0.0), "non-finite inputs should not advance yaw pose")) {
        ::close(recv_socket);
        return false;
    }

    publisher->publishControlStep(makeStep(1'600'000, 1.0, 0.0, 0.0, 1.0, 0.0));
    packet = recvPacket(recv_socket);
    const auto x_before_idle = packet ? extractNumberAfter(*packet, "\"x_m\":") : std::nullopt;
    if (!expect(packet.has_value() && x_before_idle.has_value(), "should decode fallback x pose before idle")) {
        ::close(recv_socket);
        return false;
    }
    if (!expect(closeEnough(*x_before_idle, 0.25),
                "pre-idle movement should integrate to a non-zero fallback pose")) {
        ::close(recv_socket);
        return false;
    }

    telemetry::ControlStepTelemetry idle_step = makeStep(1'200'000, 1.0, 0.0, 0.0);
    idle_step.status.active_mode = RobotMode::SAFE_IDLE;
    publisher->publishControlStep(idle_step);
    packet = recvPacket(recv_socket);
    const auto x_after_idle = packet ? extractNumberAfter(*packet, "\"x_m\":") : std::nullopt;
    const auto reset_after_idle = packet ? extractBoolAfter(*packet, "\"pose_reset\":") : std::nullopt;
    if (!expect(packet.has_value() && x_after_idle.has_value(), "should decode fallback x pose in idle mode")) {
        ::close(recv_socket);
        return false;
    }
    if (!expect(closeEnough(*x_after_idle, *x_before_idle),
                "idle transition should hold last valid fallback pose")) {
        ::close(recv_socket);
        return false;
    }
    if (!expect(reset_after_idle.has_value() && *reset_after_idle,
                "idle transition should publish pose_reset flag")) {
        ::close(recv_socket);
        return false;
    }

    publisher->publishControlStep(makeStep(1'300'000, 0.0, 0.0, 0.0, 0.0, 0.0));
    packet = recvPacket(recv_socket);
    const auto reset_after_recovery = packet ? extractBoolAfter(*packet, "\"pose_reset\":") : std::nullopt;
    if (!expect(packet.has_value() && reset_after_recovery.has_value(),
                "should decode pose_reset after SAFE_IDLE recovery sample")) {
        ::close(recv_socket);
        return false;
    }
    if (!expect(!*reset_after_recovery, "pose_reset should clear after non-reset samples")) {
        ::close(recv_socket);
        return false;
    }

    ::close(recv_socket);
    return true;
}

bool testFallbackIntegratorDoesNotUseCommandedVelocityWhenEstimatorVelocityIsZero()
{
    const int recv_socket = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (!expect(recv_socket >= 0, "should create UDP receiver socket")) {
        return false;
    }

    sockaddr_in recv_addr{};
    recv_addr.sin_family = AF_INET;
    recv_addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    recv_addr.sin_port = 0;
    if (!expect(::bind(recv_socket, reinterpret_cast<const sockaddr*>(&recv_addr), sizeof(recv_addr)) == 0,
                "should bind UDP receiver")) {
        ::close(recv_socket);
        return false;
    }

    timeval timeout{};
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;
    (void)::setsockopt(recv_socket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    socklen_t addr_len = sizeof(recv_addr);
    if (!expect(::getsockname(recv_socket, reinterpret_cast<sockaddr*>(&recv_addr), &addr_len) == 0,
                "should read bound UDP port")) {
        ::close(recv_socket);
        return false;
    }
    const int recv_port = ntohs(recv_addr.sin_port);

    telemetry::TelemetryPublisherConfig cfg{};
    cfg.enabled = true;
    cfg.udp_host = "127.0.0.1";
    cfg.udp_port = recv_port;
    auto publisher = telemetry::makeUdpTelemetryPublisher(cfg, nullptr);

    publisher->publishControlStep(makeStep(1'000'000, 0.8, 0.0, 0.0, 0.0, 0.0));
    auto packet = recvPacket(recv_socket);
    if (!expect(packet.has_value(), "should receive packet for initial command-only sample")) {
        ::close(recv_socket);
        return false;
    }

    publisher->publishControlStep(makeStep(1'250'000, 0.8, 0.0, 0.0, 0.0, 0.0));
    packet = recvPacket(recv_socket);
    const auto x_after_command_only = packet ? extractNumberAfter(*packet, "\"x_m\":") : std::nullopt;
    if (!expect(packet.has_value() && x_after_command_only.has_value(),
                "should decode fallback x pose for command-only sample")) {
        ::close(recv_socket);
        return false;
    }
    if (!expect(closeEnough(*x_after_command_only, 0.0),
                "commanded velocity without estimator motion should not advance fallback pose")) {
        ::close(recv_socket);
        return false;
    }

    ::close(recv_socket);
    return true;
}

} // namespace

int main()
{
    if (!testFallbackIntegratorHoldsPoseOnTimestampRegressionAndLongPause()) {
        return EXIT_FAILURE;
    }
    if (!testFallbackIntegratorIgnoresNonFiniteInputsAndResetsOnIdle()) {
        return EXIT_FAILURE;
    }
    if (!testFallbackIntegratorDoesNotUseCommandedVelocityWhenEstimatorVelocityIsZero()) {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
