#include "telemetry_publisher.hpp"

#include <arpa/inet.h>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <optional>
#include <string>
#include <sys/socket.h>
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

bool containsInOrder(const std::string& text,
                     const std::string& first,
                     const std::string& second,
                     const std::string& third)
{
    const std::size_t i0 = text.find(first);
    const std::size_t i1 = text.find(second);
    const std::size_t i2 = text.find(third);
    return i0 != std::string::npos && i1 != std::string::npos && i2 != std::string::npos &&
           i0 < i1 && i1 < i2;
}

std::optional<double> extractNumberAfter(const std::string& payload, const std::string& token)
{
    const std::size_t start = payload.find(token);
    if (start == std::string::npos) {
        return std::nullopt;
    }

    const std::size_t value_start = start + token.size();
    const std::size_t value_end = payload.find_first_of(",}", value_start);
    if (value_end == std::string::npos || value_end <= value_start) {
        return std::nullopt;
    }

    const std::string raw = payload.substr(value_start, value_end - value_start);
    char* end_ptr = nullptr;
    const double parsed = std::strtod(raw.c_str(), &end_ptr);
    if (end_ptr == raw.c_str()) {
        return std::nullopt;
    }
    return parsed;
}

std::optional<double> extractNestedNumber(const std::string& payload,
                                         const std::string& object_token,
                                         const std::string& field_token)
{
    const std::size_t object_start = payload.find(object_token);
    if (object_start == std::string::npos) {
        return std::nullopt;
    }

    const std::size_t object_end = payload.find("}", object_start + object_token.size());
    if (object_end == std::string::npos) {
        return std::nullopt;
    }

    const std::string object_slice = payload.substr(object_start, object_end - object_start + 1);
    return extractNumberAfter(object_slice, field_token);
}

std::optional<std::string> extractStringAfter(const std::string& payload, const std::string& token)
{
    const std::size_t start = payload.find(token);
    if (start == std::string::npos) {
        return std::nullopt;
    }

    const std::size_t value_start = start + token.size();
    const std::size_t value_end = payload.find('"', value_start);
    if (value_end == std::string::npos || value_end < value_start) {
        return std::nullopt;
    }

    return payload.substr(value_start, value_end - value_start);
}

bool setupUdpListener(int& socket_fd, uint16_t& port)
{
    socket_fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd < 0) {
        std::cerr << "FAIL: unable to create UDP listener socket errno=" << errno << '\n';
        return false;
    }

    sockaddr_in bind_addr{};
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    bind_addr.sin_port = htons(0);

    if (::bind(socket_fd, reinterpret_cast<const sockaddr*>(&bind_addr), sizeof(bind_addr)) != 0) {
        std::cerr << "FAIL: unable to bind UDP listener errno=" << errno << '\n';
        ::close(socket_fd);
        socket_fd = -1;
        return false;
    }

    sockaddr_in actual_addr{};
    socklen_t len = sizeof(actual_addr);
    if (::getsockname(socket_fd, reinterpret_cast<sockaddr*>(&actual_addr), &len) != 0) {
        std::cerr << "FAIL: unable to read UDP listener port errno=" << errno << '\n';
        ::close(socket_fd);
        socket_fd = -1;
        return false;
    }

    port = ntohs(actual_addr.sin_port);

    timeval timeout{};
    timeout.tv_sec = 0;
    timeout.tv_usec = 300000;
    (void)::setsockopt(socket_fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    return true;
}

std::optional<std::string> receiveDatagram(const int socket_fd)
{
    char buffer[65535];
    const ssize_t bytes = ::recvfrom(socket_fd, buffer, sizeof(buffer), 0, nullptr, nullptr);
    if (bytes < 0) {
        return std::nullopt;
    }
    return std::string(buffer, static_cast<std::size_t>(bytes));
}

telemetry::ControlStepTelemetry makeSampleTelemetry()
{
    telemetry::ControlStepTelemetry telemetry{};
    telemetry.timestamp_us = TimePointUs{12345678};
    telemetry.motion_intent.speed_mps = LinearRateMps{0.42};
    telemetry.motion_intent.heading_rad = AngleRad{0.9};

    telemetry.estimated_state.body_pose_state.body_trans_m = PositionM3{1.2, -0.3, 0.18};
    telemetry.estimated_state.body_pose_state.body_trans_mps = VelocityMps3{0.11, -0.08, 0.01};
    telemetry.estimated_state.body_pose_state.orientation_rad = EulerAnglesRad3{0.05, -0.02, 0.3};
    telemetry.estimated_state.body_pose_state.angular_velocity_radps =
        AngularVelocityRadPerSec3{0.01, 0.02, 0.03};

    telemetry.status.loop_counter = 77;
    telemetry.status.active_mode = RobotMode::WALK;
    telemetry.status.bus_ok = true;
    telemetry.status.estimator_valid = true;
    telemetry.status.dynamic_gait.limiter_enabled = true;
    telemetry.status.dynamic_gait.limiter_phase = 1;
    telemetry.status.dynamic_gait.active_constraint_reason = 2;
    telemetry.status.dynamic_gait.adaptation_scale_linear = 0.72;
    telemetry.status.dynamic_gait.adaptation_scale_yaw = 0.90;
    telemetry.status.dynamic_gait.adaptation_scale_cadence = 0.68;
    telemetry.status.dynamic_gait.adaptation_scale_step = 0.68;
    telemetry.status.dynamic_gait.hard_clamp_linear = true;
    telemetry.status.dynamic_gait.hard_clamp_yaw = false;
    telemetry.status.dynamic_gait.hard_clamp_reach = true;
    telemetry.status.dynamic_gait.hard_clamp_cadence = true;
    telemetry.status.dynamic_gait.saturated = true;
    return telemetry;
}

bool testPublishControlStepIncludesLocalizationPoseWhenPresent()
{
    int listener_fd = -1;
    uint16_t listener_port = 0;
    if (!setupUdpListener(listener_fd, listener_port)) {
        return false;
    }

    telemetry::TelemetryPublisherConfig config{};
    config.enabled = true;
    config.udp_host = "127.0.0.1";
    config.udp_port = static_cast<int>(listener_port);

    auto publisher = telemetry::makeUdpTelemetryPublisher(config, nullptr);

    telemetry::ControlStepTelemetry telemetry = makeSampleTelemetry();
    telemetry.autonomy_debug.localization_pose = telemetry::ControlStepTelemetry::AutonomyDebugPose{
        4.5,
        -2.1,
        1.57,
        "map"
    };

    publisher->publishControlStep(telemetry);
    const std::optional<std::string> payload = receiveDatagram(listener_fd);
    ::close(listener_fd);

    return expect(payload.has_value(), "expected control-step datagram to be emitted") &&
           expect(payload->find("\"autonomy_debug\":{") != std::string::npos,
                  "payload should include autonomy_debug object") &&
           expect(payload->find("\"current_pose\":{") != std::string::npos,
                  "payload should include autonomy_debug.current_pose") &&
           expect(payload->find("\"localization\":{") != std::string::npos,
                  "payload should include autonomy_debug.localization") &&
           expect(containsInOrder(*payload,
                                  "\"autonomy_debug\":{",
                                  "\"current_pose\":{",
                                  "\"localization\":{"),
                  "autonomy_debug should deterministically place current_pose before localization") &&
           expect(payload->find("\"frame_id\":\"map\"") != std::string::npos,
                  "localization frame_id should come from provided localization pose") &&
           expect(payload->find("\"localization\":{\"frame_id\":\"map\",\"current_pose\":{") != std::string::npos,
                  "localization.current_pose should be emitted under localization") &&
           expect(payload->find("\"current_pose\":{\"x_m\":4.5,\"y_m\":-2.1,\"yaw_rad\":1.57}") != std::string::npos,
                  "autonomy_debug.current_pose should reflect localization pose values") &&
           expect(payload->find("\"limiter\":{\"enabled\":true") != std::string::npos,
                  "payload should include limiter enabled flag") &&
           expect(payload->find("\"phase\":\"body_leads_on_start\"") != std::string::npos,
                  "payload should include limiter phase") &&
           expect(payload->find("\"active_constraint_reason\":\"slew_rate\"") != std::string::npos,
                  "payload should include limiter constraint reason") &&
           expect(payload->find("\"adaptation_scales\":{\"linear\":0.72,\"yaw\":0.9,\"cadence\":0.68,\"step\":0.68}") != std::string::npos,
                  "payload should include limiter adaptation scales") &&
           expect(payload->find("\"hard_clamp\":{\"linear\":true,\"yaw\":false,\"reach\":true,\"cadence\":true,\"saturated\":true}") != std::string::npos,
                  "payload should include hard-clamp saturation flags");
}

bool testPublishControlStepFallsBackToOdometryWhenLocalizationAbsent()
{
    int listener_fd = -1;
    uint16_t listener_port = 0;
    if (!setupUdpListener(listener_fd, listener_port)) {
        return false;
    }

    telemetry::TelemetryPublisherConfig config{};
    config.enabled = true;
    config.udp_host = "127.0.0.1";
    config.udp_port = static_cast<int>(listener_port);

    auto publisher = telemetry::makeUdpTelemetryPublisher(config, nullptr);

    telemetry::ControlStepTelemetry telemetry = makeSampleTelemetry();
    telemetry.timestamp_us = TimePointUs{22345678};
    telemetry.autonomy_debug.localization_pose.reset();

    publisher->publishControlStep(telemetry);
    const std::optional<std::string> payload = receiveDatagram(listener_fd);
    ::close(listener_fd);

    if (!expect(payload.has_value(), "expected fallback control-step datagram to be emitted") ||
        !expect(payload->find("\"autonomy_debug\":{") != std::string::npos,
                "payload should include autonomy_debug object") ||
        !expect(payload->find("\"localization\":{\"frame_id\":\"odom\"") != std::string::npos,
                "fallback frame_id should default to odom") ||
        !expect(payload->find("\"localization\":{\"frame_id\":\"odom\",\"current_pose\":{") != std::string::npos,
                "fallback should include localization.current_pose object") ||
        !expect(payload->find("\"type\":\"joints\"") != std::string::npos,
                "fallback payload should remain schema-valid for joints packets") ||
        !expect(payload->find("\"schema_version\":1") != std::string::npos,
                "fallback payload should preserve schema version")) {
        return false;
    }

    const std::optional<std::string> frame_id = extractStringAfter(*payload, "\"frame_id\":\"");
    const std::optional<double> current_x =
        extractNestedNumber(*payload, "\"autonomy_debug\":{\"current_pose\":{", "\"x_m\":");
    const std::optional<double> current_y =
        extractNestedNumber(*payload, "\"autonomy_debug\":{\"current_pose\":{", "\"y_m\":");
    const std::optional<double> current_yaw =
        extractNestedNumber(*payload, "\"autonomy_debug\":{\"current_pose\":{", "\"yaw_rad\":");

    return expect(frame_id.has_value() && *frame_id == "odom", "frame_id should parse and equal odom") &&
           expect(current_x.has_value(), "fallback current_pose.x_m should be parseable") &&
           expect(current_y.has_value(), "fallback current_pose.y_m should be parseable") &&
           expect(current_yaw.has_value(), "fallback current_pose.yaw_rad should be parseable") &&
           expect(std::isfinite(*current_x), "fallback current_pose.x_m should be finite") &&
           expect(std::isfinite(*current_y), "fallback current_pose.y_m should be finite") &&
           expect(std::isfinite(*current_yaw), "fallback current_pose.yaw_rad should be finite");
}

} // namespace

int main()
{
    if (!testPublishControlStepIncludesLocalizationPoseWhenPresent()) {
        return EXIT_FAILURE;
    }
    if (!testPublishControlStepFallsBackToOdometryWhenLocalizationAbsent()) {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
