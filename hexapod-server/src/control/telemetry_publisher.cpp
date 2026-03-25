#include "telemetry_publisher.hpp"

#include <arpa/inet.h>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <sstream>
#include <sys/socket.h>
#include <unistd.h>

namespace telemetry {

namespace {

constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;

class NoopTelemetryPublisher final : public ITelemetryPublisher {
public:
    void publishGeometry(const HexapodGeometry&) override {}
    void publishControlStep(const ControlStepTelemetry&) override {}
};

std::string legNameFromIndex(int leg_index) {
    switch (leg_index) {
        case 0: return "RR";
        case 1: return "LR";
        case 2: return "RM";
        case 3: return "LM";
        case 4: return "RF";
        case 5: return "LF";
        default: return "UNK";
    }
}

class UdpTelemetryPublisher final : public ITelemetryPublisher {
public:
    UdpTelemetryPublisher(const TelemetryPublisherConfig& config,
                          std::shared_ptr<logging::AsyncLogger> logger)
        : logger_(std::move(logger)) {
        socket_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (socket_fd_ < 0) {
            if (logger_) {
                LOG_WARN(logger_, "telemetry publisher disabled: failed to create UDP socket");
            }
            return;
        }

        const int flags = ::fcntl(socket_fd_, F_GETFL, 0);
        if (flags >= 0) {
            (void)::fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK);
        }

        std::memset(&destination_, 0, sizeof(destination_));
        destination_.sin_family = AF_INET;
        destination_.sin_port = htons(static_cast<uint16_t>(config.udp_port));
        if (::inet_pton(AF_INET, config.udp_host.c_str(), &destination_.sin_addr) != 1) {
            if (logger_) {
                LOG_WARN(logger_, "telemetry publisher disabled: invalid UDP host '", config.udp_host, "'");
            }
            ::close(socket_fd_);
            socket_fd_ = -1;
            return;
        }
    }

    ~UdpTelemetryPublisher() override {
        if (socket_fd_ >= 0) {
            ::close(socket_fd_);
        }
    }

    void publishGeometry(const HexapodGeometry& geometry) override {
        if (socket_fd_ < 0) {
            return;
        }

        double body_radius_m = 0.0;
        for (int leg = 0; leg < kNumLegs; ++leg) {
            const auto& offset = geometry.legGeometry[leg].bodyCoxaOffset;
            const double radius = std::sqrt((offset.x * offset.x) + (offset.y * offset.y));
            if (radius > body_radius_m) {
                body_radius_m = radius;
            }
        }

        std::ostringstream payload;
        payload << "{\"schema_version\":1,\"geometry\":{"
                << "\"coxa\":" << geometry.legGeometry[0].coxaLength.value * 1000.0 << ","
                << "\"femur\":" << geometry.legGeometry[0].femurLength.value * 1000.0 << ","
                << "\"tibia\":" << geometry.legGeometry[0].tibiaLength.value * 1000.0 << ","
                << "\"body_radius\":" << body_radius_m * 1000.0
                << "}}";

        send(payload.str());
    }

    void publishControlStep(const ControlStepTelemetry& telemetry) override {
        if (socket_fd_ < 0) {
            return;
        }

        std::ostringstream payload;
        payload << "{\"type\":\"joints\",\"schema_version\":1,"
                << "\"timestamp_ms\":" << (telemetry.timestamp_us.value / 1000ULL) << ","
                << "\"loop_counter\":" << telemetry.status.loop_counter << ","
                << "\"mode\":" << static_cast<int>(telemetry.status.active_mode) << ","
                << "\"bus_ok\":" << (telemetry.status.bus_ok ? "true" : "false") << ","
                << "\"estimator_valid\":" << (telemetry.status.estimator_valid ? "true" : "false") << ","
                << "\"angles_deg\":{";

        for (int leg = 0; leg < kNumLegs; ++leg) {
            if (leg != 0) {
                payload << ",";
            }
            payload << "\"" << legNameFromIndex(leg) << "\":["
                    << telemetry.joint_targets.leg_states[leg].joint_state[COXA].pos_rad.value * kRadToDeg << ","
                    << telemetry.joint_targets.leg_states[leg].joint_state[FEMUR].pos_rad.value * kRadToDeg << ","
                    << telemetry.joint_targets.leg_states[leg].joint_state[TIBIA].pos_rad.value * kRadToDeg
                    << "]";
        }

        payload << "}}";
        send(payload.str());
    }

private:
    void send(const std::string& payload) {
        const ssize_t sent = ::sendto(socket_fd_,
                                      payload.data(),
                                      payload.size(),
                                      MSG_DONTWAIT,
                                      reinterpret_cast<const sockaddr*>(&destination_),
                                      sizeof(destination_));
        if (sent >= 0) {
            send_error_count_ = 0;
            return;
        }

        ++send_error_count_;
        if (logger_ && send_error_count_ == 1) {
            LOG_WARN(logger_, "telemetry UDP send failed; dropping frame (errno=", errno, ")");
        }
    }

    int socket_fd_{-1};
    sockaddr_in destination_{};
    std::shared_ptr<logging::AsyncLogger> logger_;
    uint64_t send_error_count_{0};
};

} // namespace

std::unique_ptr<ITelemetryPublisher> makeNoopTelemetryPublisher() {
    return std::make_unique<NoopTelemetryPublisher>();
}

std::unique_ptr<ITelemetryPublisher> makeUdpTelemetryPublisher(
    const TelemetryPublisherConfig& config,
    std::shared_ptr<logging::AsyncLogger> logger) {
    return std::make_unique<UdpTelemetryPublisher>(config, std::move(logger));
}

} // namespace telemetry
