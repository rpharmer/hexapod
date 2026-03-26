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
    TelemetryPublishCounters counters() const override {
        return {};
    }
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

const char* gaitTypeToString(GaitType gait) {
    switch (gait) {
        case GaitType::TRIPOD: return "tripod";
        case GaitType::RIPPLE: return "ripple";
        case GaitType::WAVE: return "wave";
    }
    return "unknown";
}

const char* dynamicRegionToString(uint8_t region) {
    switch (region) {
        case 0: return "arc";
        case 1: return "pivot";
        case 2: return "reorientation";
        default: return "unknown";
    }
}

const char* turnModeToString(uint8_t turn_mode) {
    switch (turn_mode) {
        case 0: return "crab";
        case 1: return "in_place";
        default: return "unknown";
    }
}

const char* fallbackStageToString(uint8_t fallback_stage) {
    switch (fallback_stage) {
        case 0: return "none";
        case 1: return "stability";
        case 2: return "degraded_locomotion";
        case 3: return "safe_stop";
        case 4: return "fault_hold";
        default: return "unknown";
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
                << "\"dynamic_gait\":{"
                << "\"valid\":" << (telemetry.status.dynamic_gait.valid ? "true" : "false") << ","
                << "\"gait_family\":\"" << gaitTypeToString(telemetry.status.dynamic_gait.gait_family) << "\","
                << "\"region\":\"" << dynamicRegionToString(telemetry.status.dynamic_gait.region) << "\","
                << "\"turn_mode\":\"" << turnModeToString(telemetry.status.dynamic_gait.turn_mode) << "\","
                << "\"fallback_stage\":\"" << fallbackStageToString(telemetry.status.dynamic_gait.fallback_stage) << "\","
                << "\"cadence_hz\":" << telemetry.status.dynamic_gait.cadence_hz << ","
                << "\"reach_utilization\":" << telemetry.status.dynamic_gait.reach_utilization << ","
                << "\"suppress_stride_progression\":"
                << (telemetry.status.dynamic_gait.suppress_stride_progression ? "true" : "false") << ","
                << "\"suppress_turning\":"
                << (telemetry.status.dynamic_gait.suppress_turning ? "true" : "false") << ","
                << "\"prioritize_stability\":"
                << (telemetry.status.dynamic_gait.prioritize_stability ? "true" : "false") << ","
                << "\"envelope\":{"
                << "\"max_speed_normalized\":" << telemetry.status.dynamic_gait.envelope_max_speed_normalized << ","
                << "\"max_yaw_normalized\":" << telemetry.status.dynamic_gait.envelope_max_yaw_normalized << ","
                << "\"max_roll_pitch_rad\":" << telemetry.status.dynamic_gait.envelope_max_roll_pitch_rad << ","
                << "\"allow_tripod\":"
                << (telemetry.status.dynamic_gait.envelope_allow_tripod ? "true" : "false")
                << "},"
                << "\"leg_phase\":["
                << telemetry.status.dynamic_gait.leg_phase[0] << ','
                << telemetry.status.dynamic_gait.leg_phase[1] << ','
                << telemetry.status.dynamic_gait.leg_phase[2] << ','
                << telemetry.status.dynamic_gait.leg_phase[3] << ','
                << telemetry.status.dynamic_gait.leg_phase[4] << ','
                << telemetry.status.dynamic_gait.leg_phase[5] << "],"
                << "\"leg_duty_cycle\":["
                << telemetry.status.dynamic_gait.leg_duty_cycle[0] << ','
                << telemetry.status.dynamic_gait.leg_duty_cycle[1] << ','
                << telemetry.status.dynamic_gait.leg_duty_cycle[2] << ','
                << telemetry.status.dynamic_gait.leg_duty_cycle[3] << ','
                << telemetry.status.dynamic_gait.leg_duty_cycle[4] << ','
                << telemetry.status.dynamic_gait.leg_duty_cycle[5] << "],"
                << "\"leg_in_stance\":["
                << (telemetry.status.dynamic_gait.leg_in_stance[0] ? "true" : "false") << ','
                << (telemetry.status.dynamic_gait.leg_in_stance[1] ? "true" : "false") << ','
                << (telemetry.status.dynamic_gait.leg_in_stance[2] ? "true" : "false") << ','
                << (telemetry.status.dynamic_gait.leg_in_stance[3] ? "true" : "false") << ','
                << (telemetry.status.dynamic_gait.leg_in_stance[4] ? "true" : "false") << ','
                << (telemetry.status.dynamic_gait.leg_in_stance[5] ? "true" : "false")
                << "]},"
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
            ++packets_sent_count_;
            last_successful_send_timestamp_us_ = now_us().value;
            return;
        }

        ++send_error_count_;
        ++socket_send_failures_;
        if (logger_ && send_error_count_ == 1) {
            LOG_WARN(logger_, "telemetry UDP send failed; dropping frame (errno=", errno, ")");
        }
    }

public:
    TelemetryPublishCounters counters() const override {
        return TelemetryPublishCounters{
            packets_sent_count_,
            socket_send_failures_,
            TimePointUs{last_successful_send_timestamp_us_},
        };
    }

private:
    int socket_fd_{-1};
    sockaddr_in destination_{};
    std::shared_ptr<logging::AsyncLogger> logger_;
    uint64_t send_error_count_{0};
    uint64_t packets_sent_count_{0};
    uint64_t socket_send_failures_{0};
    uint64_t last_successful_send_timestamp_us_{0};
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
