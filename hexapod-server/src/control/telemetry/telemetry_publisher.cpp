#include "telemetry_publisher.hpp"
#include "autonomy/mission_executive.hpp"

#include <arpa/inet.h>
#include <algorithm>
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
constexpr double kFallbackMaxDtSeconds = 0.25;
constexpr double kFallbackLongPauseResetSeconds = 1.0;

bool isFiniteOrZero(const double value) {
    return std::isfinite(value);
}

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

const char* limiterPhaseToString(uint8_t phase) {
    switch (phase) {
        case 0: return "tracking";
        case 1: return "body_leads_on_start";
        case 2: return "legs_lead_on_stop";
        default: return "unknown";
    }
}

const char* limiterConstraintReasonToString(uint8_t reason) {
    switch (reason) {
        case 0: return "none";
        case 1: return "transition";
        case 2: return "slew_rate";
        case 3: return "reach_clamp";
        default: return "unknown";
    }
}

const char* missionStateToString(uint8_t mission_state) {
    switch (static_cast<autonomy::MissionState>(mission_state)) {
        case autonomy::MissionState::Idle: return "idle";
        case autonomy::MissionState::Ready: return "ready";
        case autonomy::MissionState::Exec: return "exec";
        case autonomy::MissionState::Paused: return "paused";
        case autonomy::MissionState::Aborted: return "aborted";
        case autonomy::MissionState::Complete: return "complete";
    }
    return "unknown";
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

        const double timestamp_us = static_cast<double>(telemetry.timestamp_us.value);
        const bool should_reset_for_idle = telemetry.status.active_mode == RobotMode::SAFE_IDLE;
        if (should_reset_for_idle) {
            resetFallbackOdometry(timestamp_us);
        }

        bool should_integrate = true;
        if (!std::isfinite(timestamp_us) || timestamp_us <= 0.0) {
            resetFallbackOdometry(0.0);
            should_integrate = false;
        }

        double dt_seconds = 0.0;
        if (should_integrate) {
            if (last_odometry_timestamp_us_ <= 0.0) {
                last_odometry_timestamp_us_ = timestamp_us;
                should_integrate = false;
            } else {
                dt_seconds = (timestamp_us - last_odometry_timestamp_us_) * 1e-6;
                if (!std::isfinite(dt_seconds) || dt_seconds < 0.0 || dt_seconds > kFallbackLongPauseResetSeconds) {
                    resetFallbackOdometry(timestamp_us);
                    should_integrate = false;
                } else {
                    dt_seconds = std::clamp(dt_seconds, 0.0, kFallbackMaxDtSeconds);
                    last_odometry_timestamp_us_ = timestamp_us;
                }
            }
        }

        if (should_integrate && dt_seconds > 0.0) {
            const double estimated_vx = telemetry.estimated_state.body_pose_state.body_trans_mps.x;
            const double estimated_vy = telemetry.estimated_state.body_pose_state.body_trans_mps.y;
            const bool has_estimated_planar_velocity =
                isFiniteOrZero(estimated_vx) && isFiniteOrZero(estimated_vy) &&
                (std::abs(estimated_vx) > 1e-6 || std::abs(estimated_vy) > 1e-6);

            const double body_vx_mps = has_estimated_planar_velocity ? estimated_vx : 0.0;
            const double body_vy_mps = has_estimated_planar_velocity ? estimated_vy : 0.0;

            // Frame convention:
            //  - body_vx/body_vy are body-frame planar velocities (x forward, y left).
            //  - we rotate body velocities into the world/odom frame using the *current* fallback yaw.
            //  - position integrates world-frame velocity; yaw integrates body yaw-rate.
            const double yaw_rad = odometry_pose_yaw_rad_;
            if (isFiniteOrZero(yaw_rad) && isFiniteOrZero(body_vx_mps) && isFiniteOrZero(body_vy_mps)) {
                const double world_vx_mps =
                    (body_vx_mps * std::cos(yaw_rad)) - (body_vy_mps * std::sin(yaw_rad));
                const double world_vy_mps =
                    (body_vx_mps * std::sin(yaw_rad)) + (body_vy_mps * std::cos(yaw_rad));
                odometry_pose_x_m_ += world_vx_mps * dt_seconds;
                odometry_pose_y_m_ += world_vy_mps * dt_seconds;
            }

            const double yaw_rate_radps = telemetry.estimated_state.body_pose_state.angular_velocity_radps.z;
            if (isFiniteOrZero(yaw_rate_radps) && isFiniteOrZero(odometry_pose_yaw_rad_)) {
                odometry_pose_yaw_rad_ += yaw_rate_radps * dt_seconds;
            }
        }

        std::ostringstream payload;
        payload << "{\"type\":\"joints\",\"schema_version\":1,"
                << "\"timestamp_ms\":" << (telemetry.timestamp_us.value / 1000ULL) << ","
                << "\"loop_counter\":" << telemetry.status.loop_counter << ","
                << "\"mode\":" << static_cast<int>(telemetry.status.active_mode) << ","
                << "\"bus_ok\":" << (telemetry.status.bus_ok ? "true" : "false") << ","
                << "\"estimator_valid\":" << (telemetry.status.estimator_valid ? "true" : "false") << ","
                << "\"metadata\":{\"runtime\":{\"imu\":{\"reads_enabled\":"
                << (telemetry.imu_reads_enabled ? "true" : "false")
                << ",\"read_mode\":\""
                << (telemetry.imu_reads_enabled ? "enabled" : "disabled")
                << "\"},\"autonomy\":{"
                << "\"enabled\":" << (telemetry.status.autonomy.enabled ? "true" : "false") << ","
                << "\"step_ok\":" << (telemetry.status.autonomy.step_ok ? "true" : "false") << ","
                << "\"blocked\":" << (telemetry.status.autonomy.blocked ? "true" : "false") << ","
                << "\"no_progress\":" << (telemetry.status.autonomy.no_progress ? "true" : "false") << ","
                << "\"recovery_active\":" << (telemetry.status.autonomy.recovery_active ? "true" : "false") << ","
                << "\"motion_allowed\":" << (telemetry.status.autonomy.motion_allowed ? "true" : "false") << ","
                << "\"locomotion_sent\":" << (telemetry.status.autonomy.locomotion_sent ? "true" : "false") << ","
                << "\"mission_loaded\":" << (telemetry.status.autonomy.mission_loaded ? "true" : "false") << ","
                << "\"mission_running\":" << (telemetry.status.autonomy.mission_running ? "true" : "false") << ","
                << "\"mission_state\":\"" << missionStateToString(telemetry.status.autonomy.mission_state) << "\","
                << "\"mission_completed_waypoints\":" << telemetry.status.autonomy.mission_completed_waypoints << ","
                << "\"mission_total_waypoints\":" << telemetry.status.autonomy.mission_total_waypoints
                << "}}},"
                << "\"dynamic_gait\":{"
                << "\"valid\":" << (telemetry.status.dynamic_gait.valid ? "true" : "false") << ","
                << "\"gait_family\":\"" << gaitTypeToString(telemetry.status.dynamic_gait.gait_family) << "\","
                << "\"region\":\"" << dynamicRegionToString(telemetry.status.dynamic_gait.region) << "\","
                << "\"turn_mode\":\"" << turnModeToString(telemetry.status.dynamic_gait.turn_mode) << "\","
                << "\"fallback_stage\":\"" << fallbackStageToString(telemetry.status.dynamic_gait.fallback_stage) << "\","
                << "\"cadence_hz\":" << telemetry.status.dynamic_gait.cadence_hz << ","
                << "\"reach_utilization\":" << telemetry.status.dynamic_gait.reach_utilization << ","
                << "\"body_linear_velocity_mps\":{"
                << "\"x\":" << telemetry.estimated_state.body_pose_state.body_trans_mps.x << ","
                << "\"y\":" << telemetry.estimated_state.body_pose_state.body_trans_mps.y << ","
                << "\"z\":" << telemetry.estimated_state.body_pose_state.body_trans_mps.z
                << "},"
                << "\"body_translation_m\":{"
                << "\"x\":" << telemetry.estimated_state.body_pose_state.body_trans_m.x << ","
                << "\"y\":" << telemetry.estimated_state.body_pose_state.body_trans_m.y << ","
                << "\"z\":" << telemetry.estimated_state.body_pose_state.body_trans_m.z
                << "},"
                << "\"body_orientation_rad\":{"
                << "\"x\":" << telemetry.estimated_state.body_pose_state.orientation_rad.x << ","
                << "\"y\":" << telemetry.estimated_state.body_pose_state.orientation_rad.y << ","
                << "\"z\":" << telemetry.estimated_state.body_pose_state.orientation_rad.z
                << "},"
                << "\"body_angular_velocity_radps\":{"
                << "\"x\":" << telemetry.estimated_state.body_pose_state.angular_velocity_radps.x << ","
                << "\"y\":" << telemetry.estimated_state.body_pose_state.angular_velocity_radps.y << ","
                << "\"z\":" << telemetry.estimated_state.body_pose_state.angular_velocity_radps.z
                << "},"
                << "\"commanded_body_velocity_mps\":{"
                << "\"x\":" << telemetry.motion_intent.body_pose_setpoint.body_trans_mps.x << ","
                << "\"y\":" << telemetry.motion_intent.body_pose_setpoint.body_trans_mps.y << ","
                << "\"z\":" << telemetry.motion_intent.body_pose_setpoint.body_trans_mps.z
                << "},"
                << "\"commanded_body_angular_velocity_radps\":{"
                << "\"x\":" << telemetry.motion_intent.body_pose_setpoint.angular_velocity_radps.x << ","
                << "\"y\":" << telemetry.motion_intent.body_pose_setpoint.angular_velocity_radps.y << ","
                << "\"z\":" << telemetry.motion_intent.body_pose_setpoint.angular_velocity_radps.z
                << "},"
                << "\"commanded_speed_mps\":" << telemetry.motion_intent.speed_mps.value << ","
                << "\"commanded_heading_rad\":" << telemetry.motion_intent.heading_rad.value << ","
                << "\"suppress_stride_progression\":"
                << (telemetry.status.dynamic_gait.suppress_stride_progression ? "true" : "false") << ","
                << "\"suppress_turning\":"
                << (telemetry.status.dynamic_gait.suppress_turning ? "true" : "false") << ","
                << "\"prioritize_stability\":"
                << (telemetry.status.dynamic_gait.prioritize_stability ? "true" : "false") << ","
                << "\"limiter\":{"
                << "\"enabled\":" << (telemetry.status.dynamic_gait.limiter_enabled ? "true" : "false") << ","
                << "\"phase\":\"" << limiterPhaseToString(telemetry.status.dynamic_gait.limiter_phase) << "\","
                << "\"active_constraint_reason\":\""
                << limiterConstraintReasonToString(telemetry.status.dynamic_gait.active_constraint_reason) << "\","
                << "\"adaptation_scales\":{"
                << "\"linear\":" << telemetry.status.dynamic_gait.adaptation_scale_linear << ","
                << "\"yaw\":" << telemetry.status.dynamic_gait.adaptation_scale_yaw << ","
                << "\"cadence\":" << telemetry.status.dynamic_gait.adaptation_scale_cadence << ","
                << "\"step\":" << telemetry.status.dynamic_gait.adaptation_scale_step
                << "},"
                << "\"hard_clamp\":{"
                << "\"linear\":" << (telemetry.status.dynamic_gait.hard_clamp_linear ? "true" : "false") << ","
                << "\"yaw\":" << (telemetry.status.dynamic_gait.hard_clamp_yaw ? "true" : "false") << ","
                << "\"reach\":" << (telemetry.status.dynamic_gait.hard_clamp_reach ? "true" : "false") << ","
                << "\"cadence\":" << (telemetry.status.dynamic_gait.hard_clamp_cadence ? "true" : "false") << ","
                << "\"saturated\":" << (telemetry.status.dynamic_gait.saturated ? "true" : "false")
                << "}"
                << "},"
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
                << "]},";

        payload << "\"autonomy_debug\":{";
        if (telemetry.autonomy_debug.localization_pose.has_value()) {
            const auto& pose = *telemetry.autonomy_debug.localization_pose;
            payload << "\"current_pose\":{"
                    << "\"x_m\":" << pose.x_m << ","
                    << "\"y_m\":" << pose.y_m << ","
                    << "\"yaw_rad\":" << pose.yaw_rad
                    << "},"
                    << "\"localization\":{"
                    << "\"frame_id\":\"" << pose.frame_id << "\","
                    << "\"current_pose\":{"
                    << "\"x_m\":" << pose.x_m << ","
                    << "\"y_m\":" << pose.y_m << ","
                    << "\"yaw_rad\":" << pose.yaw_rad
                    << "}}";
        } else {
            payload << "\"current_pose\":{"
                    << "\"x_m\":" << odometry_pose_x_m_ << ","
                    << "\"y_m\":" << odometry_pose_y_m_ << ","
                    << "\"yaw_rad\":" << odometry_pose_yaw_rad_
                    << "},"
                    << "\"localization\":{"
                    << "\"frame_id\":\"odom\","
                    << "\"current_pose\":{"
                    << "\"x_m\":" << odometry_pose_x_m_ << ","
                    << "\"y_m\":" << odometry_pose_y_m_ << ","
                    << "\"yaw_rad\":" << odometry_pose_yaw_rad_
                    << "}}";
        }
        payload << "},"
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
    void resetFallbackOdometry(const double timestamp_us) {
        odometry_pose_x_m_ = 0.0;
        odometry_pose_y_m_ = 0.0;
        odometry_pose_yaw_rad_ = 0.0;
        last_odometry_timestamp_us_ = timestamp_us;
    }

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
    double odometry_pose_x_m_{0.0};
    double odometry_pose_y_m_{0.0};
    double odometry_pose_yaw_rad_{0.0};
    double last_odometry_timestamp_us_{0.0};
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
