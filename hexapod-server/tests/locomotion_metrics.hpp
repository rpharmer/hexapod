#pragma once

#include "command_governor.hpp"
#include "locomotion_debug.hpp"
#include "locomotion_feasibility.hpp"
#include "scenario_driver.hpp"
#include "types.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace locomotion_test {

inline std::string jsonEscape(const std::string& value) {
    std::string escaped;
    escaped.reserve(value.size() + 8);
    for (const char ch : value) {
        switch (ch) {
        case '\\':
        case '"':
            escaped.push_back('\\');
            escaped.push_back(ch);
            break;
        case '\n':
            escaped += "\\n";
            break;
        case '\r':
            escaped += "\\r";
            break;
        case '\t':
            escaped += "\\t";
            break;
        default:
            escaped.push_back(ch);
            break;
        }
    }
    return escaped;
}

inline std::string formatDouble(const double value, const int precision = 6) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(precision) << value;
    std::string text = out.str();
    const auto trailing = text.find_last_not_of('0');
    if (trailing != std::string::npos && text[trailing] == '.') {
        text.erase(trailing);
    } else if (trailing != std::string::npos) {
        text.erase(trailing + 1);
    }
    if (text.empty()) {
        return "0";
    }
    return text;
}

inline std::string robotModeName(const RobotMode mode) {
    switch (mode) {
    case RobotMode::SAFE_IDLE:
        return "SAFE_IDLE";
    case RobotMode::HOMING:
        return "HOMING";
    case RobotMode::STAND:
        return "STAND";
    case RobotMode::WALK:
        return "WALK";
    case RobotMode::FAULT:
        return "FAULT";
    }
    return "UNKNOWN";
}

inline std::string faultName(const FaultCode fault) {
    switch (fault) {
    case FaultCode::NONE:
        return "NONE";
    case FaultCode::BUS_TIMEOUT:
        return "BUS_TIMEOUT";
    case FaultCode::ESTOP:
        return "ESTOP";
    case FaultCode::TIP_OVER:
        return "TIP_OVER";
    case FaultCode::ESTIMATOR_INVALID:
        return "ESTIMATOR_INVALID";
    case FaultCode::MOTOR_FAULT:
        return "MOTOR_FAULT";
    case FaultCode::JOINT_LIMIT:
        return "JOINT_LIMIT";
    case FaultCode::COMMAND_TIMEOUT:
        return "COMMAND_TIMEOUT";
    case FaultCode::BODY_COLLAPSE:
        return "BODY_COLLAPSE";
    }
    return "UNKNOWN";
}

inline double wrapAngleDiff(const double start, const double end) {
    return std::atan2(std::sin(end - start), std::cos(end - start));
}

struct MotionSample {
    std::size_t step_index{0};
    std::size_t phase_index{0};
    std::string phase_label{};
    ScenarioMotionIntent requested_motion{};
    ControlStatus status{};
    RobotState estimated{};
    GaitState gait{};
    CommandGovernorState governor{};
    LocomotionFeasibility locomotion_feasibility{};
    SafetyState safety{};
    Vec3 position{};
    double horizontal_speed_mps{0.0};
    double body_tilt_rad{0.0};
    double body_rate_radps{0.0};
    double yaw_rate_radps{0.0};
    double support_margin_m{0.0};
    double stride_phase_rate_hz{0.0};
    double step_length_m{0.0};
    double swing_height_m{0.0};
    double duty_factor{0.0};
    double model_trust{1.0};
    double contact_mismatch_ratio{0.0};
    telemetry::LocomotionDebugSnapshot locomotion_debug{};
    double sample_period_s{0.0};
};

struct ModeSegment {
    RobotMode mode{RobotMode::SAFE_IDLE};
    std::size_t start_step{0};
    std::size_t end_step{0};
};

struct GaitTransitionSegment {
    std::string phase_label{};
    std::size_t start_step{0};
    std::size_t end_step{0};
    double mean_step_length_m{0.0};
    double mean_duty_factor{0.0};
};

struct LocomotionMetrics {
    double sample_period_s{0.0};
    std::size_t sample_count{0};
    std::size_t walk_sample_count{0};
    std::size_t stride_count{0};
    std::size_t mode_transition_count{0};
    std::size_t fault_transition_count{0};
    std::size_t first_fault_step{0};
    bool saw_fault{false};
    FaultCode first_fault{FaultCode::NONE};
    RobotMode final_mode{RobotMode::SAFE_IDLE};
    FaultCode final_fault{FaultCode::NONE};
    double duration_s{0.0};
    double path_length_m{0.0};
    double net_displacement_m{0.0};
    double lateral_deviation_m{0.0};
    double max_abs_roll_rad{0.0};
    double max_abs_pitch_rad{0.0};
    double max_body_rate_radps{0.0};
    double mean_horizontal_speed_mps{0.0};
    double peak_horizontal_speed_mps{0.0};
    double mean_yaw_rate_radps{0.0};
    double peak_yaw_rate_radps{0.0};
    double yaw_delta_rad{0.0};
    double min_support_margin_m{std::numeric_limits<double>::infinity()};
    double min_model_trust{1.0};
    double max_contact_mismatch_ratio{0.0};
    double min_command_scale{1.0};
    double min_cadence_scale{1.0};
    double max_governor_severity{0.0};
    std::size_t recovery_hold_activation_count{0};
    bool saw_recovery_hold{false};
    bool saw_recovery_settling{false};
    std::size_t recovery_settling_transition_count{0};
    double max_governed_speed_mps{0.0};
    double max_governed_yaw_rate_radps{0.0};
    double min_governed_speed_mps{1e9};
    double min_governed_yaw_rate_radps{1e9};
    double max_step_length_m{0.0};
    double min_step_length_m{1e9};
    double max_swing_height_m{0.0};
    double min_swing_height_m{1e9};
    double max_contact_anchor_drift_m{0.0};
    double max_contact_anchor_max_drift_m{0.0};
    double min_measured_foot_world_z_m{std::numeric_limits<double>::infinity()};
    double max_commanded_tracking_error_m{0.0};
    double max_contact_tracking_error_m{0.0};
    std::vector<ModeSegment> mode_segments{};
    std::vector<GaitTransitionSegment> gait_segments{};
};

inline std::string modeSegmentJson(const ModeSegment& segment) {
    std::ostringstream out;
    out << "{\"mode\":\"" << robotModeName(segment.mode) << "\","
        << "\"start_step\":" << segment.start_step << ','
        << "\"end_step\":" << segment.end_step << '}';
    return out.str();
}

inline std::string metricsToJson(const LocomotionMetrics& metrics) {
    std::ostringstream out;
    out << '{'
        << "\"sample_count\":" << metrics.sample_count << ','
        << "\"walk_sample_count\":" << metrics.walk_sample_count << ','
        << "\"stride_count\":" << metrics.stride_count << ','
        << "\"mode_transition_count\":" << metrics.mode_transition_count << ','
        << "\"fault_transition_count\":" << metrics.fault_transition_count << ','
        << "\"saw_fault\":" << (metrics.saw_fault ? "true" : "false") << ','
        << "\"first_fault\":\"" << faultName(metrics.first_fault) << "\","
        << "\"first_fault_step\":" << metrics.first_fault_step << ','
        << "\"final_mode\":\"" << robotModeName(metrics.final_mode) << "\","
        << "\"final_fault\":\"" << faultName(metrics.final_fault) << "\","
        << "\"duration_s\":" << formatDouble(metrics.duration_s) << ','
        << "\"sample_period_s\":" << formatDouble(metrics.sample_period_s) << ','
        << "\"path_length_m\":" << formatDouble(metrics.path_length_m) << ','
        << "\"net_displacement_m\":" << formatDouble(metrics.net_displacement_m) << ','
        << "\"lateral_deviation_m\":" << formatDouble(metrics.lateral_deviation_m) << ','
        << "\"max_abs_roll_rad\":" << formatDouble(metrics.max_abs_roll_rad) << ','
        << "\"max_abs_pitch_rad\":" << formatDouble(metrics.max_abs_pitch_rad) << ','
        << "\"max_body_rate_radps\":" << formatDouble(metrics.max_body_rate_radps) << ','
        << "\"mean_horizontal_speed_mps\":" << formatDouble(metrics.mean_horizontal_speed_mps) << ','
        << "\"peak_horizontal_speed_mps\":" << formatDouble(metrics.peak_horizontal_speed_mps) << ','
        << "\"mean_yaw_rate_radps\":" << formatDouble(metrics.mean_yaw_rate_radps) << ','
        << "\"peak_yaw_rate_radps\":" << formatDouble(metrics.peak_yaw_rate_radps) << ','
        << "\"yaw_delta_rad\":" << formatDouble(metrics.yaw_delta_rad) << ','
        << "\"min_support_margin_m\":" << formatDouble(metrics.min_support_margin_m) << ','
        << "\"min_model_trust\":" << formatDouble(metrics.min_model_trust) << ','
        << "\"max_contact_mismatch_ratio\":" << formatDouble(metrics.max_contact_mismatch_ratio) << ','
        << "\"min_command_scale\":" << formatDouble(metrics.min_command_scale) << ','
        << "\"min_cadence_scale\":" << formatDouble(metrics.min_cadence_scale) << ','
        << "\"max_governor_severity\":" << formatDouble(metrics.max_governor_severity) << ','
        << "\"recovery_hold_activation_count\":" << metrics.recovery_hold_activation_count << ','
        << "\"saw_recovery_hold\":" << (metrics.saw_recovery_hold ? "true" : "false") << ','
        << "\"saw_recovery_settling\":" << (metrics.saw_recovery_settling ? "true" : "false") << ','
        << "\"recovery_settling_transition_count\":" << metrics.recovery_settling_transition_count << ','
        << "\"max_governed_speed_mps\":" << formatDouble(metrics.max_governed_speed_mps) << ','
        << "\"max_governed_yaw_rate_radps\":" << formatDouble(metrics.max_governed_yaw_rate_radps) << ','
        << "\"min_governed_speed_mps\":" << formatDouble(metrics.min_governed_speed_mps) << ','
        << "\"min_governed_yaw_rate_radps\":" << formatDouble(metrics.min_governed_yaw_rate_radps) << ','
        << "\"max_step_length_m\":" << formatDouble(metrics.max_step_length_m) << ','
        << "\"min_step_length_m\":" << formatDouble(metrics.min_step_length_m) << ','
        << "\"max_swing_height_m\":" << formatDouble(metrics.max_swing_height_m) << ','
        << "\"min_swing_height_m\":" << formatDouble(metrics.min_swing_height_m) << ','
        << "\"max_contact_anchor_drift_m\":" << formatDouble(metrics.max_contact_anchor_drift_m) << ','
        << "\"max_contact_anchor_max_drift_m\":" << formatDouble(metrics.max_contact_anchor_max_drift_m) << ','
        << "\"min_measured_foot_world_z_m\":" << formatDouble(metrics.min_measured_foot_world_z_m) << ','
        << "\"max_commanded_tracking_error_m\":" << formatDouble(metrics.max_commanded_tracking_error_m) << ','
        << "\"max_contact_tracking_error_m\":" << formatDouble(metrics.max_contact_tracking_error_m) << ','
        << "\"mode_segments\":[";
    for (std::size_t i = 0; i < metrics.mode_segments.size(); ++i) {
        if (i > 0) {
            out << ',';
        }
        out << modeSegmentJson(metrics.mode_segments[i]);
    }
    out << "],\"gait_segments\":[";
    for (std::size_t i = 0; i < metrics.gait_segments.size(); ++i) {
        if (i > 0) {
            out << ',';
        }
        const auto& segment = metrics.gait_segments[i];
        out << "{\"phase_label\":\"" << jsonEscape(segment.phase_label) << "\","
            << "\"start_step\":" << segment.start_step << ','
            << "\"end_step\":" << segment.end_step << ','
            << "\"mean_step_length_m\":" << formatDouble(segment.mean_step_length_m) << ','
            << "\"mean_duty_factor\":" << formatDouble(segment.mean_duty_factor) << '}';
    }
    out << "]}";
    return out.str();
}

inline void appendSample(LocomotionMetrics& metrics,
                         std::vector<MotionSample>& samples,
                         const MotionSample& sample) {
    samples.push_back(sample);
    metrics.sample_count = samples.size();
    metrics.final_mode = sample.status.active_mode;
    metrics.final_fault = sample.status.active_fault;
    metrics.walk_sample_count += sample.status.active_mode == RobotMode::WALK ? 1 : 0;
    metrics.max_abs_roll_rad = std::max(metrics.max_abs_roll_rad, std::abs(sample.estimated.body_twist_state.twist_pos_rad.x));
    metrics.max_abs_pitch_rad = std::max(metrics.max_abs_pitch_rad, std::abs(sample.estimated.body_twist_state.twist_pos_rad.y));
    metrics.max_body_rate_radps = std::max(metrics.max_body_rate_radps, sample.body_rate_radps);
    metrics.max_governor_severity = std::max(metrics.max_governor_severity, sample.governor.severity);
    metrics.saw_recovery_hold = metrics.saw_recovery_hold || sample.governor.recovery_hold_active;
    if (sample.governor.recovery_hold_active &&
        (samples.size() == 1 || !samples[samples.size() - 2].governor.recovery_hold_active)) {
        ++metrics.recovery_hold_activation_count;
    }
    metrics.saw_recovery_settling =
        metrics.saw_recovery_settling || sample.governor.recovery_stage == RecoveryStage::Settling;
    if (sample.governor.recovery_stage == RecoveryStage::Settling &&
        (samples.size() == 1 || samples[samples.size() - 2].governor.recovery_stage != RecoveryStage::Settling)) {
        ++metrics.recovery_settling_transition_count;
    }
    metrics.min_support_margin_m = std::min(metrics.min_support_margin_m, sample.support_margin_m);
    metrics.min_model_trust = std::min(metrics.min_model_trust, sample.model_trust);
    metrics.max_contact_mismatch_ratio = std::max(metrics.max_contact_mismatch_ratio, sample.contact_mismatch_ratio);
    metrics.min_command_scale = std::min(metrics.min_command_scale, sample.governor.command_scale);
    metrics.min_cadence_scale = std::min(metrics.min_cadence_scale, sample.governor.cadence_scale);
    metrics.max_governed_speed_mps = std::max(metrics.max_governed_speed_mps, sample.governor.governed_planar_speed_mps);
    metrics.max_governed_yaw_rate_radps = std::max(metrics.max_governed_yaw_rate_radps, sample.governor.governed_yaw_rate_radps);
    metrics.min_governed_speed_mps = std::min(metrics.min_governed_speed_mps, sample.governor.governed_planar_speed_mps);
    metrics.min_governed_yaw_rate_radps = std::min(metrics.min_governed_yaw_rate_radps, sample.governor.governed_yaw_rate_radps);
    metrics.max_step_length_m = std::max(metrics.max_step_length_m, sample.step_length_m);
    metrics.min_step_length_m = std::min(metrics.min_step_length_m, sample.step_length_m);
    metrics.max_swing_height_m = std::max(metrics.max_swing_height_m, sample.swing_height_m);
    metrics.min_swing_height_m = std::min(metrics.min_swing_height_m, sample.swing_height_m);
    metrics.path_length_m += sample.horizontal_speed_mps * metrics.sample_period_s;
    metrics.mean_horizontal_speed_mps += sample.horizontal_speed_mps;
    metrics.mean_yaw_rate_radps += std::abs(sample.yaw_rate_radps);
    metrics.peak_horizontal_speed_mps = std::max(metrics.peak_horizontal_speed_mps, sample.horizontal_speed_mps);
    metrics.peak_yaw_rate_radps = std::max(metrics.peak_yaw_rate_radps, std::abs(sample.yaw_rate_radps));
    metrics.yaw_delta_rad = wrapAngleDiff(samples.front().estimated.body_twist_state.twist_pos_rad.z,
                                          sample.estimated.body_twist_state.twist_pos_rad.z);
    if (sample.locomotion_debug.valid) {
        metrics.min_measured_foot_world_z_m = std::min(
            metrics.min_measured_foot_world_z_m, sample.locomotion_debug.min_measured_foot_world_z_m);
        metrics.max_commanded_tracking_error_m = std::max(
            metrics.max_commanded_tracking_error_m, sample.locomotion_debug.max_commanded_tracking_error_m);
        for (int leg = 0; leg < kNumLegs; ++leg) {
            const std::size_t leg_index = static_cast<std::size_t>(leg);
            metrics.max_contact_anchor_drift_m = std::max(
                metrics.max_contact_anchor_drift_m,
                sample.locomotion_debug.contact_anchor_drift_m[leg_index]);
            metrics.max_contact_anchor_max_drift_m = std::max(
                metrics.max_contact_anchor_max_drift_m,
                sample.locomotion_debug.contact_anchor_max_drift_m[leg_index]);
            if (sample.locomotion_debug.contact_anchor_valid[leg_index]) {
                metrics.max_contact_tracking_error_m = std::max(
                    metrics.max_contact_tracking_error_m,
                    sample.locomotion_debug.commanded_tracking_error_m[leg_index]);
            }
        }
    }

    if (sample.status.active_fault != FaultCode::NONE && !metrics.saw_fault) {
        metrics.saw_fault = true;
        metrics.first_fault = sample.status.active_fault;
        metrics.first_fault_step = sample.step_index;
    }
}

inline void finalizeMetrics(LocomotionMetrics& metrics, const std::vector<MotionSample>& samples) {
    if (samples.empty()) {
        metrics.min_support_margin_m = 0.0;
        metrics.min_model_trust = 0.0;
        metrics.min_command_scale = 0.0;
        metrics.min_cadence_scale = 0.0;
        metrics.min_governed_speed_mps = 0.0;
        metrics.min_governed_yaw_rate_radps = 0.0;
        metrics.min_step_length_m = 0.0;
        metrics.min_swing_height_m = 0.0;
        metrics.min_measured_foot_world_z_m = 0.0;
        return;
    }

    metrics.duration_s = static_cast<double>(samples.size()) * metrics.sample_period_s;
    metrics.mean_horizontal_speed_mps /= static_cast<double>(samples.size());
    metrics.mean_yaw_rate_radps /= static_cast<double>(samples.size());
    metrics.net_displacement_m = std::hypot(
        samples.back().position.x - samples.front().position.x,
        samples.back().position.y - samples.front().position.y);
    metrics.yaw_delta_rad = wrapAngleDiff(samples.front().estimated.body_twist_state.twist_pos_rad.z,
                                          samples.back().estimated.body_twist_state.twist_pos_rad.z);

    const Vec3 travel{
        samples.back().position.x - samples.front().position.x,
        samples.back().position.y - samples.front().position.y,
        0.0};
    const double denom = std::hypot(travel.x, travel.y);
    if (denom > 1.0e-9) {
        for (const MotionSample& sample : samples) {
            const Vec3 offset{
                sample.position.x - samples.front().position.x,
                sample.position.y - samples.front().position.y,
                0.0};
            const double deviation = std::abs(travel.x * offset.y - travel.y * offset.x) / denom;
            metrics.lateral_deviation_m = std::max(metrics.lateral_deviation_m, deviation);
        }
    }

    metrics.mode_segments.clear();
    if (!samples.empty()) {
        ModeSegment current{samples.front().status.active_mode, samples.front().step_index, samples.front().step_index};
        for (std::size_t i = 1; i < samples.size(); ++i) {
            if (samples[i].status.active_mode != current.mode) {
                current.end_step = samples[i - 1].step_index;
                metrics.mode_segments.push_back(current);
                ++metrics.mode_transition_count;
                if (samples[i].status.active_fault != samples[i - 1].status.active_fault) {
                    ++metrics.fault_transition_count;
                }
                current = ModeSegment{samples[i].status.active_mode, samples[i].step_index, samples[i].step_index};
            } else if (samples[i].status.active_fault != samples[i - 1].status.active_fault) {
                ++metrics.fault_transition_count;
            }
        }
        current.end_step = samples.back().step_index;
        metrics.mode_segments.push_back(current);
    }

    metrics.gait_segments.clear();
    if (!samples.empty()) {
        GaitTransitionSegment current{
            samples.front().phase_label,
            samples.front().step_index,
            samples.front().step_index,
            samples.front().step_length_m,
            samples.front().duty_factor,
        };
        std::size_t gait_segment_samples = 1;
        for (std::size_t i = 1; i < samples.size(); ++i) {
            const MotionSample& prev = samples[i - 1];
            const MotionSample& sample = samples[i];
            if (sample.phase_label != prev.phase_label) {
                current.end_step = samples[i - 1].step_index;
                current.mean_step_length_m /= static_cast<double>(gait_segment_samples);
                current.mean_duty_factor /= static_cast<double>(gait_segment_samples);
                metrics.gait_segments.push_back(current);
                current = GaitTransitionSegment{sample.phase_label, sample.step_index, sample.step_index, sample.step_length_m, sample.duty_factor};
                gait_segment_samples = 1;
            } else {
                current.mean_step_length_m += sample.step_length_m;
                current.mean_duty_factor += sample.duty_factor;
                ++gait_segment_samples;
                current.end_step = sample.step_index;
            }
        }
        current.mean_step_length_m /= static_cast<double>(gait_segment_samples);
        current.mean_duty_factor /= static_cast<double>(gait_segment_samples);
        metrics.gait_segments.push_back(current);
    }

    if (metrics.min_support_margin_m == std::numeric_limits<double>::infinity()) {
        metrics.min_support_margin_m = 0.0;
    }
    if (metrics.min_measured_foot_world_z_m == std::numeric_limits<double>::infinity()) {
        metrics.min_measured_foot_world_z_m = 0.0;
    }
}

} // namespace locomotion_test
