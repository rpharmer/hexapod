#include "control_config.hpp"
#include "motion_intent_utils.hpp"
#include "physics_sim_test_utils.hpp"
#include "physics_sim_bridge.hpp"
#include "physics_sim_estimator.hpp"
#include "robot_runtime.hpp"
#include "scenario_driver.hpp"
#include "locomotion_metrics.hpp"
#include "physics_sim_metrics_emit.hpp"
#include "physics_sim_test_argv.hpp"
#include "test_limits_manifest.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#if defined(__linux__)
#include <csignal>
#include <sys/wait.h>
#include <unistd.h>
#endif

namespace {

constexpr const char* kWalkDistanceSuite = "physics_sim_walk_distance";

bool expect(bool condition, const std::string& message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

Vec3 positionFromState(const RobotState& state) {
    return Vec3{state.body_twist_state.body_trans_m.x,
                state.body_twist_state.body_trans_m.y,
                state.body_twist_state.body_trans_m.z};
}

class CapturingPhysicsSimBridge final : public IHardwareBridge {
public:
    CapturingPhysicsSimBridge(std::string host,
                              int port,
                              int bus_loop_period_us,
                              int physics_solver_iterations)
        : inner_(std::move(host), port, bus_loop_period_us, physics_solver_iterations, nullptr) {}

    bool init() override {
        return inner_.init();
    }

    bool read(RobotState& out) override {
        const bool ok = inner_.read(out);
        if (ok) {
            last_state_ = out;
        }
        return ok;
    }

    bool write(const JointTargets& in) override {
        return inner_.write(in);
    }

    std::optional<BridgeCommandResultMetadata> last_bridge_result() const override {
        return inner_.last_bridge_result();
    }

    const std::optional<RobotState>& last_state() const {
        return last_state_;
    }

private:
    PhysicsSimBridge inner_;
    std::optional<RobotState> last_state_{};
};

void runControlLoopStep(RobotRuntime& runtime, const ScenarioMotionIntent& motion) {
    runtime.setMotionIntent(makeMotionIntent(motion));
    runtime.busStep();
    runtime.estimatorStep();
    runtime.safetyStep();
    runtime.controlStep();
}

void runControlLoopStep(RobotRuntime& runtime, const MotionIntent& motion) {
    MotionIntent fresh_motion = motion;
    // This test reuses some handcrafted MotionIntent instances across many loop iterations.
    // Clear transport freshness fields so RobotRuntime stamps a fresh command sample each step.
    fresh_motion.timestamp_us = TimePointUs{};
    fresh_motion.sample_id = 0;
    runtime.setMotionIntent(fresh_motion);
    runtime.busStep();
    runtime.estimatorStep();
    runtime.safetyStep();
    runtime.controlStep();
}

struct MotionRunResult {
    Vec3 start_position{};
    Vec3 end_position{};
    double start_yaw_rad{0.0};
    double end_yaw_rad{0.0};
    double walk_path_length_m{0.0};
    double max_lateral_deviation_m{0.0};
    double average_horizontal_speed_mps{0.0};
    double peak_horizontal_speed_mps{0.0};
    double average_yaw_rate_radps{0.0};
    double peak_yaw_rate_radps{0.0};
    int walk_mode_steps{0};
    int non_walk_mode_steps{0};
    int faulted_steps{0};
    ControlStatus final_status{};
};

double wrapAngleDiff(double start, double end) {
    return std::atan2(std::sin(end - start), std::cos(end - start));
}

std::string walkDistanceLimitsWalkEnvelopeJsonDynamic(const std::string& label,
                                                      const double min_path_length_m,
                                                      const double min_peak_horizontal_speed_mps,
                                                      const double min_average_speed_ratio,
                                                      const double max_average_speed_ratio,
                                                      const bool require_active_mode_walk) {
    using locomotion_test::formatDouble;
    std::ostringstream o;
    o << "{\"min_path_length_m\":" << formatDouble(min_path_length_m)
      << ",\"min_peak_horizontal_speed_mps\":" << formatDouble(min_peak_horizontal_speed_mps)
      << ",\"min_average_speed_ratio\":" << formatDouble(min_average_speed_ratio)
      << ",\"max_average_speed_ratio\":" << formatDouble(max_average_speed_ratio)
      << ",\"require_active_mode_walk\":" << (require_active_mode_walk ? "true" : "false") << '}';
    (void)label;
    return o.str();
}

std::string walkDistanceLimitsStraightJsonDynamic(const std::string& label,
                                                  const double min_path_length_m,
                                                  const double max_lateral_deviation_m,
                                                  const double max_lateral_vs_path_ratio,
                                                  const double min_peak_horizontal_speed_mps,
                                                  const double min_average_speed_ratio,
                                                  const double max_average_speed_ratio,
                                                  const bool require_active_mode_walk) {
    using locomotion_test::formatDouble;
    std::ostringstream o;
    o << "{\"min_path_length_m\":" << formatDouble(min_path_length_m)
      << ",\"max_lateral_deviation_m\":" << formatDouble(max_lateral_deviation_m)
      << ",\"max_lateral_vs_path_ratio\":" << formatDouble(max_lateral_vs_path_ratio)
      << ",\"min_peak_horizontal_speed_mps\":" << formatDouble(min_peak_horizontal_speed_mps)
      << ",\"min_average_speed_ratio\":" << formatDouble(min_average_speed_ratio)
      << ",\"max_average_speed_ratio\":" << formatDouble(max_average_speed_ratio)
      << ",\"require_active_mode_walk\":" << (require_active_mode_walk ? "true" : "false") << '}';
    (void)label;
    return o.str();
}

std::string walkDistanceLimitsTurnJsonDynamic(const std::string& label,
                                              const double max_path_length_m,
                                              const double max_net_horizontal_distance_m,
                                              const double min_peak_yaw_rate_radps,
                                              const double min_average_yaw_rate_ratio,
                                              const double max_average_yaw_rate_ratio,
                                              const double min_yaw_delta_abs_rad,
                                              const bool require_active_mode_walk) {
    using locomotion_test::formatDouble;
    std::ostringstream o;
    o << "{\"max_path_length_m\":" << formatDouble(max_path_length_m)
      << ",\"max_net_horizontal_distance_m\":" << formatDouble(max_net_horizontal_distance_m)
      << ",\"min_peak_yaw_rate_radps\":" << formatDouble(min_peak_yaw_rate_radps)
      << ",\"min_average_yaw_rate_ratio\":" << formatDouble(min_average_yaw_rate_ratio)
      << ",\"max_average_yaw_rate_ratio\":" << formatDouble(max_average_yaw_rate_ratio)
      << ",\"min_yaw_delta_abs_rad\":" << formatDouble(min_yaw_delta_abs_rad)
      << ",\"require_active_mode_walk\":" << (require_active_mode_walk ? "true" : "false") << '}';
    (void)label;
    return o.str();
}

std::string motionRunResultMetricsJson(const MotionRunResult& result,
                                       const Vec3& delta,
                                       const double horizontal_distance,
                                       const double commanded_speed,
                                       const double average_ratio) {
    std::ostringstream o;
    using locomotion_test::formatDouble;
    o << "{\"walk_path_length_m\":" << formatDouble(result.walk_path_length_m)
      << ",\"net_horizontal_distance_m\":" << formatDouble(horizontal_distance)
      << ",\"delta_x_m\":" << formatDouble(delta.x) << ",\"delta_y_m\":" << formatDouble(delta.y)
      << ",\"delta_z_m\":" << formatDouble(delta.z)
      << ",\"max_lateral_deviation_m\":" << formatDouble(result.max_lateral_deviation_m)
      << ",\"average_horizontal_speed_mps\":" << formatDouble(result.average_horizontal_speed_mps)
      << ",\"peak_horizontal_speed_mps\":" << formatDouble(result.peak_horizontal_speed_mps)
      << ",\"average_yaw_rate_radps\":" << formatDouble(result.average_yaw_rate_radps)
      << ",\"peak_yaw_rate_radps\":" << formatDouble(result.peak_yaw_rate_radps)
      << ",\"walk_mode_steps\":" << result.walk_mode_steps
      << ",\"non_walk_mode_steps\":" << result.non_walk_mode_steps
      << ",\"faulted_steps\":" << result.faulted_steps
      << ",\"commanded_speed_mps\":" << formatDouble(commanded_speed)
      << ",\"average_speed_ratio\":" << formatDouble(average_ratio)
      << ",\"start_yaw_rad\":" << formatDouble(result.start_yaw_rad)
      << ",\"end_yaw_rad\":" << formatDouble(result.end_yaw_rad)
      << ",\"final_mode\":" << static_cast<int>(result.final_status.active_mode)
      << ",\"final_fault\":" << static_cast<int>(result.final_status.active_fault) << '}';
    return o.str();
}

std::string motionRunTurnMetricsJson(const MotionRunResult& result,
                                     const Vec3& delta,
                                     const double horizontal_distance,
                                     const double yaw_delta,
                                     const double commanded_yaw_rate,
                                     const double average_ratio) {
    std::ostringstream o;
    using locomotion_test::formatDouble;
    o << "{\"walk_path_length_m\":" << formatDouble(result.walk_path_length_m)
      << ",\"net_horizontal_distance_m\":" << formatDouble(horizontal_distance)
      << ",\"delta_x_m\":" << formatDouble(delta.x) << ",\"delta_y_m\":" << formatDouble(delta.y)
      << ",\"delta_z_m\":" << formatDouble(delta.z)
      << ",\"yaw_delta_rad\":" << formatDouble(yaw_delta)
      << ",\"average_yaw_rate_radps\":" << formatDouble(result.average_yaw_rate_radps)
      << ",\"peak_yaw_rate_radps\":" << formatDouble(result.peak_yaw_rate_radps)
      << ",\"walk_mode_steps\":" << result.walk_mode_steps
      << ",\"non_walk_mode_steps\":" << result.non_walk_mode_steps
      << ",\"faulted_steps\":" << result.faulted_steps
      << ",\"commanded_yaw_rate_radps\":" << formatDouble(commanded_yaw_rate)
      << ",\"average_yaw_rate_ratio\":" << formatDouble(average_ratio)
      << ",\"final_mode\":" << static_cast<int>(result.final_status.active_mode)
      << ",\"final_fault\":" << static_cast<int>(result.final_status.active_fault) << '}';
    return o.str();
}

template <typename MotionT>
MotionRunResult runMotionSequence(RobotRuntime& runtime,
                                  CapturingPhysicsSimBridge& bridge,
                                  const ScenarioMotionIntent& stand_motion,
                                  const MotionT& motion,
                                  const int bus_loop_period_us) {
    const int kStandWarmupSteps = static_cast<int>(
        physics_sim_test_utils::scaledLegacyStepCount(100, bus_loop_period_us));
    const int kWalkSteps = static_cast<int>(
        physics_sim_test_utils::scaledLegacyStepCount(600, bus_loop_period_us));

    for (int i = 0; i < kStandWarmupSteps; ++i) {
        runControlLoopStep(runtime, stand_motion);
    }

    if (!bridge.last_state().has_value()) {
        throw std::runtime_error("bridge never produced an initial state");
    }

    MotionRunResult result{};
    result.start_position = positionFromState(bridge.last_state().value());
    result.start_yaw_rad = bridge.last_state().value().body_twist_state.twist_pos_rad.z;
    Vec3 previous_position = result.start_position;
    double horizontal_speed_sum = 0.0;
    double yaw_speed_sum = 0.0;
    std::vector<Vec3> walk_samples{};
    walk_samples.reserve(static_cast<std::size_t>(kWalkSteps));

    for (int i = 0; i < kWalkSteps; ++i) {
        runControlLoopStep(runtime, motion);

        if (!bridge.last_state().has_value()) {
            throw std::runtime_error("bridge lost state during walk sequence");
        }

        const RobotState& state = bridge.last_state().value();
        const Vec3 current_position = positionFromState(state);
        walk_samples.push_back(current_position);
        result.walk_path_length_m += std::hypot(current_position.x - previous_position.x,
                                                current_position.y - previous_position.y);
        previous_position = current_position;
        const double speed_xy = std::hypot(
            state.body_twist_state.body_trans_mps.x,
            state.body_twist_state.body_trans_mps.y);
        horizontal_speed_sum += speed_xy;
        result.peak_horizontal_speed_mps = std::max(result.peak_horizontal_speed_mps, speed_xy);
        const double yaw_rate = std::abs(state.body_twist_state.twist_vel_radps.z);
        yaw_speed_sum += yaw_rate;
        result.peak_yaw_rate_radps = std::max(result.peak_yaw_rate_radps, yaw_rate);
        result.final_status = runtime.getStatus();
        if (result.final_status.active_mode == RobotMode::WALK) {
            ++result.walk_mode_steps;
        } else {
            ++result.non_walk_mode_steps;
        }
        if (result.final_status.active_fault != FaultCode::NONE) {
            ++result.faulted_steps;
        }
    }

    result.end_position = positionFromState(bridge.last_state().value());
    result.end_yaw_rad = bridge.last_state().value().body_twist_state.twist_pos_rad.z;
    result.average_horizontal_speed_mps = horizontal_speed_sum / static_cast<double>(kWalkSteps);
    result.average_yaw_rate_radps = yaw_speed_sum / static_cast<double>(kWalkSteps);

    const Vec3 travel = result.end_position - result.start_position;
    const double travel_len = std::hypot(travel.x, travel.y);
    if (travel_len > 1e-9) {
        const double denom = travel_len;
        for (const Vec3& sample : walk_samples) {
            const Vec3 offset = sample - result.start_position;
            const double deviation = std::abs(travel.x * offset.y - travel.y * offset.x) / denom;
            result.max_lateral_deviation_m = std::max(result.max_lateral_deviation_m, deviation);
        }
    }

    return result;
}

bool checkWalkCase(const std::string& label,
                   RobotRuntime& runtime,
                   CapturingPhysicsSimBridge& bridge,
                   const ScenarioMotionIntent& stand_motion,
                   const ScenarioMotionIntent& walk_motion,
                   const int bus_loop_period_us,
                   const bool emit_metrics_json) {
    MotionRunResult result{};
    try {
        result = runMotionSequence(runtime, bridge, stand_motion, walk_motion, bus_loop_period_us);
    } catch (const std::exception& ex) {
        if (emit_metrics_json) {
            physics_sim_metrics::emitLine(
                "physics_sim_walk_distance",
                label,
                false,
                walkDistanceLimitsWalkEnvelopeJsonDynamic(label, 0.08, 0.02, 0.20, 0.90, true),
                std::string("{\"exception\":\"") + locomotion_test::jsonEscape(ex.what()) + "\"}");
        }
        return expect(false, label + ": " + ex.what());
    }

    const Vec3 delta = result.end_position - result.start_position;
    const double horizontal_distance = std::hypot(delta.x, delta.y);
    const double commanded_speed = walk_motion.speed_mps;
    const double average_ratio = commanded_speed > 0.0 ? (result.average_horizontal_speed_mps / commanded_speed) : 0.0;

    const double kMinPathLengthM =
        test_limits::getDouble(kWalkDistanceSuite, label, "", "min_path_length_m", 0.08);
    const double kMinPeakHorizontalSpeedMps =
        test_limits::getDouble(kWalkDistanceSuite, label, "", "min_peak_horizontal_speed_mps", 0.02);
    const double kMinAverageSpeedRatio =
        test_limits::getDouble(kWalkDistanceSuite, label, "", "min_average_speed_ratio", 0.20);
    const double kMaxAverageSpeedRatio =
        test_limits::getDouble(kWalkDistanceSuite, label, "", "max_average_speed_ratio", 0.90);
    const bool kRequireActiveModeWalk =
        test_limits::getBool(kWalkDistanceSuite, label, "", "require_active_mode_walk", true);
    const std::string limits_walk_json = walkDistanceLimitsWalkEnvelopeJsonDynamic(
        label, kMinPathLengthM, kMinPeakHorizontalSpeedMps, kMinAverageSpeedRatio, kMaxAverageSpeedRatio, kRequireActiveModeWalk);

    const auto emit = [&](const bool pass) {
        if (!emit_metrics_json) {
            return;
        }
        physics_sim_metrics::emitLine("physics_sim_walk_distance",
                                      label,
                                      pass,
                                      limits_walk_json,
                                      motionRunResultMetricsJson(
                                          result, delta, horizontal_distance, commanded_speed, average_ratio));
    };

    const RobotState fused_snapshot = runtime.estimatedSnapshot();
    if (!expect(fused_snapshot.has_fusion_diagnostics,
                label + ": estimator should publish fusion diagnostics during live sim walking")) {
        emit(false);
        return false;
    }

    if (!expect(result.walk_path_length_m >= kMinPathLengthM,
                label + ": walk should accumulate measurable horizontal travel")) {
        std::cerr << label << " path=" << result.walk_path_length_m
                  << " net_horiz=" << horizontal_distance
                  << " dx=" << delta.x
                  << " dy=" << delta.y
                  << " dz=" << delta.z
                  << " avg_speed=" << result.average_horizontal_speed_mps
                  << " command=" << commanded_speed
                  << " ratio=" << average_ratio
                  << " peak_speed=" << result.peak_horizontal_speed_mps
                  << " mode=" << static_cast<int>(result.final_status.active_mode) << '\n';
        emit(false);
        return false;
    }

    if (!expect(result.average_horizontal_speed_mps >= commanded_speed * kMinAverageSpeedRatio,
                label + ": average projected speed should stay above the lower band")) {
        std::cerr << label << " avg_speed=" << result.average_horizontal_speed_mps
                  << " command=" << commanded_speed
                  << " ratio=" << average_ratio
                  << " mode=" << static_cast<int>(result.final_status.active_mode)
                  << " fault=" << static_cast<int>(result.final_status.active_fault) << '\n';
        emit(false);
        return false;
    }

    if (!expect(result.average_horizontal_speed_mps <= commanded_speed * kMaxAverageSpeedRatio,
                label + ": average projected speed should stay below the upper band")) {
        std::cerr << label << " avg_speed=" << result.average_horizontal_speed_mps
                  << " command=" << commanded_speed
                  << " ratio=" << average_ratio
                  << " mode=" << static_cast<int>(result.final_status.active_mode)
                  << " fault=" << static_cast<int>(result.final_status.active_fault) << '\n';
        emit(false);
        return false;
    }

    if (!expect(result.peak_horizontal_speed_mps >= kMinPeakHorizontalSpeedMps,
                label + ": walk should produce a non-trivial horizontal body speed")) {
        emit(false);
        return false;
    }

    if (!expect(result.non_walk_mode_steps == 0,
                label + ": runtime should remain in WALK mode while the motion command is active")) {
        emit(false);
        return false;
    }
    if (!expect(result.faulted_steps == 0,
                label + ": walk should not trip safety faults during the commanded interval")) {
        emit(false);
        return false;
    }

    std::cout << label << " ok dx=" << delta.x
              << " dy=" << delta.y
              << " dz=" << delta.z
              << " horiz=" << horizontal_distance
              << " path=" << result.walk_path_length_m
              << " avg_speed=" << result.average_horizontal_speed_mps
              << " ratio=" << average_ratio
              << " peak_speed=" << result.peak_horizontal_speed_mps << '\n';
    emit(true);
    return true;
}

bool checkStraightWalkCase(const std::string& label,
                           RobotRuntime& runtime,
                           CapturingPhysicsSimBridge& bridge,
                           const ScenarioMotionIntent& stand_motion,
                           const ScenarioMotionIntent& walk_motion,
                           const int bus_loop_period_us,
                           const bool emit_metrics_json) {
    MotionRunResult result{};
    try {
        result = runMotionSequence(runtime, bridge, stand_motion, walk_motion, bus_loop_period_us);
    } catch (const std::exception& ex) {
        if (emit_metrics_json) {
            physics_sim_metrics::emitLine(
                "physics_sim_walk_distance",
                label,
                false,
                walkDistanceLimitsStraightJsonDynamic(label, 0.08, 0.20, 0.35, 0.02, 0.20, 0.90, true),
                std::string("{\"exception\":\"") + locomotion_test::jsonEscape(ex.what()) + "\"}");
        }
        return expect(false, label + ": " + ex.what());
    }

    const Vec3 delta = result.end_position - result.start_position;
    const double horizontal_distance = std::hypot(delta.x, delta.y);
    const double commanded_speed = walk_motion.speed_mps;
    const double average_ratio = commanded_speed > 0.0 ? (result.average_horizontal_speed_mps / commanded_speed) : 0.0;

    const double kMinPathLengthM =
        test_limits::getDouble(kWalkDistanceSuite, label, "", "min_path_length_m", 0.08);
    const double kMaxLateralDeviationM =
        test_limits::getDouble(kWalkDistanceSuite, label, "", "max_lateral_deviation_m", 0.20);
    const double kMaxLateralVsPathRatio =
        test_limits::getDouble(kWalkDistanceSuite, label, "", "max_lateral_vs_path_ratio", 0.35);
    const double kMinPeakHorizontalSpeedMps =
        test_limits::getDouble(kWalkDistanceSuite, label, "", "min_peak_horizontal_speed_mps", 0.02);
    const double kMinAverageSpeedRatio =
        test_limits::getDouble(kWalkDistanceSuite, label, "", "min_average_speed_ratio", 0.20);
    const double kMaxAverageSpeedRatio =
        test_limits::getDouble(kWalkDistanceSuite, label, "", "max_average_speed_ratio", 0.90);
    const bool kRequireActiveModeWalk =
        test_limits::getBool(kWalkDistanceSuite, label, "", "require_active_mode_walk", true);
    const std::string limits_straight_json = walkDistanceLimitsStraightJsonDynamic(
        label,
        kMinPathLengthM,
        kMaxLateralDeviationM,
        kMaxLateralVsPathRatio,
        kMinPeakHorizontalSpeedMps,
        kMinAverageSpeedRatio,
        kMaxAverageSpeedRatio,
        kRequireActiveModeWalk);

    const auto emit = [&](const bool pass) {
        if (!emit_metrics_json) {
            return;
        }
        physics_sim_metrics::emitLine("physics_sim_walk_distance",
                                      label,
                                      pass,
                                      limits_straight_json,
                                      motionRunResultMetricsJson(
                                          result, delta, horizontal_distance, commanded_speed, average_ratio));
    };

    if (!expect(result.walk_path_length_m >= kMinPathLengthM,
                label + ": straight walk should accumulate measurable horizontal travel")) {
        std::cerr << label << " path=" << result.walk_path_length_m
                  << " lateral_deviation=" << result.max_lateral_deviation_m
                  << " dx=" << delta.x
                  << " dy=" << delta.y
                  << " dz=" << delta.z
                  << " avg_speed=" << result.average_horizontal_speed_mps
                  << " command=" << commanded_speed
                  << " ratio=" << average_ratio
                  << " peak_speed=" << result.peak_horizontal_speed_mps
                  << " mode=" << static_cast<int>(result.final_status.active_mode) << '\n';
        emit(false);
        return false;
    }

    if (!expect(result.max_lateral_deviation_m <= kMaxLateralDeviationM,
                label + ": straight walk should stay close to its start-to-finish path")) {
        std::cerr << label << " path=" << result.walk_path_length_m
                  << " lateral_deviation=" << result.max_lateral_deviation_m
                  << " dx=" << delta.x
                  << " dy=" << delta.y
                  << " dz=" << delta.z
                  << " avg_speed=" << result.average_horizontal_speed_mps
                  << " command=" << commanded_speed
                  << " ratio=" << average_ratio << '\n';
        emit(false);
        return false;
    }

    if (!expect(result.max_lateral_deviation_m <= result.walk_path_length_m * kMaxLateralVsPathRatio,
                label + ": straight walk should not crab away from its travel line")) {
        std::cerr << label << " path=" << result.walk_path_length_m
                  << " lateral_deviation=" << result.max_lateral_deviation_m
                  << " dx=" << delta.x
                  << " dy=" << delta.y
                  << " dz=" << delta.z
                  << " avg_speed=" << result.average_horizontal_speed_mps
                  << " command=" << commanded_speed
                  << " ratio=" << average_ratio << '\n';
        emit(false);
        return false;
    }

    if (!expect(result.average_horizontal_speed_mps >= commanded_speed * kMinAverageSpeedRatio,
                label + ": average projected speed should stay above the lower band")) {
        std::cerr << label << " avg_speed=" << result.average_horizontal_speed_mps
                  << " command=" << commanded_speed
                  << " ratio=" << average_ratio
                  << " mode=" << static_cast<int>(result.final_status.active_mode)
                  << " fault=" << static_cast<int>(result.final_status.active_fault) << '\n';
        emit(false);
        return false;
    }

    if (!expect(result.average_horizontal_speed_mps <= commanded_speed * kMaxAverageSpeedRatio,
                label + ": average projected speed should stay below the upper band")) {
        std::cerr << label << " avg_speed=" << result.average_horizontal_speed_mps
                  << " command=" << commanded_speed
                  << " ratio=" << average_ratio
                  << " mode=" << static_cast<int>(result.final_status.active_mode)
                  << " fault=" << static_cast<int>(result.final_status.active_fault) << '\n';
        emit(false);
        return false;
    }

    if (!expect(result.peak_horizontal_speed_mps >= kMinPeakHorizontalSpeedMps,
                label + ": straight walk should produce a non-trivial horizontal body speed")) {
        emit(false);
        return false;
    }

    if (!expect(result.non_walk_mode_steps == 0,
                label + ": runtime should remain in WALK mode while the straight command is active")) {
        emit(false);
        return false;
    }
    if (!expect(result.faulted_steps == 0,
                label + ": straight walk should not trip safety faults during the commanded interval")) {
        emit(false);
        return false;
    }

    std::cout << label << " ok dx=" << delta.x
              << " dy=" << delta.y
              << " dz=" << delta.z
              << " lateral_deviation=" << result.max_lateral_deviation_m
              << " path=" << result.walk_path_length_m
              << " avg_speed=" << result.average_horizontal_speed_mps
              << " ratio=" << average_ratio
              << " peak_speed=" << result.peak_horizontal_speed_mps << '\n';
    emit(true);
    return true;
}

bool checkTurnCase(const std::string& label,
                   RobotRuntime& runtime,
                   CapturingPhysicsSimBridge& bridge,
                   const ScenarioMotionIntent& stand_motion,
                   const MotionIntent& turn_motion,
                   const int bus_loop_period_us,
                   const bool emit_metrics_json) {
    MotionRunResult result{};
    try {
        result = runMotionSequence(runtime, bridge, stand_motion, turn_motion, bus_loop_period_us);
    } catch (const std::exception& ex) {
        if (emit_metrics_json) {
            physics_sim_metrics::emitLine(
                "physics_sim_walk_distance",
                label,
                false,
                walkDistanceLimitsTurnJsonDynamic(label, 2.25, 0.21, 0.02, 0.05, 2.25, 0.05, true),
                std::string("{\"exception\":\"") + locomotion_test::jsonEscape(ex.what()) + "\"}");
        }
        return expect(false, label + ": " + ex.what());
    }

    const Vec3 delta = result.end_position - result.start_position;
    const double horizontal_distance = std::hypot(delta.x, delta.y);
    const double yaw_delta = wrapAngleDiff(result.start_yaw_rad, result.end_yaw_rad);
    const double commanded_yaw_rate = std::abs(turn_motion.twist.twist_vel_radps.z);
    const double average_ratio =
        commanded_yaw_rate > 0.0 ? (result.average_yaw_rate_radps / commanded_yaw_rate) : 0.0;

    const double kMaxPathLengthM =
        test_limits::getDouble(kWalkDistanceSuite, label, "", "max_path_length_m", 2.25);
    const double kMaxNetHorizontalDistanceM =
        test_limits::getDouble(kWalkDistanceSuite, label, "", "max_net_horizontal_distance_m", 0.21);
    const double kMinPeakYawRateRadps =
        test_limits::getDouble(kWalkDistanceSuite, label, "", "min_peak_yaw_rate_radps", 0.02);
    const double kMinAverageYawRateRatio =
        test_limits::getDouble(kWalkDistanceSuite, label, "", "min_average_yaw_rate_ratio", 0.05);
    const double kMaxAverageYawRateRatio =
        test_limits::getDouble(kWalkDistanceSuite, label, "", "max_average_yaw_rate_ratio", 2.25);
    const double kMinYawDeltaRad =
        test_limits::getDouble(kWalkDistanceSuite, label, "", "min_yaw_delta_abs_rad", 0.05);
    const bool kRequireActiveModeWalkTurn =
        test_limits::getBool(kWalkDistanceSuite, label, "", "require_active_mode_walk", true);
    const std::string limits_turn_json = walkDistanceLimitsTurnJsonDynamic(label,
                                                                           kMaxPathLengthM,
                                                                           kMaxNetHorizontalDistanceM,
                                                                           kMinPeakYawRateRadps,
                                                                           kMinAverageYawRateRatio,
                                                                           kMaxAverageYawRateRatio,
                                                                           kMinYawDeltaRad,
                                                                           kRequireActiveModeWalkTurn);

    const auto emit = [&](const bool pass) {
        if (!emit_metrics_json) {
            return;
        }
        physics_sim_metrics::emitLine("physics_sim_walk_distance",
                                      label,
                                      pass,
                                      limits_turn_json,
                                      motionRunTurnMetricsJson(
                                          result, delta, horizontal_distance, yaw_delta, commanded_yaw_rate, average_ratio));
    };

    if (!expect(horizontal_distance <= kMaxNetHorizontalDistanceM,
                label + ": turn-in-place should keep net horizontal drift bounded")) {
        std::cerr << label << " path=" << result.walk_path_length_m
                  << " net_horiz=" << horizontal_distance
                  << " dx=" << delta.x
                  << " dy=" << delta.y
                  << " dz=" << delta.z
                  << " yaw_delta=" << yaw_delta
                  << " avg_yaw_rate=" << result.average_yaw_rate_radps
                  << " command_yaw_rate=" << commanded_yaw_rate
                  << " ratio=" << average_ratio
                  << " peak_yaw_rate=" << result.peak_yaw_rate_radps
                  << " mode=" << static_cast<int>(result.final_status.active_mode) << '\n';
        emit(false);
        return false;
    }

    if (!expect(result.walk_path_length_m <= kMaxPathLengthM,
                label + ": turn-in-place should keep path length bounded")) {
        std::cerr << label << " path=" << result.walk_path_length_m
                  << " net_horiz=" << horizontal_distance
                  << " dx=" << delta.x
                  << " dy=" << delta.y
                  << " dz=" << delta.z
                  << " yaw_delta=" << yaw_delta
                  << " avg_yaw_rate=" << result.average_yaw_rate_radps
                  << " command_yaw_rate=" << commanded_yaw_rate
                  << " ratio=" << average_ratio
                  << " peak_yaw_rate=" << result.peak_yaw_rate_radps
                  << " mode=" << static_cast<int>(result.final_status.active_mode) << '\n';
        emit(false);
        return false;
    }

    if (!expect(std::abs(yaw_delta) >= kMinYawDeltaRad,
                label + ": turn-in-place should produce measurable yaw change")) {
        std::cerr << label << " yaw_delta=" << yaw_delta
                  << " avg_yaw_rate=" << result.average_yaw_rate_radps
                  << " command_yaw_rate=" << commanded_yaw_rate
                  << " ratio=" << average_ratio << '\n';
        emit(false);
        return false;
    }

    if (!expect(result.average_yaw_rate_radps >= commanded_yaw_rate * kMinAverageYawRateRatio,
                label + ": average yaw speed should stay above the lower band")) {
        std::cerr << label << " avg_yaw_rate=" << result.average_yaw_rate_radps
                  << " command_yaw_rate=" << commanded_yaw_rate
                  << " ratio=" << average_ratio << '\n';
        emit(false);
        return false;
    }

    if (!expect(result.average_yaw_rate_radps <= commanded_yaw_rate * kMaxAverageYawRateRatio,
                label + ": average yaw speed should stay below the upper band")) {
        std::cerr << label << " avg_yaw_rate=" << result.average_yaw_rate_radps
                  << " command_yaw_rate=" << commanded_yaw_rate
                  << " ratio=" << average_ratio << '\n';
        emit(false);
        return false;
    }

    if (!expect(result.peak_yaw_rate_radps >= kMinPeakYawRateRadps,
                label + ": turn-in-place should produce a non-trivial yaw rate")) {
        emit(false);
        return false;
    }

    if (!expect(result.non_walk_mode_steps <= 1,
                label + ": turn command should stay in WALK mode for the full commanded interval apart from at most one trailing sample")) {
        emit(false);
        return false;
    }
    if (!expect(result.faulted_steps <= 1,
                label + ": turn command should not trip more than one trailing safety reject")) {
        emit(false);
        return false;
    }

    std::cout << label << " ok dx=" << delta.x
              << " dy=" << delta.y
              << " dz=" << delta.z
              << " horiz=" << horizontal_distance
              << " path=" << result.walk_path_length_m
              << " yaw_delta=" << yaw_delta
              << " avg_yaw_rate=" << result.average_yaw_rate_radps
              << " ratio=" << average_ratio
              << " peak_yaw_rate=" << result.peak_yaw_rate_radps << '\n';
    emit(true);
    return true;
}

} // namespace

int main(int argc, char** argv) {
#if !defined(__linux__)
    std::cerr << "test_physics_sim_walk_distance: Linux-only\n";
    return EXIT_SUCCESS;
#else
    bool emit_metrics_json = false;
    const char* sim_exe = nullptr;
    physics_sim_test_argv::parse(argc, argv, emit_metrics_json, sim_exe);
    std::string manifest_err;
    if (!test_limits::init(argc, argv, manifest_err)) {
        std::cerr << manifest_err << '\n';
        return 2;
    }
    if (sim_exe == nullptr || sim_exe[0] == '\0') {
        std::cout << "skip test_physics_sim_walk_distance (pass sim path or HEXAPOD_PHYSICS_SIM_EXE)\n";
        return 0;
    }

    const auto harness = physics_sim_test_utils::loadHarnessSettings(/*prefer_test_harness_config=*/true);
    const int kPort = 22000 + (static_cast<int>(::getpid()) % 6000);
    const int kBusLoopPeriodUs = harness.bus_loop_period_us;

    pid_t pid = ::fork();
    if (pid < 0) {
        std::cerr << "fork failed\n";
        return 2;
    }
    if (pid == 0) {
        physics_sim_test_utils::quietChildProcessStdIo();
        const std::string port_str = std::to_string(kPort);
        ::execl(sim_exe,
                sim_exe,
                "--serve",
                "--serve-port",
                port_str.c_str(),
                nullptr);
        std::perror("execl");
        _exit(127);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds{250});

    auto bridge = std::make_unique<CapturingPhysicsSimBridge>(
        "127.0.0.1", kPort, kBusLoopPeriodUs, harness.physics_solver_iterations);
    CapturingPhysicsSimBridge* bridge_ptr = bridge.get();

    control_config::ControlConfig cfg = harness.control_cfg;
    cfg.freshness.estimator.max_allowed_age_us = DurationUs{10'000'000};
    cfg.freshness.intent.max_allowed_age_us = DurationUs{10'000'000};

    RobotRuntime runtime(std::move(bridge), std::make_unique<PhysicsSimEstimator>(), nullptr, cfg);
    if (!expect(runtime.init(), "runtime init should succeed against the live physics sim")) {
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return EXIT_FAILURE;
    }

    const ScenarioMotionIntent stand_motion{
        true,
        RobotMode::STAND,
        GaitType::TRIPOD,
        0.06,
        0.0,
        0.0,
        0.0};
    const ScenarioMotionIntent walk_forward_motion{
        true,
        RobotMode::WALK,
        GaitType::TRIPOD,
        0.06,
        0.20,
        0.0,
        0.0};
    const ScenarioMotionIntent walk_reverse_motion{
        true,
        RobotMode::WALK,
        GaitType::TRIPOD,
        0.06,
        0.20,
        kPi,
        0.0};
    MotionIntent turn_in_place_motion = makeMotionIntent(RobotMode::WALK, GaitType::TRIPOD, 0.06);
    turn_in_place_motion.speed_mps = LinearRateMps{0.0};
    turn_in_place_motion.heading_rad = AngleRad{0.0};
    turn_in_place_motion.twist.twist_vel_radps = Vec3{0.0, 0.0, 0.45};
    turn_in_place_motion.twist.twist_pos_rad = Vec3{0.0, 0.0, 0.0};

    if (!checkWalkCase(
            "forward_walk", runtime, *bridge_ptr, stand_motion, walk_forward_motion, kBusLoopPeriodUs, emit_metrics_json)) {
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return EXIT_FAILURE;
    }

    if (!checkWalkCase(
            "reverse_walk", runtime, *bridge_ptr, stand_motion, walk_reverse_motion, kBusLoopPeriodUs, emit_metrics_json)) {
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return EXIT_FAILURE;
    }

    if (!checkStraightWalkCase("straight_walk",
                               runtime,
                               *bridge_ptr,
                               stand_motion,
                               walk_forward_motion,
                               kBusLoopPeriodUs,
                               emit_metrics_json)) {
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return EXIT_FAILURE;
    }

    if (!checkTurnCase(
            "turn_in_place", runtime, *bridge_ptr, stand_motion, turn_in_place_motion, kBusLoopPeriodUs, emit_metrics_json)) {
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return EXIT_FAILURE;
    }

    ::kill(pid, SIGTERM);
    ::waitpid(pid, nullptr, 0);
    return 0;
#endif
}
