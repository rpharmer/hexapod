#include "control_config.hpp"
#include "hexapod_dynamics_constants.hpp"
#include "locomotion_command.hpp"
#include "motion_intent_utils.hpp"
#include "physics_sim_test_utils.hpp"
#include "physics_sim_link_speed_audit.hpp"
#include "physics_sim_bridge.hpp"
#include "physics_sim_estimator.hpp"
#include "robot_runtime.hpp"
#include "scenario_driver.hpp"
#include "locomotion_metrics.hpp"
#include "physics_sim_metrics_emit.hpp"
#include "physics_sim_test_argv.hpp"
#include "test_limits_manifest.hpp"
#include "swing_clearance_census.hpp"
#include "velocity_lead_experiment.hpp"
#include "stored_motion_experiment.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
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
constexpr std::size_t kSolverFailureReasonCount =
    static_cast<std::size_t>(physics_sim::SolverFailureReason::ExtremePenetration) + 1U;

const char* solverFailureReasonName(const physics_sim::SolverFailureReason reason) {
    switch (reason) {
        case physics_sim::SolverFailureReason::None:
            return "none";
        case physics_sim::SolverFailureReason::InvalidDt:
            return "invalid_dt";
        case physics_sim::SolverFailureReason::ReadState:
            return "read_state";
        case physics_sim::SolverFailureReason::NonFiniteState:
            return "non_finite_state";
        case physics_sim::SolverFailureReason::NonFiniteMass:
            return "non_finite_mass";
        case physics_sim::SolverFailureReason::NonFiniteAcceleration:
            return "non_finite_acceleration";
        case physics_sim::SolverFailureReason::UnsupportedIsland:
            return "unsupported_island";
        case physics_sim::SolverFailureReason::SolverNotConverged:
            return "solver_not_converged";
        case physics_sim::SolverFailureReason::NonFiniteImpulse:
            return "non_finite_impulse";
        case physics_sim::SolverFailureReason::NonFiniteVelocity:
            return "non_finite_velocity";
        case physics_sim::SolverFailureReason::SpeedLimit:
            return "speed_limit";
        case physics_sim::SolverFailureReason::NonFiniteConfiguration:
            return "non_finite_configuration";
        case physics_sim::SolverFailureReason::WriteState:
            return "write_state";
        case physics_sim::SolverFailureReason::NonFiniteEnergy:
            return "non_finite_energy";
        case physics_sim::SolverFailureReason::ExtremePenetration:
            return "extreme_penetration";
    }
    return "unknown";
}

constexpr std::size_t kSpeedLimitFrameCount = 5;

const char* speedLimitFrameName(const std::uint8_t frame) {
    switch (static_cast<physics_sim::SolverSpeedLimitFrame>(frame)) {
        case physics_sim::SolverSpeedLimitFrame::None:
            return "none";
        case physics_sim::SolverSpeedLimitFrame::Chassis:
            return "chassis";
        case physics_sim::SolverSpeedLimitFrame::Coxa:
            return "coxa";
        case physics_sim::SolverSpeedLimitFrame::Femur:
            return "femur";
        case physics_sim::SolverSpeedLimitFrame::Tibia:
            return "tibia";
    }
    return "unknown";
}

const char* speedLimitSupportName(const std::uint8_t support) {
    switch (static_cast<physics_sim::SolverSpeedLimitSupport>(support)) {
        case physics_sim::SolverSpeedLimitSupport::Unknown:
            return "unknown";
        case physics_sim::SolverSpeedLimitSupport::Swing:
            return "swing";
        case physics_sim::SolverSpeedLimitSupport::Stance:
            return "stance";
    }
    return "unknown";
}

const char* solverStatusName(const physics_sim::SolverStatus status) {
    switch (status) {
        case physics_sim::SolverStatus::Healthy:
            return "Healthy";
        case physics_sim::SolverStatus::RecoveredRetry:
            return "RecoveredRetry";
        case physics_sim::SolverStatus::HeldLastGood:
            return "HeldLastGood";
        case physics_sim::SolverStatus::UnsupportedIsland:
            return "UnsupportedIsland";
    }
    return "unknown";
}

void appendSpeedLimitFrameHistogram(
    std::ostream& out,
    const std::array<int, kSpeedLimitFrameCount>& histogram) {
    bool any = false;
    for (std::size_t i = 0; i < histogram.size(); ++i) {
        if (histogram[i] == 0) {
            continue;
        }
        if (any) {
            out << ',';
        }
        any = true;
        out << speedLimitFrameName(static_cast<std::uint8_t>(i)) << '=' << histogram[i];
    }
    if (!any) {
        out << "none";
    }
}

void appendFailureReasonHistogram(
    std::ostream& out,
    const std::array<int, kSolverFailureReasonCount>& histogram) {
    bool any = false;
    for (std::size_t i = 0; i < histogram.size(); ++i) {
        if (histogram[i] == 0) {
            continue;
        }
        if (any) {
            out << ',';
        }
        any = true;
        out << solverFailureReasonName(
                   static_cast<physics_sim::SolverFailureReason>(i))
            << '=' << histogram[i];
    }
    if (!any) {
        out << "none";
    }
}

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
                              PhysicsSimSolverSettings solver_settings)
        : inner_(std::move(host), port, bus_loop_period_us, solver_settings, nullptr),
          velocity_lead_(bus_loop_period_us * 1e-6), stored_motion_(bus_loop_period_us * 1e-6) {}

    bool init() override {
        return inner_.init();
    }

    bool read(RobotState& out) override {
        applied_targets_ = pending_targets_;
        const bool ok = inner_.read(out);
        last_solver_telemetry_ = inner_.latestSolverTelemetry();
        if (ok) auditPublishedPhysicsSimLinkSpeed(out, last_solver_telemetry_);
        if (!ok && !first_failed_solver_telemetry_.has_value()) {
            first_failed_solver_telemetry_ = last_solver_telemetry_;
        }
        if (ok) {
            last_state_ = out;
        }
        return ok;
    }

    bool write(const JointTargets& in) override {
        const auto led = velocity_lead_.apply(in, physics_sim_test_utils::VelocityLeadExperiment::enabledFromEnv(),
                                             physics_sim_test_utils::VelocityLeadExperiment::filteredFromEnv());
        const auto sent = stored_motion_.apply(led, last_state_ ? &*last_state_ : nullptr,
            geometry_config::activeHexapodGeometry(), physics_sim_test_utils::StoredMotionExperiment::enabledFromEnv());
        if (!inner_.write(sent)) {
            return false;
        }
        pending_targets_ = sent;
        return true;
    }

    std::optional<BridgeCommandResultMetadata> last_bridge_result() const override {
        return inner_.last_bridge_result();
    }

    bool supportsAutomaticBusTimeoutRecovery() const override {
        return inner_.supportsAutomaticBusTimeoutRecovery();
    }
    bool usesPhysicsSimBodyAngularConvention() const override {
        return inner_.usesPhysicsSimBodyAngularConvention();
    }
    bool supportsAbsoluteBodyPositionFeedback() const override { return inner_.supportsAbsoluteBodyPositionFeedback(); }

    bool latestSampleHealthyForAutomaticRecovery() const override {
        return inner_.latestSampleHealthyForAutomaticRecovery();
    }

    const std::optional<RobotState>& last_state() const {
        return last_state_;
    }

    const std::optional<PhysicsSimSolverTelemetry>& last_solver_telemetry() const {
        return last_solver_telemetry_;
    }

    const std::optional<PhysicsSimSolverTelemetry>& first_failed_solver_telemetry() const {
        return first_failed_solver_telemetry_;
    }

    void clearFirstFailedSolverTelemetry() {
        first_failed_solver_telemetry_.reset();
    }

    const JointTargets& applied_targets() const { return applied_targets_; }

private:
    PhysicsSimBridge inner_;
    physics_sim_test_utils::VelocityLeadExperiment velocity_lead_;
    physics_sim_test_utils::StoredMotionExperiment stored_motion_;
    std::optional<RobotState> last_state_{};
    std::optional<PhysicsSimSolverTelemetry> last_solver_telemetry_{};
    std::optional<PhysicsSimSolverTelemetry> first_failed_solver_telemetry_{};
    JointTargets pending_targets_{};
    JointTargets applied_targets_{};
};

void runControlLoopStep(RobotRuntime& runtime,
                        const ScenarioMotionIntent& motion,
                        TimePointUs command_time) {
    MotionIntent fresh_motion = makeMotionIntent(motion);
    fresh_motion.timestamp_us = command_time;
    fresh_motion.sample_id = 0;
    runtime.setMotionIntent(fresh_motion);
    runtime.busStep();
    runtime.estimatorStep();
    runtime.safetyStep();
    runtime.controlStep();
}

void runControlLoopStep(RobotRuntime& runtime,
                        const MotionIntent& motion,
                        TimePointUs command_time) {
    MotionIntent fresh_motion = motion;
    // Drive gait and governor timing from the fixed physics cadence, not solver wall time.
    fresh_motion.timestamp_us = command_time;
    fresh_motion.sample_id = 0;
    runtime.setMotionIntent(fresh_motion);
    runtime.busStep();
    runtime.estimatorStep();
    runtime.safetyStep();
    runtime.controlStep();
}

struct TurnTrajTick {
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double yaw{0.0};
    double vx{0.0};
    double vy{0.0};
    double wz{0.0};
    int support{0};
    int fused_count{0};
    int raw_count{0};
    int planned_count{0};
    int left_fused{0};
    int right_fused{0};
    std::array<int, kNumLegs> fused_support{};
    std::array<int, kNumLegs> raw_contact{};
    std::array<int, kNumLegs> planned_stance{};
    std::array<int, kNumLegs> leg_contact_count{};
};

struct SupportDivergenceLegTick {
    double commanded_world_x{0.0};
    double commanded_world_y{0.0};
    double commanded_world_z{0.0};
    double measured_world_x{0.0};
    double measured_world_y{0.0};
    double measured_world_z{0.0};
    double terrain_z{0.0};
    double measured_dz_dt{0.0};
    bool in_stance{false};
    double phase{0.0};
    double duty_factor{0.0};
    bool raw_contact{false};
    bool fused_load_bearing{false};
    std::uint8_t fused_contact_phase{0};
    bool contact_anchor_valid{false};
    double contact_anchor_drift_m{0.0};
    std::array<double, 3> target_rad{};
    std::array<double, 3> measured_rad{};
    std::array<double, 3> error_rad{};
    int leg_contact_count{0};
    // Body-frame stages of the commanded foot: what the planner asked for, what
    // IK produced before the slew clamp, and what survived it.
    std::array<double, 3> planned_body{};
    std::array<double, 3> pre_slew_body{};
    std::array<double, 3> post_clamp_body{};
};

struct SupportDivergenceTick {
    int step{0};
    bool walk{false};
    bool held{false};
    int solver_status{-1};
    int speed_limit_frame{0};
    int speed_limit_support{0};
    double body_x{0.0};
    double body_y{0.0};
    double body_z{0.0};
    // Attribution of the commanded foot motion. The swing plan is re-evaluated
    // every control sample against the live body-velocity estimate, so recording
    // the estimate alongside the planner's own Cartesian output separates
    // "planner demands this" from "IK or the slew clamp distorted it".
    double body_vx{0.0};
    double body_vy{0.0};
    double body_vz{0.0};
    std::array<SupportDivergenceLegTick, kNumLegs> legs{};
};

struct TurnEntrySnapshot {
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double roll{0.0};
    double pitch{0.0};
    double yaw{0.0};
    double vx{0.0};
    double vy{0.0};
    double wz{0.0};
    int support{0};
    double stance_width_m{0.0};
    double foot_centroid_x{0.0};
    double foot_centroid_y{0.0};
    double body_to_centroid_x{0.0};
    double body_to_centroid_y{0.0};
    std::array<double, kNumLegs> foot_world_x{};
    std::array<double, kNumLegs> foot_world_y{};
    std::array<double, kNumLegs> foot_body_x{};
    std::array<double, kNumLegs> foot_body_y{};
    std::array<int, kNumLegs> fused_support{};
};

struct MotionRunResult {
    swing_census::Census swing_events{};
    // Passive eligibility census for the default self-weight screen gates.
    // Not a claim that FF was enabled or that these are post-filter offsets.
    int ff_observed_walk_samples{0};
    int ff_default_imu_permitted_samples{0};
    int ff_default_gyro_rejected_samples{0};
    int ff_default_accel_rejected_samples{0};
    int ff_reported_stiffness_leg_samples{0};
    int ff_default_gate_transitions{0};
    bool ff_previous_gate{false};
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
    double minimum_body_height_m{std::numeric_limits<double>::infinity()};
    double maximum_body_height_m{-std::numeric_limits<double>::infinity()};
    double maximum_body_height_error_m{0.0};
    double maximum_servo_tracking_error_rad{0.0};
    double servo_tracking_error_squared_sum{0.0};
    std::uint64_t servo_tracking_error_samples{0};
    std::array<double, 3> maximum_servo_tracking_error_by_joint_rad{};
    std::array<double, 3> servo_tracking_error_squared_sum_by_joint{};
    std::array<std::uint64_t, 3> servo_tracking_error_samples_by_joint{};
    double maximum_servo_target_rate_radps{0.0};
    std::array<double, 3> maximum_servo_target_rate_by_joint_radps{};
    std::uint64_t servo_target_rate_samples{0};
    std::uint64_t servo_target_rate_above_no_load_samples{0};
    std::array<double, kNumLegs> mean_loaded_stance_commanded_sweep_mps{};
    std::array<double, kNumLegs> mean_loaded_stance_pre_slew_sweep_mps{};
    std::array<double, kNumLegs> mean_loaded_stance_measured_sweep_mps{};
    std::array<double, kNumLegs> mean_loaded_stance_world_slip_mps{};
    std::array<double, kNumLegs> mean_loaded_stance_tracking_error_m{};
    std::array<std::uint64_t, kNumLegs> loaded_stance_samples{};
    std::array<std::uint64_t, kNumLegs> loaded_stance_slew_limited_samples{};
    std::array<std::uint64_t, kNumLegs> loaded_stance_workspace_limited_samples{};
    std::array<std::uint64_t, kNumLegs> loaded_stance_stroke_limited_samples{};
    std::array<std::uint64_t, kNumLegs> loaded_stance_hold_samples{};
    std::array<double, kNumLegs> loaded_stance_hold_phase_min{
        std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
    std::array<double, kNumLegs> loaded_stance_hold_phase_max{};
    std::array<double, kNumLegs> mean_loaded_stance_post_clamp_distortion_m{};
    std::array<double, kNumLegs> mean_loaded_stance_latched_length_m{};
    std::array<double, kNumLegs> mean_loaded_stance_used_length_m{};
    // Gait-execution census. Planned swing that never leaves the ground stores
    // PD error until contact breaks; every frozen first-trip SpeedLimit dump
    // has 0.63-2.98 rad standing at the trip. Planned stance without contact is
    // the complementary miss. Scored on every case so a candidate liftoff
    // change is judged on support and progress, not only on held counts.
    std::array<std::uint64_t, kNumLegs> planned_swing_samples{};
    std::array<std::uint64_t, kNumLegs> planned_swing_contact_samples{};
    std::array<std::uint64_t, kNumLegs> planned_stance_samples{};
    std::array<std::uint64_t, kNumLegs> planned_stance_no_contact_samples{};
    std::array<std::uint64_t, kNumLegs> max_liftoff_delay_samples{};
    double max_loaded_swing_joint_error_rad{0.0};
    double max_planned_swing_measured_foot_clearance_m{0.0};
    std::array<double, 3> mean_planned_stance_joint_error_rad{};
    std::array<double, 3> planned_stance_joint_error_sum{};
    std::uint64_t planned_stance_joint_error_samples{0};
    std::uint64_t raw_contact_count_sum{0};
    std::uint64_t planned_stance_count_sum{0};
    std::uint64_t support_census_samples{0};
    double mean_body_forward_speed_mps{0.0};
    double mean_governed_planar_speed_mps{0.0};
    double mean_governed_yaw_rate_radps{0.0};
    double mean_governor_command_scale{0.0};
    double mean_governor_cadence_scale{0.0};
    double mean_gait_frequency_hz{0.0};
    double mean_gait_duty_factor{0.0};
    double mean_gait_step_length_m{0.0};
    std::uint64_t governor_freeze_samples{0};
    std::array<std::uint64_t, 4> governor_recovery_stage_samples{};
    double mean_body_tilt_rad{0.0};
    double mean_body_roll_rad{0.0};
    double mean_body_pitch_rad{0.0};
    double min_body_pitch_rad{std::numeric_limits<double>::infinity()};
    double max_body_pitch_rad{-std::numeric_limits<double>::infinity()};
    double max_body_tilt_rad{0.0};
    std::uint64_t body_tilt_above_stability_hold_samples{0};
    int walk_mode_steps{0};
    int non_walk_mode_steps{0};
    int faulted_steps{0};
    int solver_healthy_steps{0};
    int solver_recovered_steps{0};
    int solver_held_steps{0};
    int solver_unsupported_steps{0};
    int solver_not_converged_steps{0};
    std::array<int, kSolverFailureReasonCount> solver_recovered_failure_reason{};
    std::array<int, kSolverFailureReasonCount> solver_held_failure_reason{};
    std::array<int, kSpeedLimitFrameCount> solver_speed_limit_frame{};
    int solver_speed_limit_swing_steps{0};
    int solver_speed_limit_stance_steps{0};
    int max_consecutive_held_steps{0};
    int first_non_walk_step{-1};
    int first_fault_step{-1};
    FaultCode first_fault{FaultCode::NONE};
    bool first_read_fail{false};
    int first_failed_solver_status{-1};
    int first_failed_failure_reason{-1};
    int first_failed_speed_limit_frame{0};
    int first_failed_speed_limit_support{0};
    double first_failed_chassis_w{0.0};
    double first_failed_max_link_w{0.0};
    std::uint64_t first_failed_retry_count{0};
    std::uint64_t first_failed_held_state_count{0};
    int first_failed_iterations{0};
    double first_failed_ncp_dual{0.0};
    double first_failed_ncp_comp{0.0};
    double first_failed_admm_rho{0.0};
    std::uint32_t first_failed_contact_constraint_count{0};
    std::uint64_t first_failed_worst_contact_id{0};
    int recovered_speed_limit_streak_before_first_hold{0};
    int recovered_speed_limit_frame_before_first_hold{0};
    int recovered_speed_limit_support_before_first_hold{0};
    int recovered_ncp_streak_before_first_hold{0};
    int max_solver_iterations{0};
    int p99_solver_iterations{0};
    double max_solver_primal_residual{0.0};
    double max_solver_dual_residual{0.0};
    double max_solver_complementarity_residual{0.0};
    double max_solver_ncp_dual_residual{0.0};
    double max_solver_ncp_complementarity_residual{0.0};
    double peak_solver_normal_impulse{0.0};
    double peak_solver_friction_impulse{0.0};
    double max_solver_contact_penetration{0.0};
    double p99_solver_contact_penetration{0.0};
    double max_solver_mechanical_energy_delta_abs{0.0};
    double sum_solver_actuator_work{0.0};
    double max_solver_compliant_projected_residual{0.0};
    double p99_solver_compliant_projected_residual{0.0};
    double peak_solver_actuator_impulse{0.0};
    double peak_solver_servo_torque_utilization{0.0};
    double peak_solver_preintegration_linear_speed{0.0};
    double peak_solver_preintegration_angular_speed{0.0};
    double peak_solver_chassis_preintegration_angular_speed{0.0};
    double peak_solver_max_link_preintegration_angular_speed{0.0};
    std::uint64_t solver_rollback_count_start{0};
    std::uint64_t solver_rollback_count_end{0};
    ControlStatus final_status{};
    double intent_cmd_vx_mps{0.0};
    double intent_cmd_vy_mps{0.0};
    double intent_cmd_yaw_radps{0.0};
    double intent_twist_vx_mps{0.0};
    double intent_twist_vy_mps{0.0};
    double intent_twist_wz_radps{0.0};
    double planar_vx_mps{0.0};
    double planar_vy_mps{0.0};
    double planar_yaw_rate_radps{0.0};
    double raw_vx_mps{0.0};
    double raw_vy_mps{0.0};
    double raw_wz_radps{0.0};
    double intent_planar_mps{0.0};
    double yaw_equiv_mps{0.0};
    bool yaw_dominant{false};
    double stand_end_body_vx_mps{0.0};
    double stand_end_body_vy_mps{0.0};
    double stand_end_body_wz_radps{0.0};
    double first_walk_gait_phase_leg0{0.0};
    double first_walk_requested_planar_speed_mps{0.0};
    double first_walk_governed_planar_speed_mps{0.0};
    double first_walk_requested_yaw_rate_radps{0.0};
    double first_walk_governed_yaw_rate_radps{0.0};
    bool captured_first_walk_frame{false};
    std::vector<TurnTrajTick> turn_traj_ticks{};
    TurnEntrySnapshot stand_end_entry{};
    TurnEntrySnapshot first_walk_entry{};
    bool captured_stand_end_entry{false};
    bool captured_first_walk_entry{false};
    std::vector<SupportDivergenceTick> support_divergence_ticks{};
};

MotionIntent asMotionIntent(const MotionIntent& motion) {
    return motion;
}

MotionIntent asMotionIntent(const ScenarioMotionIntent& motion) {
    return makeMotionIntent(motion);
}

void fillWalkCommandCensus(MotionRunResult& result, const MotionIntent& intent) {
    const PlanarMotionCommand planar = planarMotionCommand(intent);
    const BodyTwist raw = rawLocomotionTwistFromIntent(intent, planar);
    result.intent_cmd_vx_mps = intent.cmd_vx_mps.value;
    result.intent_cmd_vy_mps = intent.cmd_vy_mps.value;
    result.intent_cmd_yaw_radps = intent.cmd_yaw_radps.value;
    result.intent_twist_vx_mps = intent.twist.body_trans_mps.x;
    result.intent_twist_vy_mps = intent.twist.body_trans_mps.y;
    result.intent_twist_wz_radps = intent.twist.twist_vel_radps.z;
    result.planar_vx_mps = planar.vx_mps;
    result.planar_vy_mps = planar.vy_mps;
    result.planar_yaw_rate_radps = planar.yaw_rate_radps;
    result.raw_vx_mps = raw.linear_mps.x;
    result.raw_vy_mps = raw.linear_mps.y;
    result.raw_wz_radps = raw.angular_radps.z;
    result.intent_planar_mps = std::hypot(result.intent_cmd_vx_mps, result.intent_cmd_vy_mps);
    // Same 0.11 m yaw-equivalent radius as GaitScheduler walk-entry seeding.
    constexpr double kWalkEntryYawRadiusM = 0.11;
    result.yaw_equiv_mps = std::abs(result.intent_cmd_yaw_radps) * kWalkEntryYawRadiusM;
    result.yaw_dominant = result.yaw_equiv_mps > result.intent_planar_mps + 1e-6;
}

double wrapAngleDiff(double start, double end) {
    return std::atan2(std::sin(end - start), std::cos(end - start));
}

/**
 * Scored-turn dumps select a pass or a fail donor by net window, so the same
 * plant can supply both sides of a fail-versus-pass comparison. Absent or
 * malformed bounds do not gate.
 */
bool turnDumpNetInWindow(const char* min_env_name,
                         const char* max_env_name,
                         const double net_horiz_m,
                         const char* log_tag) {
    const auto bound = [](const char* name, double& value) {
        const char* text = std::getenv(name);
        if (text == nullptr || text[0] == '\0') {
            return false;
        }
        char* end = nullptr;
        const double parsed = std::strtod(text, &end);
        if (end == text || *end != '\0' || !std::isfinite(parsed)) {
            return false;
        }
        value = parsed;
        return true;
    };
    double min_net = 0.0;
    if (bound(min_env_name, min_net) && net_horiz_m < min_net) {
        std::cerr << log_tag << " skip net=" << net_horiz_m << " min=" << min_net << '\n';
        return false;
    }
    double max_net = 0.0;
    if (bound(max_env_name, max_net) && net_horiz_m > max_net) {
        std::cerr << log_tag << " skip net=" << net_horiz_m << " max=" << max_net << '\n';
        return false;
    }
    return true;
}

void maybeWriteTurnTrajDump(const MotionRunResult& result,
                            const double net_horiz_m,
                            const double yaw_delta,
                            const double equivalent_radius_m,
                            const double path_per_rad_m) {
    const char* path = std::getenv("HEXAPOD_TURN_TRAJ_DUMP_PATH");
    if (path == nullptr || path[0] == '\0') {
        return;
    }
    {
        std::ifstream exists(path);
        if (exists.good()) {
            std::cerr << "[turn-traj-dump] skip existing path=" << path << '\n';
            return;
        }
    }
    if (!turnDumpNetInWindow("HEXAPOD_TURN_TRAJ_DUMP_MIN_NET_M",
                             "HEXAPOD_TURN_TRAJ_DUMP_MAX_NET_M",
                             net_horiz_m,
                             "[turn-traj-dump]")) {
        return;
    }
    std::ofstream out(path, std::ios::out | std::ios::trunc);
    if (!out) {
        std::cerr << "[turn-traj-dump] failed to write path=" << path << '\n';
        return;
    }
    out << std::setprecision(17);
    out << "{\"schema_version\":1,\"kind\":\"turn_traj_dump\""
        << ",\"net_horizontal_distance_m\":" << net_horiz_m
        << ",\"yaw_delta_rad\":" << yaw_delta
        << ",\"r_equivalent_m\":" << equivalent_radius_m
        << ",\"path_per_rad_m\":" << path_per_rad_m
        << ",\"walk_path_length_m\":" << result.walk_path_length_m
        << ",\"held\":" << result.solver_held_steps
        << ",\"cmd_yaw_radps\":" << result.intent_cmd_yaw_radps
        << ",\"yaw_dominant\":" << (result.yaw_dominant ? "true" : "false")
        << ",\"start_x_m\":" << result.start_position.x
        << ",\"start_y_m\":" << result.start_position.y
        << ",\"end_x_m\":" << result.end_position.x
        << ",\"end_y_m\":" << result.end_position.y
        << ",\"samples\":[";
    for (std::size_t i = 0; i < result.turn_traj_ticks.size(); ++i) {
        const TurnTrajTick& t = result.turn_traj_ticks[i];
        if (i > 0) {
            out << ',';
        }
        out << "{\"x\":" << t.x
            << ",\"y\":" << t.y
            << ",\"z\":" << t.z
            << ",\"yaw\":" << t.yaw
            << ",\"vx\":" << t.vx
            << ",\"vy\":" << t.vy
            << ",\"wz\":" << t.wz
            << ",\"support\":" << t.support
            << ",\"fused_count\":" << t.fused_count
            << ",\"raw_count\":" << t.raw_count
            << ",\"planned_count\":" << t.planned_count
            << ",\"left_fused\":" << t.left_fused
            << ",\"right_fused\":" << t.right_fused
            << ",\"fused_support\":[";
        for (int leg = 0; leg < kNumLegs; ++leg) {
            if (leg > 0) {
                out << ',';
            }
            out << t.fused_support[static_cast<std::size_t>(leg)];
        }
        out << "],\"raw_contact\":[";
        for (int leg = 0; leg < kNumLegs; ++leg) {
            if (leg > 0) {
                out << ',';
            }
            out << t.raw_contact[static_cast<std::size_t>(leg)];
        }
        out << "],\"planned_stance\":[";
        for (int leg = 0; leg < kNumLegs; ++leg) {
            if (leg > 0) {
                out << ',';
            }
            out << t.planned_stance[static_cast<std::size_t>(leg)];
        }
        out << "],\"leg_contact_count\":[";
        for (int leg = 0; leg < kNumLegs; ++leg) {
            if (leg > 0) {
                out << ',';
            }
            out << t.leg_contact_count[static_cast<std::size_t>(leg)];
        }
        out << "]}";
    }
    out << "]}\n";
    std::cerr << "[turn-traj-dump] path=" << path
              << " samples=" << result.turn_traj_ticks.size()
              << " net=" << net_horiz_m
              << " yaw=" << yaw_delta
              << " r=" << equivalent_radius_m
              << " held=" << result.solver_held_steps << '\n';
}

void fillTurnEntrySnapshot(TurnEntrySnapshot& snap,
                           const RobotState& state,
                           const telemetry::LocomotionDebugSnapshot& debug) {
    snap.x = state.body_twist_state.body_trans_m.x;
    snap.y = state.body_twist_state.body_trans_m.y;
    snap.z = state.body_twist_state.body_trans_m.z;
    snap.roll = state.body_twist_state.twist_pos_rad.x;
    snap.pitch = state.body_twist_state.twist_pos_rad.y;
    snap.yaw = state.body_twist_state.twist_pos_rad.z;
    snap.vx = state.body_twist_state.body_trans_mps.x;
    snap.vy = state.body_twist_state.body_trans_mps.y;
    snap.wz = state.body_twist_state.twist_vel_radps.z;

    double centroid_x = 0.0;
    double centroid_y = 0.0;
    int support = 0;
    for (int i = 0; i < kNumLegs; ++i) {
        snap.foot_world_x[static_cast<std::size_t>(i)] = debug.measured_foot_world_m[static_cast<std::size_t>(i)].x;
        snap.foot_world_y[static_cast<std::size_t>(i)] = debug.measured_foot_world_m[static_cast<std::size_t>(i)].y;
        snap.foot_body_x[static_cast<std::size_t>(i)] = debug.measured_foot_body_m[static_cast<std::size_t>(i)].x;
        snap.foot_body_y[static_cast<std::size_t>(i)] = debug.measured_foot_body_m[static_cast<std::size_t>(i)].y;
        const bool loaded = debug.fused_support[static_cast<std::size_t>(i)];
        snap.fused_support[static_cast<std::size_t>(i)] = loaded ? 1 : 0;
        if (loaded) {
            centroid_x += snap.foot_world_x[static_cast<std::size_t>(i)];
            centroid_y += snap.foot_world_y[static_cast<std::size_t>(i)];
            ++support;
        }
    }
    snap.support = support;
    if (support > 0) {
        snap.foot_centroid_x = centroid_x / static_cast<double>(support);
        snap.foot_centroid_y = centroid_y / static_cast<double>(support);
        const double c = std::cos(snap.yaw);
        const double s = std::sin(snap.yaw);
        const double dx = snap.foot_centroid_x - snap.x;
        const double dy = snap.foot_centroid_y - snap.y;
        snap.body_to_centroid_x = c * dx + s * dy;
        snap.body_to_centroid_y = -s * dx + c * dy;
    }

    double width = 0.0;
    for (int i = 0; i < kNumLegs; ++i) {
        if (snap.fused_support[static_cast<std::size_t>(i)] == 0) {
            continue;
        }
        for (int j = i + 1; j < kNumLegs; ++j) {
            if (snap.fused_support[static_cast<std::size_t>(j)] == 0) {
                continue;
            }
            width = std::max(
                width,
                std::hypot(snap.foot_world_x[static_cast<std::size_t>(i)]
                               - snap.foot_world_x[static_cast<std::size_t>(j)],
                           snap.foot_world_y[static_cast<std::size_t>(i)]
                               - snap.foot_world_y[static_cast<std::size_t>(j)]));
        }
    }
    snap.stance_width_m = width;
}

void writeTurnEntrySnapshotJson(std::ostream& out, const TurnEntrySnapshot& snap) {
    out << "{\"x\":" << snap.x
        << ",\"y\":" << snap.y
        << ",\"z\":" << snap.z
        << ",\"roll\":" << snap.roll
        << ",\"pitch\":" << snap.pitch
        << ",\"yaw\":" << snap.yaw
        << ",\"vx\":" << snap.vx
        << ",\"vy\":" << snap.vy
        << ",\"wz\":" << snap.wz
        << ",\"support\":" << snap.support
        << ",\"stance_width_m\":" << snap.stance_width_m
        << ",\"foot_centroid_x\":" << snap.foot_centroid_x
        << ",\"foot_centroid_y\":" << snap.foot_centroid_y
        << ",\"body_to_centroid_x\":" << snap.body_to_centroid_x
        << ",\"body_to_centroid_y\":" << snap.body_to_centroid_y
        << ",\"feet\":[";
    for (int i = 0; i < kNumLegs; ++i) {
        if (i > 0) {
            out << ',';
        }
        out << "{\"wx\":" << snap.foot_world_x[static_cast<std::size_t>(i)]
            << ",\"wy\":" << snap.foot_world_y[static_cast<std::size_t>(i)]
            << ",\"bx\":" << snap.foot_body_x[static_cast<std::size_t>(i)]
            << ",\"by\":" << snap.foot_body_y[static_cast<std::size_t>(i)]
            << ",\"support\":" << snap.fused_support[static_cast<std::size_t>(i)] << '}';
    }
    out << "]}";
}

void maybeWriteTurnEntryDump(const MotionRunResult& result, const double net_horiz_m) {
    const char* path = std::getenv("HEXAPOD_TURN_ENTRY_DUMP_PATH");
    if (path == nullptr || path[0] == '\0') {
        return;
    }
    {
        std::ifstream exists(path);
        if (exists.good()) {
            std::cerr << "[turn-entry-dump] skip existing path=" << path << '\n';
            return;
        }
    }
    if (!turnDumpNetInWindow("HEXAPOD_TURN_ENTRY_DUMP_MIN_NET_M",
                             "HEXAPOD_TURN_ENTRY_DUMP_MAX_NET_M",
                             net_horiz_m,
                             "[turn-entry-dump]")) {
        return;
    }
    if (!result.captured_stand_end_entry || !result.captured_first_walk_entry) {
        std::cerr << "[turn-entry-dump] incomplete snapshots path=" << path << '\n';
        return;
    }
    std::ofstream out(path, std::ios::out | std::ios::trunc);
    if (!out) {
        std::cerr << "[turn-entry-dump] failed to write path=" << path << '\n';
        return;
    }
    out << std::setprecision(17);
    out << "{\"schema_version\":1,\"kind\":\"turn_entry_dump\""
        << ",\"net_horizontal_distance_m\":" << net_horiz_m
        << ",\"held\":" << result.solver_held_steps
        << ",\"cmd_yaw_radps\":" << result.intent_cmd_yaw_radps
        << ",\"yaw_dominant\":" << (result.yaw_dominant ? "true" : "false")
        << ",\"start_x_m\":" << result.start_position.x
        << ",\"start_y_m\":" << result.start_position.y
        << ",\"stand_end\":";
    writeTurnEntrySnapshotJson(out, result.stand_end_entry);
    out << ",\"first_walk\":";
    writeTurnEntrySnapshotJson(out, result.first_walk_entry);
    out << "}\n";
    std::cerr << "[turn-entry-dump] path=" << path
              << " net=" << net_horiz_m
              << " stand_z=" << result.stand_end_entry.z
              << " first_z=" << result.first_walk_entry.z
              << " stand_width=" << result.stand_end_entry.stance_width_m
              << " first_width=" << result.first_walk_entry.stance_width_m
              << " stand_support=" << result.stand_end_entry.support
              << " first_support=" << result.first_walk_entry.support
              << " first_xy=" << result.first_walk_entry.x << "," << result.first_walk_entry.y
              << " held=" << result.solver_held_steps << '\n';
}

void maybeArmCutpointDump() {
    const char* dump_path = std::getenv("HEXAPOD_PINOCCHIO_CUTPOINT_DUMP_PATH");
    if (dump_path == nullptr || dump_path[0] == '\0') {
        return;
    }
    {
        std::ifstream exists(dump_path);
        if (exists.good()) {
            std::cerr << "[turn-entry-cutpoint] skip existing dump path=" << dump_path << '\n';
            return;
        }
    }
    const char* request = std::getenv("HEXAPOD_PINOCCHIO_CUTPOINT_DUMP_REQUEST");
    if (request == nullptr || request[0] == '\0') {
        std::cerr << "[turn-entry-cutpoint] DUMP_PATH set but DUMP_REQUEST unset\n";
        return;
    }
    std::ofstream out(request, std::ios::out | std::ios::trunc);
    if (!out) {
        std::cerr << "[turn-entry-cutpoint] failed to arm request=" << request << '\n';
        return;
    }
    out << "1\n";
    std::cerr << "[turn-entry-cutpoint] armed dump request=" << request
              << " path=" << dump_path << '\n';
}

void maybeArmCutpointRestore() {
    const char* restore_path = std::getenv("HEXAPOD_PINOCCHIO_CUTPOINT_RESTORE_PATH");
    if (restore_path == nullptr || restore_path[0] == '\0') {
        return;
    }
    const char* request = std::getenv("HEXAPOD_PINOCCHIO_CUTPOINT_RESTORE_REQUEST");
    if (request == nullptr || request[0] == '\0') {
        std::cerr << "[turn-entry-cutpoint] RESTORE_PATH set but RESTORE_REQUEST unset\n";
        return;
    }
    std::ofstream out(request, std::ios::out | std::ios::trunc);
    if (!out) {
        std::cerr << "[turn-entry-cutpoint] failed to arm restore request=" << request << '\n';
        return;
    }
    out << "1\n";
    std::cerr << "[turn-entry-cutpoint] armed restore request=" << request
              << " path=" << restore_path << '\n';
}

void writeJsonBoolArray(std::ostream& out, const std::array<bool, kNumLegs>& values) {
    out << '[';
    for (int i = 0; i < kNumLegs; ++i) {
        if (i > 0) {
            out << ',';
        }
        out << (values[static_cast<std::size_t>(i)] ? "true" : "false");
    }
    out << ']';
}

void maybeWriteSupportDivergenceDump(const std::string& label, const MotionRunResult& result) {
    const char* path = std::getenv("HEXAPOD_SUPPORT_DIVERGENCE_DUMP_PATH");
    if (path == nullptr || path[0] == '\0') {
        return;
    }
    const char* case_filter = std::getenv("HEXAPOD_SUPPORT_DIVERGENCE_DUMP_CASE");
    if (case_filter != nullptr && case_filter[0] != '\0' && label != case_filter) {
        return;
    }
    if (result.support_divergence_ticks.empty()) {
        return;
    }
    if (result.solver_held_steps <= 0 && result.first_non_walk_step < 0) {
        return;
    }
    {
        std::ifstream exists(path);
        if (exists.good()) {
            std::cerr << "[support-divergence-dump] skip existing path=" << path << '\n';
            return;
        }
    }
    std::ofstream out(path, std::ios::out | std::ios::trunc);
    if (!out) {
        std::cerr << "[support-divergence-dump] failed to write path=" << path << '\n';
        return;
    }
    out << std::setprecision(17);
    out << "{\"schema_version\":1,\"kind\":\"support_divergence_dump\""
        << ",\"case\":\"" << label << "\""
        << ",\"held\":" << result.solver_held_steps
        << ",\"first_non_walk_step\":" << result.first_non_walk_step
        << ",\"first_failed_speed_limit_frame\":" << result.first_failed_speed_limit_frame
        << ",\"first_failed_speed_limit_support\":" << result.first_failed_speed_limit_support
        << ",\"first_failed_max_link_w\":" << result.first_failed_max_link_w
        << ",\"first_failed_held_state_count\":" << result.first_failed_held_state_count
        << ",\"samples\":[";
    for (std::size_t i = 0; i < result.support_divergence_ticks.size(); ++i) {
        const SupportDivergenceTick& tick = result.support_divergence_ticks[i];
        if (i > 0) {
            out << ',';
        }
        out << "{\"step\":" << tick.step
            << ",\"walk\":" << (tick.walk ? "true" : "false")
            << ",\"held\":" << (tick.held ? "true" : "false")
            << ",\"solver_status\":" << tick.solver_status
            << ",\"speed_limit_frame\":" << tick.speed_limit_frame
            << ",\"speed_limit_support\":" << tick.speed_limit_support
            << ",\"body_x\":" << tick.body_x
            << ",\"body_y\":" << tick.body_y
            << ",\"body_z\":" << tick.body_z
            << ",\"body_v\":[" << tick.body_vx << ',' << tick.body_vy << ',' << tick.body_vz << "]"
            << ",\"legs\":[";
        for (int leg = 0; leg < kNumLegs; ++leg) {
            const SupportDivergenceLegTick& L = tick.legs[static_cast<std::size_t>(leg)];
            if (leg > 0) {
                out << ',';
            }
            out << "{\"leg\":" << leg
                << ",\"commanded_world\":[" << L.commanded_world_x << ',' << L.commanded_world_y
                << ',' << L.commanded_world_z << "]"
                << ",\"measured_world\":[" << L.measured_world_x << ',' << L.measured_world_y
                << ',' << L.measured_world_z << "]"
                << ",\"terrain_z\":" << L.terrain_z
                << ",\"measured_dz_dt\":" << L.measured_dz_dt
                << ",\"in_stance\":" << (L.in_stance ? "true" : "false")
                << ",\"phase\":" << L.phase
                << ",\"duty_factor\":" << L.duty_factor
                << ",\"raw_contact\":" << (L.raw_contact ? "true" : "false")
                << ",\"fused_load_bearing\":" << (L.fused_load_bearing ? "true" : "false")
                << ",\"fused_contact_phase\":" << static_cast<unsigned>(L.fused_contact_phase)
                << ",\"contact_anchor_valid\":" << (L.contact_anchor_valid ? "true" : "false")
                << ",\"contact_anchor_drift_m\":" << L.contact_anchor_drift_m
                << ",\"target_rad\":[" << L.target_rad[0] << ',' << L.target_rad[1] << ','
                << L.target_rad[2] << "]"
                << ",\"measured_rad\":[" << L.measured_rad[0] << ',' << L.measured_rad[1] << ','
                << L.measured_rad[2] << "]"
                << ",\"error_rad\":[" << L.error_rad[0] << ',' << L.error_rad[1] << ','
                << L.error_rad[2] << "]"
                << ",\"leg_contact_count\":" << L.leg_contact_count
                << ",\"planned_body\":[" << L.planned_body[0] << ',' << L.planned_body[1] << ','
                << L.planned_body[2] << "]"
                << ",\"pre_slew_body\":[" << L.pre_slew_body[0] << ',' << L.pre_slew_body[1] << ','
                << L.pre_slew_body[2] << "]"
                << ",\"post_clamp_body\":[" << L.post_clamp_body[0] << ','
                << L.post_clamp_body[1] << ',' << L.post_clamp_body[2] << "]}";
        }
        out << "]}";
    }
    out << "]}\n";
    std::cerr << "[support-divergence-dump] path=" << path
              << " case=" << label
              << " samples=" << result.support_divergence_ticks.size()
              << " held=" << result.solver_held_steps << '\n';
}

void maybeWriteTurnEntryControllerDump(RobotRuntime& runtime) {
    const char* path = std::getenv("HEXAPOD_TURN_ENTRY_CONTROLLER_DUMP_PATH");
    if (path == nullptr || path[0] == '\0') {
        return;
    }
    {
        std::ifstream exists(path);
        if (exists.good()) {
            std::cerr << "[turn-entry-controller-dump] skip existing path=" << path << '\n';
            return;
        }
    }
    const GaitState gait = runtime.gaitSnapshot();
    const CommandGovernorState governor = runtime.commandGovernorSnapshot();
    const telemetry::LocomotionDebugSnapshot debug = runtime.locomotionDebugSnapshot();
    std::ofstream out(path, std::ios::out | std::ios::trunc);
    if (!out) {
        std::cerr << "[turn-entry-controller-dump] failed path=" << path << '\n';
        return;
    }
    out << std::setprecision(17);
    out << "{\"schema_version\":1,\"kind\":\"turn_entry_controller\"";
    out << ",\"duty_factor\":" << gait.duty_factor
        << ",\"step_length_m\":" << gait.step_length_m
        << ",\"swing_height_m\":" << gait.swing_height_m
        << ",\"stride_phase_rate_hz\":" << gait.stride_phase_rate_hz.value
        << ",\"gov_planar\":" << governor.governed_planar_speed_mps
        << ",\"gov_yaw\":" << governor.governed_yaw_rate_radps
        << ",\"command_scale\":" << governor.command_scale
        << ",\"cadence_scale\":" << governor.cadence_scale
        << ",\"support_count\":" << governor.current_support_count
        << ",\"gait_phase\":[";
    for (int i = 0; i < kNumLegs; ++i) {
        if (i > 0) {
            out << ',';
        }
        out << gait.phase[static_cast<std::size_t>(i)];
    }
    out << "],\"phase_offset\":[";
    for (int i = 0; i < kNumLegs; ++i) {
        if (i > 0) {
            out << ',';
        }
        out << gait.phase_offset[static_cast<std::size_t>(i)];
    }
    out << "],\"in_stance\":";
    writeJsonBoolArray(out, gait.in_stance);
    out << ",\"fused_support\":";
    writeJsonBoolArray(out, debug.fused_support);
    out << ",\"anchor_valid\":";
    writeJsonBoolArray(out, debug.contact_anchor_valid);
    out << ",\"anchor_world\":[";
    for (int i = 0; i < kNumLegs; ++i) {
        if (i > 0) {
            out << ',';
        }
        const Vec3& p = debug.contact_anchor_world_m[static_cast<std::size_t>(i)];
        out << "{\"x\":" << p.x << ",\"y\":" << p.y << ",\"z\":" << p.z << '}';
    }
    out << "]}\n";
    std::cerr << "[turn-entry-controller-dump] path=" << path
              << " phase0=" << gait.phase[0]
              << " support=" << governor.current_support_count << '\n';
}

bool extractControllerNumber(const std::string& text, const char* key, double& out) {
    const std::string needle = std::string("\"") + key + "\":";
    const auto pos = text.find(needle);
    if (pos == std::string::npos) {
        return false;
    }
    char* end = nullptr;
    out = std::strtod(text.c_str() + pos + needle.size(), &end);
    return end != text.c_str() + pos + needle.size() && std::isfinite(out);
}

bool extractControllerArray(const std::string& text, const char* key, std::array<double, kNumLegs>& out) {
    const std::string needle = std::string("\"") + key + "\":[";
    const auto pos = text.find(needle);
    if (pos == std::string::npos) {
        return false;
    }
    const char* cursor = text.c_str() + pos + needle.size();
    for (int i = 0; i < kNumLegs; ++i) {
        char* end = nullptr;
        out[static_cast<std::size_t>(i)] = std::strtod(cursor, &end);
        if (end == cursor) {
            return false;
        }
        cursor = end;
        if (*cursor == ',') {
            ++cursor;
        }
    }
    return true;
}

bool extractControllerBoolArray(const std::string& text, const char* key, std::array<bool, kNumLegs>& out) {
    const std::string needle = std::string("\"") + key + "\":[";
    const auto pos = text.find(needle);
    if (pos == std::string::npos) {
        return false;
    }
    std::size_t i = pos + needle.size();
    for (int idx = 0; idx < kNumLegs && i < text.size(); ++idx) {
        while (i < text.size() && (text[i] == ' ' || text[i] == ',')) {
            ++i;
        }
        if (text.compare(i, 4, "true") == 0) {
            out[static_cast<std::size_t>(idx)] = true;
            i += 4;
        } else if (text.compare(i, 5, "false") == 0) {
            out[static_cast<std::size_t>(idx)] = false;
            i += 5;
        } else {
            return false;
        }
    }
    return true;
}

void maybeRestoreTurnEntryController(RobotRuntime& runtime) {
    const char* path = std::getenv("HEXAPOD_TURN_ENTRY_CONTROLLER_RESTORE_PATH");
    if (path == nullptr || path[0] == '\0') {
        return;
    }
    std::ifstream in(path, std::ios::in | std::ios::binary);
    if (!in) {
        std::cerr << "[turn-entry-controller-restore] missing path=" << path << '\n';
        return;
    }
    std::ostringstream ss;
    ss << in.rdbuf();
    const std::string text = ss.str();
    GaitState gait{};
    CommandGovernorState governor{};
    std::array<bool, kNumLegs> anchor_valid{};
    std::array<Vec3, kNumLegs> anchor_world{};
    std::array<double, kNumLegs> phase{};
    std::array<double, kNumLegs> offsets{};
    (void)extractControllerArray(text, "gait_phase", phase);
    (void)extractControllerArray(text, "phase_offset", offsets);
    gait.phase = phase;
    gait.phase_offset = offsets;
    (void)extractControllerNumber(text, "duty_factor", gait.duty_factor);
    (void)extractControllerNumber(text, "step_length_m", gait.step_length_m);
    (void)extractControllerNumber(text, "swing_height_m", gait.swing_height_m);
    (void)extractControllerNumber(text, "stride_phase_rate_hz", gait.stride_phase_rate_hz.value);
    (void)extractControllerBoolArray(text, "in_stance", gait.in_stance);
    (void)extractControllerNumber(text, "gov_planar", governor.governed_planar_speed_mps);
    (void)extractControllerNumber(text, "gov_yaw", governor.governed_yaw_rate_radps);
    (void)extractControllerNumber(text, "command_scale", governor.command_scale);
    (void)extractControllerNumber(text, "cadence_scale", governor.cadence_scale);
    double support = 0.0;
    if (extractControllerNumber(text, "support_count", support)) {
        governor.current_support_count = static_cast<int>(support);
    }
    (void)extractControllerBoolArray(text, "anchor_valid", anchor_valid);
    const auto anchors_pos = text.find("\"anchor_world\":[");
    if (anchors_pos != std::string::npos) {
        std::size_t i = anchors_pos + 16;
        for (int idx = 0; idx < kNumLegs && i < text.size(); ++idx) {
            const auto start = text.find('{', i);
            if (start == std::string::npos) {
                break;
            }
            const auto end = text.find('}', start);
            if (end == std::string::npos) {
                break;
            }
            const std::string obj = text.substr(start, end - start + 1);
            (void)extractControllerNumber(obj, "x", anchor_world[static_cast<std::size_t>(idx)].x);
            (void)extractControllerNumber(obj, "y", anchor_world[static_cast<std::size_t>(idx)].y);
            (void)extractControllerNumber(obj, "z", anchor_world[static_cast<std::size_t>(idx)].z);
            i = end + 1;
        }
    }
    runtime.debugRestoreTurnEntryController(gait, governor, anchor_valid, anchor_world);
    std::cerr << "[turn-entry-controller-restore] path=" << path
              << " phase0=" << gait.phase[0] << '\n';
}

struct DirectionalTravel {
    double forward_projection_m{0.0};
    double lateral_projection_m{0.0};
    double alignment_cosine{0.0};
};

DirectionalTravel directionalTravelFromStartYaw(const MotionRunResult& result,
                                                 const Vec3& delta,
                                                 const double commanded_body_heading_rad) {
    const double expected_world_heading = result.start_yaw_rad + commanded_body_heading_rad;
    const double c = std::cos(expected_world_heading);
    const double s = std::sin(expected_world_heading);
    DirectionalTravel out{};
    out.forward_projection_m = delta.x * c + delta.y * s;
    out.lateral_projection_m = -delta.x * s + delta.y * c;
    const double distance = std::hypot(delta.x, delta.y);
    out.alignment_cosine = distance > 1.0e-9 ? out.forward_projection_m / distance : 0.0;
    return out;
}

std::string walkDistanceLimitsWalkEnvelopeJsonDynamic(const std::string& label,
                                                      const double min_path_length_m,
                                                      const double min_net_horizontal_distance_m,
                                                      const double min_command_direction_projection_m,
                                                      const double min_command_direction_cosine,
                                                      const double min_peak_horizontal_speed_mps,
                                                      const double min_average_speed_ratio,
                                                      const double max_average_speed_ratio,
                                                      const bool require_active_mode_walk) {
    using locomotion_test::formatDouble;
    std::ostringstream o;
    o << "{\"min_path_length_m\":" << formatDouble(min_path_length_m)
      << ",\"min_net_horizontal_distance_m\":" << formatDouble(min_net_horizontal_distance_m)
      << ",\"min_command_direction_projection_m\":" << formatDouble(min_command_direction_projection_m)
      << ",\"min_command_direction_cosine\":" << formatDouble(min_command_direction_cosine)
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
                                                  const double min_command_direction_projection_m,
                                                  const double min_command_direction_cosine,
                                                  const double min_peak_horizontal_speed_mps,
                                                  const double min_average_speed_ratio,
                                                  const double max_average_speed_ratio,
                                                  const bool require_active_mode_walk) {
    using locomotion_test::formatDouble;
    std::ostringstream o;
    o << "{\"min_path_length_m\":" << formatDouble(min_path_length_m)
      << ",\"max_lateral_deviation_m\":" << formatDouble(max_lateral_deviation_m)
      << ",\"max_lateral_vs_path_ratio\":" << formatDouble(max_lateral_vs_path_ratio)
      << ",\"min_command_direction_projection_m\":" << formatDouble(min_command_direction_projection_m)
      << ",\"min_command_direction_cosine\":" << formatDouble(min_command_direction_cosine)
      << ",\"min_peak_horizontal_speed_mps\":" << formatDouble(min_peak_horizontal_speed_mps)
      << ",\"min_average_speed_ratio\":" << formatDouble(min_average_speed_ratio)
      << ",\"max_average_speed_ratio\":" << formatDouble(max_average_speed_ratio)
      << ",\"require_active_mode_walk\":" << (require_active_mode_walk ? "true" : "false") << '}';
    (void)label;
    return o.str();
}

void appendUintArray(std::ostream& o, const char* key,
                     const std::array<std::uint64_t, kNumLegs>& values) {
    o << key << '[';
    for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
        o << (leg == 0 ? "" : ",") << values[leg];
    }
    o << ']';
}

std::string walkDistanceLimitsTurnJsonDynamic(const std::string& label,
                                              const double max_path_length_m,
                                              const double max_net_horizontal_distance_m,
                                              const double min_peak_yaw_rate_radps,
                                              const double min_average_yaw_rate_ratio,
                                              const double max_average_yaw_rate_ratio,
                                              const double min_yaw_delta_abs_rad,
                                              const bool require_commanded_yaw_direction,
                                              const bool require_active_mode_walk) {
    using locomotion_test::formatDouble;
    std::ostringstream o;
    o << "{\"max_path_length_m\":" << formatDouble(max_path_length_m)
      << ",\"max_net_horizontal_distance_m\":" << formatDouble(max_net_horizontal_distance_m)
      << ",\"min_peak_yaw_rate_radps\":" << formatDouble(min_peak_yaw_rate_radps)
      << ",\"min_average_yaw_rate_ratio\":" << formatDouble(min_average_yaw_rate_ratio)
      << ",\"max_average_yaw_rate_ratio\":" << formatDouble(max_average_yaw_rate_ratio)
      << ",\"min_yaw_delta_abs_rad\":" << formatDouble(min_yaw_delta_abs_rad)
      << ",\"require_commanded_yaw_direction\":" << (require_commanded_yaw_direction ? "true" : "false")
      << ",\"require_active_mode_walk\":" << (require_active_mode_walk ? "true" : "false") << '}';
    (void)label;
    return o.str();
}

std::string motionRunResultMetricsJson(const MotionRunResult& result,
                                       const Vec3& delta,
                                       const double horizontal_distance,
                                       const double commanded_speed,
                                       const double average_ratio,
                                       const DirectionalTravel& direction) {
    std::ostringstream o;
    using locomotion_test::formatDouble;
    o << "{\"walk_path_length_m\":" << formatDouble(result.walk_path_length_m)
      << ",\"ff_observed_walk_samples\":" << result.ff_observed_walk_samples
      << ",\"ff_default_imu_permitted_samples\":" << result.ff_default_imu_permitted_samples
      << ",\"ff_default_gyro_rejected_samples\":" << result.ff_default_gyro_rejected_samples
      << ",\"ff_default_accel_rejected_samples\":" << result.ff_default_accel_rejected_samples
      << ",\"ff_reported_stiffness_leg_samples\":" << result.ff_reported_stiffness_leg_samples
      << ",\"ff_default_gate_transitions\":" << result.ff_default_gate_transitions
      << ",\"net_horizontal_distance_m\":" << formatDouble(horizontal_distance)
      << ",\"delta_x_m\":" << formatDouble(delta.x) << ",\"delta_y_m\":" << formatDouble(delta.y)
      << ",\"delta_z_m\":" << formatDouble(delta.z)
      << ",\"command_direction_projection_m\":" << formatDouble(direction.forward_projection_m)
      << ",\"command_lateral_projection_m\":" << formatDouble(direction.lateral_projection_m)
      << ",\"command_direction_cosine\":" << formatDouble(direction.alignment_cosine)
      << ",\"max_lateral_deviation_m\":" << formatDouble(result.max_lateral_deviation_m)
      << ",\"average_horizontal_speed_mps\":" << formatDouble(result.average_horizontal_speed_mps)
      << ",\"peak_horizontal_speed_mps\":" << formatDouble(result.peak_horizontal_speed_mps)
      << ",\"average_yaw_rate_radps\":" << formatDouble(result.average_yaw_rate_radps)
      << ",\"peak_yaw_rate_radps\":" << formatDouble(result.peak_yaw_rate_radps)
      << ",\"minimum_body_height_m\":" << formatDouble(result.minimum_body_height_m)
      << ",\"maximum_body_height_m\":" << formatDouble(result.maximum_body_height_m)
      << ",\"maximum_body_height_error_m\":"
      << formatDouble(result.maximum_body_height_error_m)
      << ",\"maximum_servo_tracking_error_rad\":"
      << formatDouble(result.maximum_servo_tracking_error_rad)
      << ",\"servo_tracking_error_rms_rad\":"
      << formatDouble(result.servo_tracking_error_samples == 0
              ? 0.0
              : std::sqrt(result.servo_tracking_error_squared_sum
                  / static_cast<double>(result.servo_tracking_error_samples)))
      << ",\"maximum_servo_tracking_error_by_joint_rad\":["
      << formatDouble(result.maximum_servo_tracking_error_by_joint_rad[0]) << ','
      << formatDouble(result.maximum_servo_tracking_error_by_joint_rad[1]) << ','
      << formatDouble(result.maximum_servo_tracking_error_by_joint_rad[2]) << ']'
      << ",\"servo_tracking_error_rms_by_joint_rad\":["
      << formatDouble(result.servo_tracking_error_samples_by_joint[0] == 0
              ? 0.0
              : std::sqrt(result.servo_tracking_error_squared_sum_by_joint[0]
                  / static_cast<double>(result.servo_tracking_error_samples_by_joint[0]))) << ','
      << formatDouble(result.servo_tracking_error_samples_by_joint[1] == 0
              ? 0.0
              : std::sqrt(result.servo_tracking_error_squared_sum_by_joint[1]
                  / static_cast<double>(result.servo_tracking_error_samples_by_joint[1]))) << ','
      << formatDouble(result.servo_tracking_error_samples_by_joint[2] == 0
              ? 0.0
              : std::sqrt(result.servo_tracking_error_squared_sum_by_joint[2]
                  / static_cast<double>(result.servo_tracking_error_samples_by_joint[2]))) << ']'
      << ",\"maximum_servo_target_rate_radps\":"
      << formatDouble(result.maximum_servo_target_rate_radps)
      << ",\"maximum_servo_target_rate_by_joint_radps\":["
      << formatDouble(result.maximum_servo_target_rate_by_joint_radps[0]) << ','
      << formatDouble(result.maximum_servo_target_rate_by_joint_radps[1]) << ','
      << formatDouble(result.maximum_servo_target_rate_by_joint_radps[2]) << ']'
      << ",\"servo_target_rate_above_no_load_samples\":"
      << result.servo_target_rate_above_no_load_samples
      << ",\"servo_target_rate_samples\":" << result.servo_target_rate_samples
      << ",\"mean_body_forward_speed_mps\":"
      << formatDouble(result.mean_body_forward_speed_mps)
      << ",\"mean_loaded_stance_commanded_sweep_mps\":[";
    for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
        o << (leg == 0 ? "" : ",")
          << formatDouble(result.mean_loaded_stance_commanded_sweep_mps[leg]);
    }
    o << ']';
    o << ",\"mean_loaded_stance_measured_sweep_mps\":[";
    for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
        o << (leg == 0 ? "" : ",")
          << formatDouble(result.mean_loaded_stance_measured_sweep_mps[leg]);
    }
    o << ']';
    o << ",\"mean_loaded_stance_world_slip_mps\":[";
    for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
        o << (leg == 0 ? "" : ",")
          << formatDouble(result.mean_loaded_stance_world_slip_mps[leg]);
    }
    o << ']';
    o << ",\"mean_loaded_stance_tracking_error_m\":[";
    for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
        o << (leg == 0 ? "" : ",")
          << formatDouble(result.mean_loaded_stance_tracking_error_m[leg]);
    }
    o << ']';
    o << ",\"loaded_stance_samples\":[";
    for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
        o << (leg == 0 ? "" : ",") << result.loaded_stance_samples[leg];
    }
    o << ']';
    appendUintArray(o, ",\"planned_swing_samples\":", result.planned_swing_samples);
    appendUintArray(o, ",\"planned_swing_contact_samples\":", result.planned_swing_contact_samples);
    appendUintArray(o, ",\"planned_stance_samples\":", result.planned_stance_samples);
    appendUintArray(o, ",\"planned_stance_no_contact_samples\":",
                    result.planned_stance_no_contact_samples);
    appendUintArray(o, ",\"max_liftoff_delay_samples\":", result.max_liftoff_delay_samples);
    result.swing_events.json(o);
    o << ",\"max_loaded_swing_joint_error_rad\":"
      << formatDouble(result.max_loaded_swing_joint_error_rad)
      << ",\"max_planned_swing_measured_foot_clearance_m\":"
      << formatDouble(result.max_planned_swing_measured_foot_clearance_m)
      << ",\"mean_planned_stance_joint_error_rad\":["
      << formatDouble(result.mean_planned_stance_joint_error_rad[0]) << ','
      << formatDouble(result.mean_planned_stance_joint_error_rad[1]) << ','
      << formatDouble(result.mean_planned_stance_joint_error_rad[2]) << "]"
      << ",\"raw_contact_count_sum\":" << result.raw_contact_count_sum
      << ",\"planned_stance_count_sum\":" << result.planned_stance_count_sum
      << ",\"support_census_samples\":" << result.support_census_samples
      << ",\"walk_mode_steps\":" << result.walk_mode_steps
      << ",\"non_walk_mode_steps\":" << result.non_walk_mode_steps
      << ",\"faulted_steps\":" << result.faulted_steps
      << ",\"solver_healthy_steps\":" << result.solver_healthy_steps
      << ",\"solver_recovered_steps\":" << result.solver_recovered_steps
      << ",\"solver_held_steps\":" << result.solver_held_steps
      << ",\"solver_unsupported_steps\":" << result.solver_unsupported_steps
      << ",\"solver_not_converged_steps\":" << result.solver_not_converged_steps
      << ",\"solver_recovered_failure_reason\":\"";
    appendFailureReasonHistogram(o, result.solver_recovered_failure_reason);
    o << "\",\"solver_held_failure_reason\":\"";
    appendFailureReasonHistogram(o, result.solver_held_failure_reason);
    o << "\",\"solver_speed_limit_frame\":\"";
    appendSpeedLimitFrameHistogram(o, result.solver_speed_limit_frame);
    o << "\",\"solver_speed_limit_swing_steps\":" << result.solver_speed_limit_swing_steps
      << ",\"solver_speed_limit_stance_steps\":" << result.solver_speed_limit_stance_steps
      << ",\"max_solver_iterations\":" << result.max_solver_iterations
      << ",\"p99_solver_iterations\":" << result.p99_solver_iterations
      << ",\"max_solver_primal_residual\":"
      << formatDouble(result.max_solver_primal_residual, 12)
      << ",\"max_solver_dual_residual\":"
      << formatDouble(result.max_solver_dual_residual, 12)
      << ",\"max_solver_complementarity_residual\":"
      << formatDouble(result.max_solver_complementarity_residual, 12)
      << ",\"max_solver_ncp_dual_residual\":"
      << formatDouble(result.max_solver_ncp_dual_residual, 12)
      << ",\"max_solver_ncp_complementarity_residual\":"
      << formatDouble(result.max_solver_ncp_complementarity_residual, 12)
      << ",\"peak_solver_normal_impulse\":"
      << formatDouble(result.peak_solver_normal_impulse, 12)
      << ",\"max_solver_contact_penetration\":"
      << formatDouble(result.max_solver_contact_penetration, 12)
      << ",\"p99_solver_contact_penetration\":"
      << formatDouble(result.p99_solver_contact_penetration, 12)
      << ",\"max_solver_mechanical_energy_delta_abs\":"
      << formatDouble(result.max_solver_mechanical_energy_delta_abs, 12)
      << ",\"sum_solver_actuator_work\":"
      << formatDouble(result.sum_solver_actuator_work, 12)
      << ",\"max_solver_compliant_projected_residual\":"
      << formatDouble(result.max_solver_compliant_projected_residual, 12)
      << ",\"p99_solver_compliant_projected_residual\":"
      << formatDouble(result.p99_solver_compliant_projected_residual, 12)
      << ",\"peak_solver_friction_impulse\":"
      << formatDouble(result.peak_solver_friction_impulse, 12)
      << ",\"peak_solver_actuator_impulse\":"
      << formatDouble(result.peak_solver_actuator_impulse, 12)
      << ",\"peak_solver_servo_torque_utilization\":"
      << formatDouble(result.peak_solver_servo_torque_utilization, 12)
      << ",\"peak_solver_preintegration_linear_speed\":"
      << formatDouble(result.peak_solver_preintegration_linear_speed, 12)
      << ",\"peak_solver_preintegration_angular_speed\":"
      << formatDouble(result.peak_solver_preintegration_angular_speed, 12)
      << ",\"peak_solver_chassis_preintegration_angular_speed\":"
      << formatDouble(result.peak_solver_chassis_preintegration_angular_speed, 12)
      << ",\"peak_solver_max_link_preintegration_angular_speed\":"
      << formatDouble(result.peak_solver_max_link_preintegration_angular_speed, 12)
      << ",\"max_consecutive_held_steps\":" << result.max_consecutive_held_steps
      << ",\"first_non_walk_step\":" << result.first_non_walk_step
      << ",\"first_fault_step\":" << result.first_fault_step
      << ",\"first_fault\":" << static_cast<int>(result.first_fault)
      << ",\"first_read_fail\":" << (result.first_read_fail ? "true" : "false")
      << ",\"first_failed_solver_status\":" << result.first_failed_solver_status
      << ",\"first_failed_failure_reason\":" << result.first_failed_failure_reason
      << ",\"first_failed_speed_limit_frame\":" << result.first_failed_speed_limit_frame
      << ",\"first_failed_speed_limit_support\":" << result.first_failed_speed_limit_support
      << ",\"first_failed_chassis_w\":" << formatDouble(result.first_failed_chassis_w)
      << ",\"first_failed_max_link_w\":" << formatDouble(result.first_failed_max_link_w)
      << ",\"first_failed_retry_count\":" << result.first_failed_retry_count
      << ",\"first_failed_held_state_count\":" << result.first_failed_held_state_count
      << ",\"first_failed_iterations\":" << result.first_failed_iterations
      << ",\"first_failed_ncp_dual\":" << formatDouble(result.first_failed_ncp_dual)
      << ",\"first_failed_ncp_comp\":" << formatDouble(result.first_failed_ncp_comp)
      << ",\"first_failed_admm_rho\":" << formatDouble(result.first_failed_admm_rho)
      << ",\"first_failed_contact_constraint_count\":"
      << result.first_failed_contact_constraint_count
      << ",\"first_failed_worst_contact_id\":" << result.first_failed_worst_contact_id
      << ",\"recovered_speed_limit_streak_before_first_hold\":"
      << result.recovered_speed_limit_streak_before_first_hold
      << ",\"recovered_speed_limit_frame_before_first_hold\":"
      << result.recovered_speed_limit_frame_before_first_hold
      << ",\"recovered_speed_limit_support_before_first_hold\":"
      << result.recovered_speed_limit_support_before_first_hold
      << ",\"recovered_ncp_streak_before_first_hold\":"
      << result.recovered_ncp_streak_before_first_hold
      << ",\"solver_rollbacks\":"
      << (result.solver_rollback_count_end - result.solver_rollback_count_start)
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
                                     const double average_ratio,
                                     const bool commanded_yaw_direction_match) {
    std::ostringstream o;
    using locomotion_test::formatDouble;
    o << "{\"walk_path_length_m\":" << formatDouble(result.walk_path_length_m)
      << ",\"net_horizontal_distance_m\":" << formatDouble(horizontal_distance)
      << ",\"delta_x_m\":" << formatDouble(delta.x) << ",\"delta_y_m\":" << formatDouble(delta.y)
      << ",\"delta_z_m\":" << formatDouble(delta.z)
      << ",\"yaw_delta_rad\":" << formatDouble(yaw_delta)
      << ",\"average_yaw_rate_radps\":" << formatDouble(result.average_yaw_rate_radps)
      << ",\"peak_yaw_rate_radps\":" << formatDouble(result.peak_yaw_rate_radps)
      << ",\"minimum_body_height_m\":" << formatDouble(result.minimum_body_height_m)
      << ",\"maximum_body_height_m\":" << formatDouble(result.maximum_body_height_m)
      << ",\"maximum_body_height_error_m\":"
      << formatDouble(result.maximum_body_height_error_m)
      << ",\"maximum_servo_tracking_error_rad\":"
      << formatDouble(result.maximum_servo_tracking_error_rad)
      << ",\"servo_tracking_error_rms_rad\":"
      << formatDouble(result.servo_tracking_error_samples == 0
              ? 0.0
              : std::sqrt(result.servo_tracking_error_squared_sum
                  / static_cast<double>(result.servo_tracking_error_samples)))
      << ",\"maximum_servo_target_rate_radps\":"
      << formatDouble(result.maximum_servo_target_rate_radps)
      << ",\"maximum_servo_target_rate_by_joint_radps\":["
      << formatDouble(result.maximum_servo_target_rate_by_joint_radps[0]) << ','
      << formatDouble(result.maximum_servo_target_rate_by_joint_radps[1]) << ','
      << formatDouble(result.maximum_servo_target_rate_by_joint_radps[2]) << ']'
      << ",\"servo_target_rate_above_no_load_samples\":"
      << result.servo_target_rate_above_no_load_samples
      << ",\"servo_target_rate_samples\":" << result.servo_target_rate_samples;
    appendUintArray(o, ",\"planned_swing_samples\":", result.planned_swing_samples);
    appendUintArray(o, ",\"planned_swing_contact_samples\":", result.planned_swing_contact_samples);
    appendUintArray(o, ",\"planned_stance_samples\":", result.planned_stance_samples);
    appendUintArray(o, ",\"planned_stance_no_contact_samples\":",
                    result.planned_stance_no_contact_samples);
    appendUintArray(o, ",\"max_liftoff_delay_samples\":", result.max_liftoff_delay_samples);
    result.swing_events.json(o);
    o << ",\"max_loaded_swing_joint_error_rad\":"
      << formatDouble(result.max_loaded_swing_joint_error_rad)
      << ",\"max_planned_swing_measured_foot_clearance_m\":"
      << formatDouble(result.max_planned_swing_measured_foot_clearance_m)
      << ",\"mean_planned_stance_joint_error_rad\":["
      << formatDouble(result.mean_planned_stance_joint_error_rad[0]) << ','
      << formatDouble(result.mean_planned_stance_joint_error_rad[1]) << ','
      << formatDouble(result.mean_planned_stance_joint_error_rad[2]) << "]"
      << ",\"raw_contact_count_sum\":" << result.raw_contact_count_sum
      << ",\"planned_stance_count_sum\":" << result.planned_stance_count_sum
      << ",\"support_census_samples\":" << result.support_census_samples
      << ",\"walk_mode_steps\":" << result.walk_mode_steps
      << ",\"non_walk_mode_steps\":" << result.non_walk_mode_steps
      << ",\"faulted_steps\":" << result.faulted_steps
      << ",\"solver_healthy_steps\":" << result.solver_healthy_steps
      << ",\"solver_recovered_steps\":" << result.solver_recovered_steps
      << ",\"solver_held_steps\":" << result.solver_held_steps
      << ",\"solver_unsupported_steps\":" << result.solver_unsupported_steps
      << ",\"solver_not_converged_steps\":" << result.solver_not_converged_steps
      << ",\"solver_recovered_failure_reason\":\"";
    appendFailureReasonHistogram(o, result.solver_recovered_failure_reason);
    o << "\",\"solver_held_failure_reason\":\"";
    appendFailureReasonHistogram(o, result.solver_held_failure_reason);
    o << "\",\"solver_speed_limit_frame\":\"";
    appendSpeedLimitFrameHistogram(o, result.solver_speed_limit_frame);
    o << "\",\"solver_speed_limit_swing_steps\":" << result.solver_speed_limit_swing_steps
      << ",\"solver_speed_limit_stance_steps\":" << result.solver_speed_limit_stance_steps
      << ",\"max_solver_iterations\":" << result.max_solver_iterations
      << ",\"p99_solver_iterations\":" << result.p99_solver_iterations
      << ",\"max_solver_primal_residual\":"
      << formatDouble(result.max_solver_primal_residual, 12)
      << ",\"max_solver_dual_residual\":"
      << formatDouble(result.max_solver_dual_residual, 12)
      << ",\"max_solver_complementarity_residual\":"
      << formatDouble(result.max_solver_complementarity_residual, 12)
      << ",\"max_solver_ncp_dual_residual\":"
      << formatDouble(result.max_solver_ncp_dual_residual, 12)
      << ",\"max_solver_ncp_complementarity_residual\":"
      << formatDouble(result.max_solver_ncp_complementarity_residual, 12)
      << ",\"peak_solver_normal_impulse\":"
      << formatDouble(result.peak_solver_normal_impulse, 12)
      << ",\"max_solver_contact_penetration\":"
      << formatDouble(result.max_solver_contact_penetration, 12)
      << ",\"p99_solver_contact_penetration\":"
      << formatDouble(result.p99_solver_contact_penetration, 12)
      << ",\"max_solver_mechanical_energy_delta_abs\":"
      << formatDouble(result.max_solver_mechanical_energy_delta_abs, 12)
      << ",\"sum_solver_actuator_work\":"
      << formatDouble(result.sum_solver_actuator_work, 12)
      << ",\"max_solver_compliant_projected_residual\":"
      << formatDouble(result.max_solver_compliant_projected_residual, 12)
      << ",\"p99_solver_compliant_projected_residual\":"
      << formatDouble(result.p99_solver_compliant_projected_residual, 12)
      << ",\"peak_solver_friction_impulse\":"
      << formatDouble(result.peak_solver_friction_impulse, 12)
      << ",\"peak_solver_actuator_impulse\":"
      << formatDouble(result.peak_solver_actuator_impulse, 12)
      << ",\"peak_solver_servo_torque_utilization\":"
      << formatDouble(result.peak_solver_servo_torque_utilization, 12)
      << ",\"peak_solver_preintegration_linear_speed\":"
      << formatDouble(result.peak_solver_preintegration_linear_speed, 12)
      << ",\"peak_solver_preintegration_angular_speed\":"
      << formatDouble(result.peak_solver_preintegration_angular_speed, 12)
      << ",\"peak_solver_chassis_preintegration_angular_speed\":"
      << formatDouble(result.peak_solver_chassis_preintegration_angular_speed, 12)
      << ",\"peak_solver_max_link_preintegration_angular_speed\":"
      << formatDouble(result.peak_solver_max_link_preintegration_angular_speed, 12)
      << ",\"max_consecutive_held_steps\":" << result.max_consecutive_held_steps
      << ",\"first_non_walk_step\":" << result.first_non_walk_step
      << ",\"first_fault_step\":" << result.first_fault_step
      << ",\"first_fault\":" << static_cast<int>(result.first_fault)
      << ",\"first_read_fail\":" << (result.first_read_fail ? "true" : "false")
      << ",\"first_failed_solver_status\":" << result.first_failed_solver_status
      << ",\"first_failed_failure_reason\":" << result.first_failed_failure_reason
      << ",\"first_failed_speed_limit_frame\":" << result.first_failed_speed_limit_frame
      << ",\"first_failed_speed_limit_support\":" << result.first_failed_speed_limit_support
      << ",\"first_failed_chassis_w\":" << formatDouble(result.first_failed_chassis_w)
      << ",\"first_failed_max_link_w\":" << formatDouble(result.first_failed_max_link_w)
      << ",\"first_failed_retry_count\":" << result.first_failed_retry_count
      << ",\"first_failed_held_state_count\":" << result.first_failed_held_state_count
      << ",\"first_failed_iterations\":" << result.first_failed_iterations
      << ",\"first_failed_ncp_dual\":" << formatDouble(result.first_failed_ncp_dual)
      << ",\"first_failed_ncp_comp\":" << formatDouble(result.first_failed_ncp_comp)
      << ",\"first_failed_admm_rho\":" << formatDouble(result.first_failed_admm_rho)
      << ",\"first_failed_contact_constraint_count\":"
      << result.first_failed_contact_constraint_count
      << ",\"first_failed_worst_contact_id\":" << result.first_failed_worst_contact_id
      << ",\"recovered_speed_limit_streak_before_first_hold\":"
      << result.recovered_speed_limit_streak_before_first_hold
      << ",\"recovered_speed_limit_frame_before_first_hold\":"
      << result.recovered_speed_limit_frame_before_first_hold
      << ",\"recovered_speed_limit_support_before_first_hold\":"
      << result.recovered_speed_limit_support_before_first_hold
      << ",\"recovered_ncp_streak_before_first_hold\":"
      << result.recovered_ncp_streak_before_first_hold
      << ",\"solver_rollbacks\":"
      << (result.solver_rollback_count_end - result.solver_rollback_count_start)
      << ",\"commanded_yaw_rate_radps\":" << formatDouble(commanded_yaw_rate)
      << ",\"raw_wz_radps\":" << formatDouble(result.raw_wz_radps)
      << ",\"raw_vx_mps\":" << formatDouble(result.raw_vx_mps)
      << ",\"raw_vy_mps\":" << formatDouble(result.raw_vy_mps)
      << ",\"intent_cmd_vx_mps\":" << formatDouble(result.intent_cmd_vx_mps)
      << ",\"intent_cmd_vy_mps\":" << formatDouble(result.intent_cmd_vy_mps)
      << ",\"intent_cmd_yaw_radps\":" << formatDouble(result.intent_cmd_yaw_radps)
      << ",\"intent_twist_vx_mps\":" << formatDouble(result.intent_twist_vx_mps)
      << ",\"intent_twist_vy_mps\":" << formatDouble(result.intent_twist_vy_mps)
      << ",\"intent_twist_wz_radps\":" << formatDouble(result.intent_twist_wz_radps)
      << ",\"planar_vx_mps\":" << formatDouble(result.planar_vx_mps)
      << ",\"planar_vy_mps\":" << formatDouble(result.planar_vy_mps)
      << ",\"planar_yaw_rate_radps\":" << formatDouble(result.planar_yaw_rate_radps)
      << ",\"intent_planar_mps\":" << formatDouble(result.intent_planar_mps)
      << ",\"yaw_equiv_mps\":" << formatDouble(result.yaw_equiv_mps)
      << ",\"yaw_dominant\":" << (result.yaw_dominant ? "true" : "false")
      << ",\"start_x_m\":" << formatDouble(result.start_position.x)
      << ",\"start_y_m\":" << formatDouble(result.start_position.y)
      << ",\"end_x_m\":" << formatDouble(result.end_position.x)
      << ",\"end_y_m\":" << formatDouble(result.end_position.y)
      << ",\"stand_end_body_vx_mps\":" << formatDouble(result.stand_end_body_vx_mps)
      << ",\"stand_end_body_vy_mps\":" << formatDouble(result.stand_end_body_vy_mps)
      << ",\"stand_end_body_wz_radps\":" << formatDouble(result.stand_end_body_wz_radps)
      << ",\"first_walk_gait_phase_leg0\":"
      << formatDouble(result.first_walk_gait_phase_leg0)
      << ",\"first_walk_requested_planar_speed_mps\":"
      << formatDouble(result.first_walk_requested_planar_speed_mps)
      << ",\"first_walk_governed_planar_speed_mps\":"
      << formatDouble(result.first_walk_governed_planar_speed_mps)
      << ",\"first_walk_requested_yaw_rate_radps\":"
      << formatDouble(result.first_walk_requested_yaw_rate_radps)
      << ",\"first_walk_governed_yaw_rate_radps\":"
      << formatDouble(result.first_walk_governed_yaw_rate_radps)
      << ",\"captured_first_walk_frame\":"
      << (result.captured_first_walk_frame ? "true" : "false");
    const double abs_yaw = std::abs(yaw_delta);
    const bool radius_valid = abs_yaw > 0.1;
    const double equivalent_radius_m =
        radius_valid ? (horizontal_distance / (2.0 * std::sin(abs_yaw * 0.5))) : 0.0;
    const double path_per_rad_m =
        abs_yaw > 1.0e-9 ? (result.walk_path_length_m / abs_yaw) : 0.0;
    o << ",\"equivalent_turn_radius_m\":" << formatDouble(equivalent_radius_m)
      << ",\"equivalent_turn_radius_valid\":" << (radius_valid ? "true" : "false")
      << ",\"path_per_rad_m\":" << formatDouble(path_per_rad_m)
      << ",\"held_count\":" << result.solver_held_steps
      << ",\"mean_loaded_stance_commanded_sweep_mps\":[";
    for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
        o << (leg == 0 ? "" : ",")
          << formatDouble(result.mean_loaded_stance_commanded_sweep_mps[leg]);
    }
    o << "],\"mean_loaded_stance_measured_sweep_mps\":[";
    for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
        o << (leg == 0 ? "" : ",")
          << formatDouble(result.mean_loaded_stance_measured_sweep_mps[leg]);
    }
    o << "],\"mean_loaded_stance_world_slip_mps\":[";
    for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
        o << (leg == 0 ? "" : ",")
          << formatDouble(result.mean_loaded_stance_world_slip_mps[leg]);
    }
    o << ']'
      << ",\"average_yaw_rate_ratio\":" << formatDouble(average_ratio)
      << ",\"commanded_yaw_direction_match\":"
      << (commanded_yaw_direction_match ? "true" : "false")
      << ",\"final_mode\":" << static_cast<int>(result.final_status.active_mode)
      << ",\"final_fault\":" << static_cast<int>(result.final_status.active_fault) << '}';
    return o.str();
}

template <typename MotionT>
MotionRunResult runMotionSequence(RobotRuntime& runtime,
                                  CapturingPhysicsSimBridge& bridge,
                                  const ScenarioMotionIntent& stand_motion,
                                  const MotionT& motion,
                                  const int bus_loop_period_us,
                                  const bool arm_turn_entry_plant = false) {
    const int kStandWarmupSteps = static_cast<int>(
        physics_sim_test_utils::scaledLegacyStepCount(100, bus_loop_period_us));
    int walk_steps = static_cast<int>(
        physics_sim_test_utils::scaledLegacyStepCount(600, bus_loop_period_us));
    if (const char* value = std::getenv("HEXAPOD_WALK_TEST_STEP_LIMIT")) {
        char* end = nullptr;
        const long parsed = std::strtol(value, &end, 10);
        if (end != value && *end == '\0' && parsed > 0) {
            walk_steps = std::min(walk_steps, static_cast<int>(parsed));
        }
    }
    // Keep the synthetic stream ahead of wall time so slow instrumented solver builds do not
    // trip the runtime freshness gate while preserving an exact per-step command cadence.
    TimePointUs command_time{now_us().value + 3'600'000'000ULL};
    const auto advance_command_time = [&]() {
        command_time.value += static_cast<uint64_t>(bus_loop_period_us);
        return command_time;
    };

    for (int i = 0; i < kStandWarmupSteps; ++i) {
        runControlLoopStep(runtime, stand_motion, advance_command_time());
    }
    bridge.clearFirstFailedSolverTelemetry();

    if (!bridge.last_state().has_value()) {
        throw std::runtime_error("bridge never produced an initial state");
    }

    MotionRunResult result{};
    if (bridge.last_solver_telemetry().has_value()) {
        result.solver_rollback_count_start =
            bridge.last_solver_telemetry()->rollback_count;
    }
    result.start_position = positionFromState(bridge.last_state().value());
    result.minimum_body_height_m = result.start_position.z;
    result.maximum_body_height_m = result.start_position.z;
    result.maximum_body_height_error_m = std::abs(
        result.start_position.z - stand_motion.body_height_m);
    result.start_yaw_rad = bridge.last_state().value().body_twist_state.twist_pos_rad.z;
    fillWalkCommandCensus(result, asMotionIntent(motion));
    result.stand_end_body_vx_mps = bridge.last_state().value().body_twist_state.body_trans_mps.x;
    result.stand_end_body_vy_mps = bridge.last_state().value().body_twist_state.body_trans_mps.y;
    result.stand_end_body_wz_radps = bridge.last_state().value().body_twist_state.twist_vel_radps.z;
    fillTurnEntrySnapshot(result.stand_end_entry,
                          bridge.last_state().value(),
                          runtime.locomotionDebugSnapshot());
    result.captured_stand_end_entry = true;
    const bool rebase_start_after_restore =
        std::getenv("HEXAPOD_PINOCCHIO_CUTPOINT_RESTORE_PATH") != nullptr
        && std::getenv("HEXAPOD_PINOCCHIO_CUTPOINT_RESTORE_PATH")[0] != '\0';
    if (arm_turn_entry_plant) {
        maybeWriteTurnEntryControllerDump(runtime);
        maybeRestoreTurnEntryController(runtime);
        maybeArmCutpointDump();
        maybeArmCutpointRestore();
    }
    Vec3 previous_position = result.start_position;
    double horizontal_speed_sum = 0.0;
    double yaw_speed_sum = 0.0;
    std::vector<Vec3> walk_samples{};
    walk_samples.reserve(static_cast<std::size_t>(walk_steps));
    const bool collect_turn_traj = std::getenv("HEXAPOD_TURN_TRAJ_DUMP_PATH") != nullptr
        && std::getenv("HEXAPOD_TURN_TRAJ_DUMP_PATH")[0] != '\0';
    if (collect_turn_traj) {
        result.turn_traj_ticks.reserve(static_cast<std::size_t>(walk_steps));
    }
    const bool collect_support_divergence =
        std::getenv("HEXAPOD_SUPPORT_DIVERGENCE_DUMP_PATH") != nullptr
        && std::getenv("HEXAPOD_SUPPORT_DIVERGENCE_DUMP_PATH")[0] != '\0';
    constexpr std::size_t kSupportDivergenceRing = 256;
    std::deque<SupportDivergenceTick> support_divergence_ring{};
    bool support_divergence_frozen = false;
    std::vector<int> solver_iteration_samples{};
    solver_iteration_samples.reserve(static_cast<std::size_t>(walk_steps));
    std::vector<double> compliant_projected_residual_samples{};
    compliant_projected_residual_samples.reserve(static_cast<std::size_t>(walk_steps));
    std::vector<double> contact_penetration_samples{};
    contact_penetration_samples.reserve(static_cast<std::size_t>(walk_steps));
    JointTargets previous_applied_targets = bridge.applied_targets();
    std::array<std::uint64_t, kNumLegs> liftoff_delay_run{};
    telemetry::LocomotionDebugSnapshot previous_locomotion_debug =
        runtime.locomotionDebugSnapshot();
    GaitState previous_gait = runtime.gaitSnapshot();
    const double command_step_s = static_cast<double>(bus_loop_period_us) * 1.0e-6;
    int current_held_streak = 0;
    int recovered_speed_limit_streak = 0;
    int last_recovered_speed_limit_frame = 0;
    int last_recovered_speed_limit_support = 0;
    int recovered_ncp_streak = 0;
    std::array<double, kNumLegs> loaded_stance_commanded_sweep_sum{};
    std::array<double, kNumLegs> loaded_stance_pre_slew_sweep_sum{};
    std::array<double, kNumLegs> loaded_stance_measured_sweep_sum{};
    std::array<double, kNumLegs> loaded_stance_world_slip_sum{};
    std::array<double, kNumLegs> loaded_stance_tracking_error_sum{};
    std::array<double, kNumLegs> loaded_stance_post_clamp_distortion_sum{};
    std::array<double, kNumLegs> loaded_stance_latched_length_sum{};
    std::array<double, kNumLegs> loaded_stance_used_length_sum{};
    double body_forward_speed_sum = 0.0;
    double governed_planar_speed_sum = 0.0;
    double governed_yaw_rate_sum = 0.0;
    double governor_command_scale_sum = 0.0;
    double governor_cadence_scale_sum = 0.0;
    double gait_frequency_sum = 0.0;
    double gait_duty_factor_sum = 0.0;
    double gait_step_length_sum = 0.0;
    double body_tilt_sum = 0.0;
    double body_roll_sum = 0.0;
    double body_pitch_sum = 0.0;

    for (int i = 0; i < walk_steps; ++i) {
        runControlLoopStep(runtime, motion, advance_command_time());

        if (!bridge.last_state().has_value()) {
            throw std::runtime_error("bridge lost state during walk sequence");
        }

        const RobotState& state = bridge.last_state().value();
        const telemetry::LocomotionDebugSnapshot locomotion_debug =
            runtime.locomotionDebugSnapshot();
        const std::array<bool, kNumLegs> slew_clamp_hits = runtime.slewClampHitSnapshot();
        const std::array<bool, kNumLegs> workspace_hits = runtime.workspaceXyHitSnapshot();
        const std::array<bool, kNumLegs> stroke_hits = runtime.strokeClampHitSnapshot();
        const CommandGovernorState governor = runtime.commandGovernorSnapshot();
        const GaitState gait = runtime.gaitSnapshot();
        const Vec3 current_position = positionFromState(state);
        // Use the same fused pose as the debug FK, not the raw bridge pose.
        const RobotState census_est = runtime.estimatedSnapshot();
        BodyPose census_pose{};
        census_pose.position = census_est.body_twist_state.body_trans_m;
        census_pose.roll = AngleRad{census_est.body_twist_state.twist_pos_rad.x};
        census_pose.pitch = AngleRad{census_est.body_twist_state.twist_pos_rad.y};
        census_pose.yaw = AngleRad{census_est.body_twist_state.twist_pos_rad.z};
        const Mat3 census_rotation = census_pose.rotationBodyToWorld();
        const bool census_valid = locomotion_debug.valid && state.bus_ok
            && runtime.getStatus().active_mode == RobotMode::WALK;
        if (census_valid) {
            const auto& imu = census_est.imu;
            const double gyro = std::hypot(imu.gyro_radps.x, imu.gyro_radps.y, imu.gyro_radps.z);
            const double accel = std::hypot(imu.accel_mps2.x, imu.accel_mps2.y, imu.accel_mps2.z);
            const bool gyroOK = std::isfinite(gyro)
                && gyro <= control_config::kDefaultGravityFeedforwardMaxGyroRadps;
            const bool accelOK = std::isfinite(accel) && std::abs(accel - hexapod_dynamics::kStandardGravityMps2)
                <= control_config::kDefaultGravityFeedforwardAccelNormMarginMps2;
            const bool permitted = census_est.has_imu && imu.valid && gyroOK && accelOK;
            if (result.ff_observed_walk_samples && permitted != result.ff_previous_gate)
                ++result.ff_default_gate_transitions;
            result.ff_previous_gate = permitted;
            ++result.ff_observed_walk_samples;
            result.ff_default_imu_permitted_samples += permitted;
            result.ff_default_gyro_rejected_samples += !gyroOK;
            result.ff_default_accel_rejected_samples += !accelOK;
            for (bool valid : census_est.joint_stiffness_valid)
                result.ff_reported_stiffness_leg_samples += valid;
        }
        for (int leg = 0; leg < kNumLegs; ++leg) {
            swing_census::Sample sample{};
            sample.time_s = i * command_step_s;
            sample.stance = locomotion_debug.planned_stance[leg];
            sample.contact = locomotion_debug.raw_contact[leg];
            sample.body_z = census_pose.position.z;
            sample.rotation_z = {census_rotation.m[2][0], census_rotation.m[2][1], census_rotation.m[2][2]};
            sample.foot_body = locomotion_debug.measured_foot_body_m[leg];
            sample.command_body = locomotion_debug.post_clamp_fk_body_m[leg];
            sample.foot_world_z = locomotion_debug.measured_foot_world_m[leg].z;
            result.swing_events.update(leg, sample, census_valid);
        }
        if (i == 0) {
            result.captured_first_walk_frame = true;
            result.first_walk_gait_phase_leg0 = gait.phase[0];
            result.first_walk_requested_planar_speed_mps = governor.requested_planar_speed_mps;
            result.first_walk_governed_planar_speed_mps = governor.governed_planar_speed_mps;
            result.first_walk_requested_yaw_rate_radps = governor.requested_yaw_rate_radps;
            result.first_walk_governed_yaw_rate_radps = governor.governed_yaw_rate_radps;
            fillTurnEntrySnapshot(result.first_walk_entry, state, locomotion_debug);
            result.captured_first_walk_entry = true;
            if (rebase_start_after_restore) {
                result.start_position = current_position;
                result.start_yaw_rad = state.body_twist_state.twist_pos_rad.z;
                result.minimum_body_height_m = current_position.z;
                result.maximum_body_height_m = current_position.z;
                previous_position = current_position;
            }
        }
        result.minimum_body_height_m = std::min(
            result.minimum_body_height_m, current_position.z);
        result.maximum_body_height_m = std::max(
            result.maximum_body_height_m, current_position.z);
        result.maximum_body_height_error_m = std::max(
            result.maximum_body_height_error_m,
            std::abs(current_position.z - stand_motion.body_height_m));
        const JointTargets& applied_targets = bridge.applied_targets();
        for (std::size_t leg = 0; leg < state.leg_states.size(); ++leg) {
            for (std::size_t joint = 0;
                 joint < state.leg_states[leg].joint_state.size();
                 ++joint) {
                const double error = std::remainder(
                    applied_targets.leg_states[leg].joint_state[joint].pos_rad.value
                        - state.leg_states[leg].joint_state[joint].pos_rad.value,
                    6.28318530717958647692);
                result.maximum_servo_tracking_error_rad = std::max(
                    result.maximum_servo_tracking_error_rad, std::abs(error));
                result.servo_tracking_error_squared_sum += error * error;
                ++result.servo_tracking_error_samples;
                result.maximum_servo_tracking_error_by_joint_rad[joint] = std::max(
                    result.maximum_servo_tracking_error_by_joint_rad[joint],
                    std::abs(error));
                result.servo_tracking_error_squared_sum_by_joint[joint] += error * error;
                ++result.servo_tracking_error_samples_by_joint[joint];
                const double target_delta = std::remainder(
                    applied_targets.leg_states[leg].joint_state[joint].pos_rad.value
                        - previous_applied_targets.leg_states[leg]
                              .joint_state[joint].pos_rad.value,
                    6.28318530717958647692);
                const double target_rate = command_step_s > 0.0
                    ? std::abs(target_delta) / command_step_s
                    : 0.0;
                result.maximum_servo_target_rate_radps = std::max(
                    result.maximum_servo_target_rate_radps, target_rate);
                result.maximum_servo_target_rate_by_joint_radps[joint] = std::max(
                    result.maximum_servo_target_rate_by_joint_radps[joint], target_rate);
                ++result.servo_target_rate_samples;
                constexpr double kRateComparisonTolerance = 1.0e-6;
                if (target_rate
                    > hexapod_dynamics::kServoNoLoadSpeedRadPerSec
                        * (1.0 + kRateComparisonTolerance)) {
                    ++result.servo_target_rate_above_no_load_samples;
                }
            }
        }
        walk_samples.push_back(current_position);
        if (collect_turn_traj) {
            TurnTrajTick tick{};
            tick.x = current_position.x;
            tick.y = current_position.y;
            tick.z = current_position.z;
            tick.yaw = state.body_twist_state.twist_pos_rad.z;
            tick.vx = state.body_twist_state.body_trans_mps.x;
            tick.vy = state.body_twist_state.body_trans_mps.y;
            tick.wz = state.body_twist_state.twist_vel_radps.z;
            int fused = 0;
            int raw = 0;
            int planned = 0;
            int left_fused = 0;
            int right_fused = 0;
            for (int leg = 0; leg < kNumLegs; ++leg) {
                const std::size_t idx = static_cast<std::size_t>(leg);
                tick.fused_support[idx] = locomotion_debug.fused_support[idx] ? 1 : 0;
                tick.raw_contact[idx] = locomotion_debug.raw_contact[idx] ? 1 : 0;
                tick.planned_stance[idx] = locomotion_debug.planned_stance[idx] ? 1 : 0;
                if (locomotion_debug.fused_support[idx]) {
                    ++fused;
                    if (leg < 3) {
                        ++left_fused;
                    } else {
                        ++right_fused;
                    }
                }
                if (locomotion_debug.raw_contact[idx]) {
                    ++raw;
                }
                if (locomotion_debug.planned_stance[idx]) {
                    ++planned;
                }
                if (bridge.last_solver_telemetry().has_value()) {
                    tick.leg_contact_count[idx] = static_cast<int>(
                        bridge.last_solver_telemetry()->leg_contact_count[idx]);
                }
            }
            tick.fused_count = fused;
            tick.raw_count = raw;
            tick.planned_count = planned;
            tick.left_fused = left_fused;
            tick.right_fused = right_fused;
            tick.support = std::max(fused, governor.current_support_count);
            result.turn_traj_ticks.push_back(tick);
        }
        result.walk_path_length_m += std::hypot(current_position.x - previous_position.x,
                                                current_position.y - previous_position.y);
        previous_position = current_position;
        const double speed_xy = std::hypot(
            state.body_twist_state.body_trans_mps.x,
            state.body_twist_state.body_trans_mps.y);
        horizontal_speed_sum += speed_xy;
        const double start_yaw_cos = std::cos(result.start_yaw_rad);
        const double start_yaw_sin = std::sin(result.start_yaw_rad);
        body_forward_speed_sum +=
            state.body_twist_state.body_trans_mps.x * start_yaw_cos
            + state.body_twist_state.body_trans_mps.y * start_yaw_sin;
        governed_planar_speed_sum += governor.governed_planar_speed_mps;
        governed_yaw_rate_sum += governor.governed_yaw_rate_radps;
        governor_command_scale_sum += governor.command_scale;
        governor_cadence_scale_sum += governor.cadence_scale;
        gait_frequency_sum += gait.stride_phase_rate_hz.value;
        gait_duty_factor_sum += gait.duty_factor;
        gait_step_length_sum += gait.step_length_m;
        body_tilt_sum += governor.body_tilt_rad;
        const double body_roll_rad = state.body_twist_state.twist_pos_rad.x;
        const double body_pitch_rad = state.body_twist_state.twist_pos_rad.y;
        body_roll_sum += body_roll_rad;
        body_pitch_sum += body_pitch_rad;
        result.min_body_pitch_rad = std::min(result.min_body_pitch_rad, body_pitch_rad);
        result.max_body_pitch_rad = std::max(result.max_body_pitch_rad, body_pitch_rad);
        result.max_body_tilt_rad = std::max(result.max_body_tilt_rad, governor.body_tilt_rad);
        result.body_tilt_above_stability_hold_samples += governor.body_tilt_rad > 0.20 ? 1U : 0U;
        result.governor_freeze_samples += governor.freeze_phase ? 1U : 0U;
        const std::size_t recovery_stage = static_cast<std::size_t>(governor.recovery_stage);
        if (recovery_stage < result.governor_recovery_stage_samples.size()) {
            ++result.governor_recovery_stage_samples[recovery_stage];
        }
        result.peak_horizontal_speed_mps = std::max(result.peak_horizontal_speed_mps, speed_xy);
        const double yaw_rate = std::abs(state.body_twist_state.twist_vel_radps.z);
        yaw_speed_sum += yaw_rate;
        result.peak_yaw_rate_radps = std::max(result.peak_yaw_rate_radps, yaw_rate);
        result.final_status = runtime.getStatus();

        if (locomotion_debug.valid && result.final_status.active_mode == RobotMode::WALK) {
            ++result.support_census_samples;
            for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
                const bool planned_stance = locomotion_debug.planned_stance[leg];
                const bool raw_contact = locomotion_debug.raw_contact[leg];
                result.raw_contact_count_sum += raw_contact ? 1U : 0U;
                result.planned_stance_count_sum += planned_stance ? 1U : 0U;
                if (planned_stance) {
                    ++result.planned_stance_samples[leg];
                    result.planned_stance_no_contact_samples[leg] += raw_contact ? 0U : 1U;
                    liftoff_delay_run[leg] = 0;
                    bool finite_error = true;
                    std::array<double, 3> abs_error{};
                    for (std::size_t joint = 0; joint < kJointsPerLeg; ++joint) {
                        const double error = std::remainder(
                            applied_targets.leg_states[leg].joint_state[joint].pos_rad.value
                                - state.leg_states[leg].joint_state[joint].pos_rad.value,
                            6.28318530717958647692);
                        if (!std::isfinite(error)) {
                            finite_error = false;
                            break;
                        }
                        abs_error[joint] = std::abs(error);
                    }
                    if (finite_error) {
                        ++result.planned_stance_joint_error_samples;
                        for (std::size_t joint = 0; joint < kJointsPerLeg; ++joint) {
                            result.planned_stance_joint_error_sum[joint] += abs_error[joint];
                        }
                    }
                } else {
                    ++result.planned_swing_samples[leg];
                    if (raw_contact) {
                        ++result.planned_swing_contact_samples[leg];
                        ++liftoff_delay_run[leg];
                        result.max_liftoff_delay_samples[leg] = std::max(
                            result.max_liftoff_delay_samples[leg], liftoff_delay_run[leg]);
                        for (std::size_t joint = 0; joint < kJointsPerLeg; ++joint) {
                            const double error = std::remainder(
                                applied_targets.leg_states[leg].joint_state[joint].pos_rad.value
                                    - state.leg_states[leg].joint_state[joint].pos_rad.value,
                                6.28318530717958647692);
                            if (std::isfinite(error)) {
                                result.max_loaded_swing_joint_error_rad = std::max(
                                    result.max_loaded_swing_joint_error_rad, std::abs(error));
                            }
                        }
                    } else {
                        liftoff_delay_run[leg] = 0;
                    }
                    const double clearance = locomotion_debug.measured_foot_world_m[leg].z;
                    if (std::isfinite(clearance)) {
                        result.max_planned_swing_measured_foot_clearance_m = std::max(
                            result.max_planned_swing_measured_foot_clearance_m, clearance);
                    }
                }
            }
        }

        if (locomotion_debug.valid && previous_locomotion_debug.valid && command_step_s > 0.0) {
            for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
                const bool continuously_loaded_stance =
                    locomotion_debug.planned_stance[leg]
                    && previous_locomotion_debug.planned_stance[leg]
                    && locomotion_debug.fused_load_bearing[leg]
                    && previous_locomotion_debug.fused_load_bearing[leg];
                if (!continuously_loaded_stance) {
                    continue;
                }

                loaded_stance_commanded_sweep_sum[leg] +=
                    -(locomotion_debug.post_clamp_fk_body_m[leg].x
                        - previous_locomotion_debug.post_clamp_fk_body_m[leg].x)
                    / command_step_s;
                loaded_stance_pre_slew_sweep_sum[leg] +=
                    -(locomotion_debug.pre_slew_fk_body_m[leg].x
                        - previous_locomotion_debug.pre_slew_fk_body_m[leg].x)
                    / command_step_s;
                loaded_stance_measured_sweep_sum[leg] +=
                    -(locomotion_debug.measured_foot_body_m[leg].x
                        - previous_locomotion_debug.measured_foot_body_m[leg].x)
                    / command_step_s;
                const Vec3 world_delta =
                    locomotion_debug.measured_foot_world_m[leg]
                    - previous_locomotion_debug.measured_foot_world_m[leg];
                loaded_stance_world_slip_sum[leg] +=
                    (world_delta.x * start_yaw_cos + world_delta.y * start_yaw_sin)
                    / command_step_s;
                loaded_stance_tracking_error_sum[leg] +=
                    locomotion_debug.commanded_tracking_error_m[leg];
                loaded_stance_post_clamp_distortion_sum[leg] +=
                    locomotion_debug.post_clamp_distortion_m[leg];
                loaded_stance_latched_length_sum[leg] +=
                    locomotion_debug.latched_stroke_length_m[leg];
                loaded_stance_used_length_sum[leg] +=
                    locomotion_debug.latched_stroke_used_m[leg];
                result.loaded_stance_slew_limited_samples[leg] +=
                    slew_clamp_hits[leg] ? 1U : 0U;
                result.loaded_stance_workspace_limited_samples[leg] +=
                    workspace_hits[leg] ? 1U : 0U;
                result.loaded_stance_stroke_limited_samples[leg] +=
                    stroke_hits[leg] ? 1U : 0U;
                result.loaded_stance_hold_samples[leg] +=
                    locomotion_debug.hold_stance[leg] ? 1U : 0U;
                if (locomotion_debug.hold_stance[leg]) {
                    result.loaded_stance_hold_phase_min[leg] = std::min(
                        result.loaded_stance_hold_phase_min[leg], gait.phase[leg]);
                    result.loaded_stance_hold_phase_max[leg] = std::max(
                        result.loaded_stance_hold_phase_max[leg], gait.phase[leg]);
                }
                ++result.loaded_stance_samples[leg];
            }
        }
        if (result.final_status.active_mode == RobotMode::WALK) {
            ++result.walk_mode_steps;
        } else {
            ++result.non_walk_mode_steps;
            if (result.first_non_walk_step < 0) {
                result.first_non_walk_step = i;
            }
        }
        if (result.final_status.active_fault != FaultCode::NONE) {
            ++result.faulted_steps;
            if (result.first_fault_step < 0) {
                result.first_fault_step = i;
                result.first_fault = result.final_status.active_fault;
            }
        }
        if (bridge.last_solver_telemetry().has_value()) {
            const PhysicsSimSolverTelemetry& solver = bridge.last_solver_telemetry().value();
            switch (solver.status) {
                case physics_sim::SolverStatus::Healthy:
                    ++result.solver_healthy_steps;
                    recovered_speed_limit_streak = 0;
                    recovered_ncp_streak = 0;
                    break;
                case physics_sim::SolverStatus::RecoveredRetry:
                    ++result.solver_recovered_steps;
                    {
                        const std::size_t reason = static_cast<std::size_t>(solver.failure_reason);
                        if (reason < result.solver_recovered_failure_reason.size()) {
                            ++result.solver_recovered_failure_reason[reason];
                        }
                    }
                    if (solver.failure_reason == physics_sim::SolverFailureReason::SpeedLimit) {
                        ++recovered_speed_limit_streak;
                        last_recovered_speed_limit_frame =
                            static_cast<int>(solver.speed_limit_frame);
                        last_recovered_speed_limit_support =
                            static_cast<int>(solver.speed_limit_support);
                        recovered_ncp_streak = 0;
                    } else if (
                        solver.failure_reason
                        == physics_sim::SolverFailureReason::SolverNotConverged) {
                        ++recovered_ncp_streak;
                        recovered_speed_limit_streak = 0;
                    } else {
                        recovered_speed_limit_streak = 0;
                        recovered_ncp_streak = 0;
                    }
                    break;
                case physics_sim::SolverStatus::HeldLastGood:
                    ++result.solver_held_steps;
                    ++current_held_streak;
                    result.max_consecutive_held_steps = std::max(
                        result.max_consecutive_held_steps, current_held_streak);
                    {
                        const std::size_t reason = static_cast<std::size_t>(solver.failure_reason);
                        if (reason < result.solver_held_failure_reason.size()) {
                            ++result.solver_held_failure_reason[reason];
                        }
                    }
                    if (result.solver_held_steps == 1) {
                        result.recovered_speed_limit_streak_before_first_hold =
                            recovered_speed_limit_streak;
                        result.recovered_speed_limit_frame_before_first_hold =
                            last_recovered_speed_limit_frame;
                        result.recovered_speed_limit_support_before_first_hold =
                            last_recovered_speed_limit_support;
                        result.recovered_ncp_streak_before_first_hold = recovered_ncp_streak;
                        if (const char* trace = std::getenv(
                                "HEXAPOD_WALK_TEST_TRACE_FIRST_HOLD");
                            trace != nullptr && trace[0] != '\0' && trace[0] != '0') {
                            std::cerr << "[walk-first-held-gait] step=" << i
                                      << " reason="
                                      << static_cast<unsigned>(solver.failure_reason)
                                      << " duty=" << gait.duty_factor
                                      << " legs=";
                            for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
                                const Vec3& cmd =
                                    previous_locomotion_debug.commanded_foot_body_m[leg];
                                const Vec3& planned =
                                    previous_locomotion_debug.planned_leg_target_body_m[leg];
                                const Vec3& cmdVel =
                                    previous_locomotion_debug.post_clamp_fk_vel_body_mps[leg];
                                const Vec3& measured =
                                    previous_locomotion_debug.measured_foot_body_m[leg];
                                double maxTargetRate = 0.0;
                                for (std::size_t joint = 0; joint < 3; ++joint) {
                                    const double delta = std::remainder(
                                        applied_targets.leg_states[leg].joint_state[joint]
                                                .pos_rad.value
                                            - previous_applied_targets.leg_states[leg]
                                                  .joint_state[joint].pos_rad.value,
                                        6.28318530717958647692);
                                    maxTargetRate = std::max(
                                        maxTargetRate,
                                        command_step_s > 0.0
                                            ? std::abs(delta) / command_step_s
                                            : 0.0);
                                }
                                std::cerr << (leg == 0 ? "[" : ",")
                                          << leg << ':' << previous_gait.phase[leg] << ':'
                                          << (previous_locomotion_debug.planned_stance[leg] ? 1 : 0)
                                          << ':' << (previous_locomotion_debug.raw_contact[leg] ? 1 : 0)
                                          << ':'
                                          << (previous_locomotion_debug.fused_load_bearing[leg] ? 1 : 0)
                                          << ':' << static_cast<unsigned>(
                                                 previous_locomotion_debug.fused_contact_phase[leg])
                                          << ':' << planned.x << ':' << planned.y << ':'
                                          << planned.z
                                          << ':' << cmd.x << ':' << cmd.y << ':' << cmd.z
                                          << ':' << cmdVel.x << ':' << cmdVel.y << ':' << cmdVel.z
                                          << ':' << measured.x << ':' << measured.y << ':'
                                          << measured.z << ':'
                                          << previous_locomotion_debug.commanded_tracking_error_m[leg]
                                          << ':'
                                          << previous_locomotion_debug.post_clamp_distortion_m[leg]
                                          << ':' << maxTargetRate;
                            }
                            std::cerr << "]\n";
                        }
                    }
                    recovered_speed_limit_streak = 0;
                    recovered_ncp_streak = 0;
                    break;
                case physics_sim::SolverStatus::UnsupportedIsland:
                    ++result.solver_unsupported_steps;
                    current_held_streak = 0;
                    break;
            }
            if (solver.status != physics_sim::SolverStatus::HeldLastGood
                && solver.status != physics_sim::SolverStatus::UnsupportedIsland) {
                current_held_streak = 0;
            }
            if (solver.failure_reason == physics_sim::SolverFailureReason::SpeedLimit) {
                const std::size_t frame = static_cast<std::size_t>(solver.speed_limit_frame);
                if (frame < result.solver_speed_limit_frame.size()) {
                    ++result.solver_speed_limit_frame[frame];
                }
                if (solver.speed_limit_support
                    == static_cast<std::uint8_t>(physics_sim::SolverSpeedLimitSupport::Swing)) {
                    ++result.solver_speed_limit_swing_steps;
                } else if (
                    solver.speed_limit_support
                    == static_cast<std::uint8_t>(physics_sim::SolverSpeedLimitSupport::Stance)) {
                    ++result.solver_speed_limit_stance_steps;
                }
            }
            if (solver.failure_reason == physics_sim::SolverFailureReason::SolverNotConverged) {
                ++result.solver_not_converged_steps;
            }
            result.max_solver_iterations = std::max(
                result.max_solver_iterations, static_cast<int>(solver.iterations));
            solver_iteration_samples.push_back(static_cast<int>(solver.iterations));
            result.max_solver_primal_residual = std::max(
                result.max_solver_primal_residual,
                static_cast<double>(solver.primal_residual));
            result.max_solver_dual_residual = std::max(
                result.max_solver_dual_residual,
                static_cast<double>(solver.dual_residual));
            result.max_solver_complementarity_residual = std::max(
                result.max_solver_complementarity_residual,
                static_cast<double>(solver.complementarity_residual));
            result.max_solver_ncp_dual_residual = std::max(
                result.max_solver_ncp_dual_residual,
                static_cast<double>(solver.ncp_dual_residual));
            result.max_solver_ncp_complementarity_residual = std::max(
                result.max_solver_ncp_complementarity_residual,
                static_cast<double>(solver.ncp_complementarity_residual));
            result.peak_solver_normal_impulse = std::max(
                result.peak_solver_normal_impulse,
                static_cast<double>(solver.peak_normal_impulse));
            result.max_solver_contact_penetration = std::max(
                result.max_solver_contact_penetration,
                static_cast<double>(solver.max_contact_penetration));
            contact_penetration_samples.push_back(
                static_cast<double>(solver.max_contact_penetration));
            result.max_solver_mechanical_energy_delta_abs = std::max(
                result.max_solver_mechanical_energy_delta_abs,
                std::abs(static_cast<double>(solver.mechanical_energy_delta)));
            result.sum_solver_actuator_work += static_cast<double>(solver.actuator_work);
            result.max_solver_compliant_projected_residual = std::max(
                result.max_solver_compliant_projected_residual,
                static_cast<double>(solver.compliant_projected_residual));
            compliant_projected_residual_samples.push_back(
                static_cast<double>(solver.compliant_projected_residual));
            result.peak_solver_friction_impulse = std::max(
                result.peak_solver_friction_impulse,
                static_cast<double>(solver.peak_friction_impulse));
            result.peak_solver_actuator_impulse = std::max(
                result.peak_solver_actuator_impulse,
                static_cast<double>(solver.peak_actuator_impulse));
            result.peak_solver_servo_torque_utilization = std::max(
                result.peak_solver_servo_torque_utilization,
                static_cast<double>(solver.peak_servo_torque_utilization));
            result.peak_solver_preintegration_linear_speed = std::max(
                result.peak_solver_preintegration_linear_speed,
                static_cast<double>(solver.preintegration_linear_speed));
            result.peak_solver_preintegration_angular_speed = std::max(
                result.peak_solver_preintegration_angular_speed,
                static_cast<double>(solver.preintegration_angular_speed));
            result.peak_solver_chassis_preintegration_angular_speed = std::max(
                result.peak_solver_chassis_preintegration_angular_speed,
                static_cast<double>(solver.chassis_preintegration_angular_speed));
            result.peak_solver_max_link_preintegration_angular_speed = std::max(
                result.peak_solver_max_link_preintegration_angular_speed,
                static_cast<double>(solver.max_link_preintegration_angular_speed));
            result.solver_rollback_count_end = solver.rollback_count;
        }
        if (collect_support_divergence && !support_divergence_frozen) {
            const bool walk_tick = result.final_status.active_mode == RobotMode::WALK;
            const bool held_tick =
                bridge.last_solver_telemetry().has_value()
                && bridge.last_solver_telemetry()->status
                    == physics_sim::SolverStatus::HeldLastGood;
            SupportDivergenceTick tick{};
            tick.step = i;
            tick.walk = walk_tick;
            tick.held = held_tick;
            if (bridge.last_solver_telemetry().has_value()) {
                const PhysicsSimSolverTelemetry& solver = bridge.last_solver_telemetry().value();
                tick.solver_status = static_cast<int>(solver.status);
                tick.speed_limit_frame = static_cast<int>(solver.speed_limit_frame);
                tick.speed_limit_support = static_cast<int>(solver.speed_limit_support);
            }
            tick.body_x = current_position.x;
            tick.body_y = current_position.y;
            tick.body_z = current_position.z;
            tick.body_vx = state.body_twist_state.body_trans_mps.x;
            tick.body_vy = state.body_twist_state.body_trans_mps.y;
            tick.body_vz = state.body_twist_state.body_trans_mps.z;
            for (int leg = 0; leg < kNumLegs; ++leg) {
                const std::size_t idx = static_cast<std::size_t>(leg);
                SupportDivergenceLegTick& L = tick.legs[idx];
                const Vec3& commanded = locomotion_debug.commanded_foot_world_m[idx];
                const Vec3& measured = locomotion_debug.measured_foot_world_m[idx];
                L.commanded_world_x = commanded.x;
                L.commanded_world_y = commanded.y;
                L.commanded_world_z = commanded.z;
                L.measured_world_x = measured.x;
                L.measured_world_y = measured.y;
                L.measured_world_z = measured.z;
                L.terrain_z = 0.0;
                if (i > 0 && command_step_s > 0.0) {
                    L.measured_dz_dt =
                        (measured.z - previous_locomotion_debug.measured_foot_world_m[idx].z)
                        / command_step_s;
                }
                L.in_stance = gait.in_stance[idx];
                L.phase = gait.phase[idx];
                L.duty_factor = gait.duty_factor;
                L.raw_contact = locomotion_debug.raw_contact[idx];
                L.fused_load_bearing = locomotion_debug.fused_load_bearing[idx];
                L.fused_contact_phase = locomotion_debug.fused_contact_phase[idx];
                L.contact_anchor_valid = locomotion_debug.contact_anchor_valid[idx];
                L.contact_anchor_drift_m = locomotion_debug.contact_anchor_drift_m[idx];
                for (int joint = 0; joint < 3; ++joint) {
                    const std::size_t jidx = static_cast<std::size_t>(joint);
                    L.target_rad[jidx] =
                        applied_targets.leg_states[idx].joint_state[jidx].pos_rad.value;
                    L.measured_rad[jidx] =
                        state.leg_states[idx].joint_state[jidx].pos_rad.value;
                    L.error_rad[jidx] = std::remainder(
                        L.target_rad[jidx] - L.measured_rad[jidx], 6.28318530717958647692);
                }
                if (bridge.last_solver_telemetry().has_value()) {
                    L.leg_contact_count =
                        static_cast<int>(bridge.last_solver_telemetry()->leg_contact_count[idx]);
                }
                const Vec3& planned_body = locomotion_debug.planned_leg_target_body_m[idx];
                const Vec3& pre_slew_body = locomotion_debug.pre_slew_fk_body_m[idx];
                const Vec3& post_clamp_body = locomotion_debug.post_clamp_fk_body_m[idx];
                L.planned_body = {planned_body.x, planned_body.y, planned_body.z};
                L.pre_slew_body = {pre_slew_body.x, pre_slew_body.y, pre_slew_body.z};
                L.post_clamp_body = {post_clamp_body.x, post_clamp_body.y, post_clamp_body.z};
            }
            if (walk_tick) {
                support_divergence_ring.push_back(tick);
                if (support_divergence_ring.size() > kSupportDivergenceRing) {
                    support_divergence_ring.pop_front();
                }
            }
            if (held_tick || !walk_tick) {
                support_divergence_frozen = true;
            }
        }
        previous_applied_targets = applied_targets;
        previous_locomotion_debug = locomotion_debug;
        previous_gait = gait;
    }

    result.support_divergence_ticks.assign(
        // Preserve the old ring independently of the complete event census.
        support_divergence_ring.begin(), support_divergence_ring.end());
    result.swing_events.finish(walk_steps * command_step_s);

    if (const auto& failed = bridge.first_failed_solver_telemetry(); failed.has_value()) {
        result.first_read_fail = true;
        result.first_failed_solver_status = static_cast<int>(failed->status);
        result.first_failed_failure_reason = static_cast<int>(failed->failure_reason);
        result.first_failed_speed_limit_frame = static_cast<int>(failed->speed_limit_frame);
        result.first_failed_speed_limit_support = static_cast<int>(failed->speed_limit_support);
        result.first_failed_chassis_w = failed->chassis_preintegration_angular_speed;
        result.first_failed_max_link_w = failed->max_link_preintegration_angular_speed;
        result.first_failed_retry_count = failed->retry_count;
        result.first_failed_held_state_count = failed->held_state_count;
        result.first_failed_iterations = static_cast<int>(failed->iterations);
        result.first_failed_ncp_dual = failed->ncp_dual_residual;
        result.first_failed_ncp_comp = failed->ncp_complementarity_residual;
        result.first_failed_admm_rho = failed->admm_rho;
        result.first_failed_contact_constraint_count = failed->contact_constraint_count;
        result.first_failed_worst_contact_id = failed->worst_contact_id;
    }

    result.end_position = positionFromState(bridge.last_state().value());
    result.end_yaw_rad = bridge.last_state().value().body_twist_state.twist_pos_rad.z;
    result.average_horizontal_speed_mps = horizontal_speed_sum / static_cast<double>(walk_steps);
    result.mean_body_forward_speed_mps = body_forward_speed_sum / static_cast<double>(walk_steps);
    result.mean_governed_planar_speed_mps =
        governed_planar_speed_sum / static_cast<double>(walk_steps);
    result.mean_governed_yaw_rate_radps =
        governed_yaw_rate_sum / static_cast<double>(walk_steps);
    result.mean_governor_command_scale =
        governor_command_scale_sum / static_cast<double>(walk_steps);
    result.mean_governor_cadence_scale =
        governor_cadence_scale_sum / static_cast<double>(walk_steps);
    result.mean_gait_frequency_hz = gait_frequency_sum / static_cast<double>(walk_steps);
    result.mean_gait_duty_factor = gait_duty_factor_sum / static_cast<double>(walk_steps);
    result.mean_gait_step_length_m = gait_step_length_sum / static_cast<double>(walk_steps);
    result.mean_body_tilt_rad = body_tilt_sum / static_cast<double>(walk_steps);
    result.mean_body_roll_rad = body_roll_sum / static_cast<double>(walk_steps);
    result.mean_body_pitch_rad = body_pitch_sum / static_cast<double>(walk_steps);
    result.average_yaw_rate_radps = yaw_speed_sum / static_cast<double>(walk_steps);
    for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
        if (result.loaded_stance_samples[leg] == 0) {
            continue;
        }
        const double denom = static_cast<double>(result.loaded_stance_samples[leg]);
        result.mean_loaded_stance_commanded_sweep_mps[leg] =
            loaded_stance_commanded_sweep_sum[leg] / denom;
        result.mean_loaded_stance_pre_slew_sweep_mps[leg] =
            loaded_stance_pre_slew_sweep_sum[leg] / denom;
        result.mean_loaded_stance_measured_sweep_mps[leg] =
            loaded_stance_measured_sweep_sum[leg] / denom;
        result.mean_loaded_stance_world_slip_mps[leg] =
            loaded_stance_world_slip_sum[leg] / denom;
        result.mean_loaded_stance_tracking_error_m[leg] =
            loaded_stance_tracking_error_sum[leg] / denom;
        result.mean_loaded_stance_post_clamp_distortion_m[leg] =
            loaded_stance_post_clamp_distortion_sum[leg] / denom;
        result.mean_loaded_stance_latched_length_m[leg] =
            loaded_stance_latched_length_sum[leg] / denom;
        result.mean_loaded_stance_used_length_m[leg] =
            loaded_stance_used_length_sum[leg] / denom;
    }
    if (result.planned_stance_joint_error_samples > 0) {
        const double denom = static_cast<double>(result.planned_stance_joint_error_samples);
        for (std::size_t joint = 0; joint < 3; ++joint) {
            result.mean_planned_stance_joint_error_rad[joint] =
                result.planned_stance_joint_error_sum[joint] / denom;
        }
    }
    if (!solver_iteration_samples.empty()) {
        std::sort(solver_iteration_samples.begin(), solver_iteration_samples.end());
        const std::size_t p99Index = std::min(
            solver_iteration_samples.size() - 1U,
            (99U * solver_iteration_samples.size() + 99U) / 100U - 1U);
        result.p99_solver_iterations = solver_iteration_samples[p99Index];
    }
    if (!compliant_projected_residual_samples.empty()) {
        std::sort(compliant_projected_residual_samples.begin(),
                  compliant_projected_residual_samples.end());
        const std::size_t p99Index = std::min(
            compliant_projected_residual_samples.size() - 1U,
            (99U * compliant_projected_residual_samples.size() + 99U) / 100U - 1U);
        result.p99_solver_compliant_projected_residual =
            compliant_projected_residual_samples[p99Index];
    }
    if (!contact_penetration_samples.empty()) {
        std::sort(contact_penetration_samples.begin(), contact_penetration_samples.end());
        const std::size_t p99Index = std::min(
            contact_penetration_samples.size() - 1U,
            (99U * contact_penetration_samples.size() + 99U) / 100U - 1U);
        result.p99_solver_contact_penetration = contact_penetration_samples[p99Index];
    }

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
                walkDistanceLimitsWalkEnvelopeJsonDynamic(
                    label, 0.08, 0.05, 0.04, 0.50, 0.02, 0.20, 0.90, true),
                std::string("{\"exception\":\"") + locomotion_test::jsonEscape(ex.what()) + "\"}");
        }
        return expect(false, label + ": " + ex.what());
    }

    maybeWriteSupportDivergenceDump(label, result);

    {
        std::ostringstream recovered;
        std::ostringstream held;
        std::ostringstream frames;
        appendFailureReasonHistogram(recovered, result.solver_recovered_failure_reason);
        appendFailureReasonHistogram(held, result.solver_held_failure_reason);
        appendSpeedLimitFrameHistogram(frames, result.solver_speed_limit_frame);
        std::cout << label
                  << " solver_recovered=" << result.solver_recovered_steps
                  << " recovered_reasons=" << recovered.str()
                  << " solver_held=" << result.solver_held_steps
                  << " held_reasons=" << held.str()
                  << " peak_pre_w=" << result.peak_solver_preintegration_angular_speed
                  << " chassis_w=" << result.peak_solver_chassis_preintegration_angular_speed
                  << " max_link_w=" << result.peak_solver_max_link_preintegration_angular_speed
                  << " speed_limit_frames=" << frames.str()
                  << " speed_limit_support=swing=" << result.solver_speed_limit_swing_steps
                  << ",stance=" << result.solver_speed_limit_stance_steps
                  << " max_held_streak=" << result.max_consecutive_held_steps
                  << " first_non_walk=" << result.first_non_walk_step
                  << " first_fault_step=" << result.first_fault_step
                  << " non_walk=" << result.non_walk_mode_steps
                  << " faulted=" << result.faulted_steps
                  << " mode=" << static_cast<int>(result.final_status.active_mode)
                  << " fault=" << static_cast<int>(result.final_status.active_fault)
                  << " first_fault=" << static_cast<int>(result.first_fault)
                  << " first_read_fail=" << (result.first_read_fail ? 1 : 0)
                  << " first_failed_status="
                  << (result.first_failed_solver_status < 0
                          ? "none"
                          : solverStatusName(static_cast<physics_sim::SolverStatus>(
                                result.first_failed_solver_status)))
                  << " first_failed_reason="
                  << (result.first_failed_failure_reason < 0
                          ? "none"
                          : solverFailureReasonName(
                                static_cast<physics_sim::SolverFailureReason>(
                                    result.first_failed_failure_reason)))
                  << " first_failed_frame="
                  << speedLimitFrameName(static_cast<std::uint8_t>(
                         std::max(0, result.first_failed_speed_limit_frame)))
                  << " first_failed_support="
                  << speedLimitSupportName(static_cast<std::uint8_t>(
                         std::max(0, result.first_failed_speed_limit_support)))
                  << " first_failed_chassis_w=" << result.first_failed_chassis_w
                  << " first_failed_max_link_w=" << result.first_failed_max_link_w
                  << " first_failed_retry_count=" << result.first_failed_retry_count
                  << " first_failed_held_count=" << result.first_failed_held_state_count
                  << " first_failed_iters=" << result.first_failed_iterations
                  << " first_failed_ncp_dual=" << result.first_failed_ncp_dual
                  << " first_failed_ncp_comp=" << result.first_failed_ncp_comp
                  << " first_failed_rho=" << result.first_failed_admm_rho
                  << " first_failed_contacts=" << result.first_failed_contact_constraint_count
                  << " first_failed_worst=" << result.first_failed_worst_contact_id
                  << " recovered_sl_streak="
                  << result.recovered_speed_limit_streak_before_first_hold
                  << " recovered_sl_frame="
                  << speedLimitFrameName(static_cast<std::uint8_t>(
                         std::max(0, result.recovered_speed_limit_frame_before_first_hold)))
                  << " recovered_sl_support="
                  << speedLimitSupportName(static_cast<std::uint8_t>(
                         std::max(0, result.recovered_speed_limit_support_before_first_hold)))
                  << " recovered_ncp_streak="
                  << result.recovered_ncp_streak_before_first_hold
                  << '\n';
        std::cout << label << " support_census";
        const auto percent = [](const std::uint64_t part, const std::uint64_t whole) {
            return whole == 0 ? 0.0
                              : 100.0 * static_cast<double>(part) / static_cast<double>(whole);
        };
        std::cout << " swing_drag_pct=[";
        for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
            std::cout << (leg == 0 ? "" : ",")
                      << percent(result.planned_swing_contact_samples[leg],
                                 result.planned_swing_samples[leg]);
        }
        std::cout << "] stance_lost_pct=[";
        for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
            std::cout << (leg == 0 ? "" : ",")
                      << percent(result.planned_stance_no_contact_samples[leg],
                                 result.planned_stance_samples[leg]);
        }
        std::cout << "] liftoff_delay=[";
        for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
            std::cout << (leg == 0 ? "" : ",") << result.max_liftoff_delay_samples[leg];
        }
        std::cout << "] mean_raw_support="
                  << (result.support_census_samples == 0
                          ? 0.0
                          : static_cast<double>(result.raw_contact_count_sum)
                              / static_cast<double>(result.support_census_samples))
                  << " mean_planned_support="
                  << (result.support_census_samples == 0
                          ? 0.0
                          : static_cast<double>(result.planned_stance_count_sum)
                              / static_cast<double>(result.support_census_samples))
                  << " max_loaded_swing_err=" << result.max_loaded_swing_joint_error_rad
                  << " max_swing_clearance=" << result.max_planned_swing_measured_foot_clearance_m
                  << " femur_stance_err=" << result.mean_planned_stance_joint_error_rad[1]
                  << " coxa_stance_err=" << result.mean_planned_stance_joint_error_rad[0]
                  << " height_p2p="
                  << (result.maximum_body_height_m - result.minimum_body_height_m)
                  << '\n';
        std::cout << label
                  << " loaded_stance_census body_forward="
                  << result.mean_body_forward_speed_mps
                  << " governed_speed=" << result.mean_governed_planar_speed_mps
                  << " command_scale=" << result.mean_governor_command_scale
                  << " cadence_scale=" << result.mean_governor_cadence_scale
                  << " gait_hz=" << result.mean_gait_frequency_hz
                  << " duty=" << result.mean_gait_duty_factor
                  << " step_length=" << result.mean_gait_step_length_m
                  << " freeze=" << result.governor_freeze_samples
                  << " recovery_stage=["
                  << result.governor_recovery_stage_samples[0] << ','
                  << result.governor_recovery_stage_samples[1] << ','
                  << result.governor_recovery_stage_samples[2] << ','
                  << result.governor_recovery_stage_samples[3] << ']'
                  << " tilt_mean=" << result.mean_body_tilt_rad
                  << " roll_mean=" << result.mean_body_roll_rad
                  << " pitch_mean=" << result.mean_body_pitch_rad
                  << " pitch_range=[" << result.min_body_pitch_rad << ':'
                  << result.max_body_pitch_rad << ']'
                  << " tilt_max=" << result.max_body_tilt_rad
                  << " tilt_hold_samples=" << result.body_tilt_above_stability_hold_samples
                  << " commanded_sweep=[";
        for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
            std::cout << (leg == 0 ? "" : ",")
                      << result.mean_loaded_stance_commanded_sweep_mps[leg];
        }
        std::cout << "] pre_slew_sweep=[";
        for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
            std::cout << (leg == 0 ? "" : ",")
                      << result.mean_loaded_stance_pre_slew_sweep_mps[leg];
        }
        std::cout << "] measured_sweep=[";
        for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
            std::cout << (leg == 0 ? "" : ",")
                      << result.mean_loaded_stance_measured_sweep_mps[leg];
        }
        std::cout << "] world_slip=[";
        for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
            std::cout << (leg == 0 ? "" : ",")
                      << result.mean_loaded_stance_world_slip_mps[leg];
        }
        std::cout << "] tracking_error=[";
        for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
            std::cout << (leg == 0 ? "" : ",")
                      << result.mean_loaded_stance_tracking_error_m[leg];
        }
        std::cout << "] clamp_distortion=[";
        for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
            std::cout << (leg == 0 ? "" : ",")
                      << result.mean_loaded_stance_post_clamp_distortion_m[leg];
        }
        std::cout << "] latched_length=[";
        for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
            std::cout << (leg == 0 ? "" : ",")
                      << result.mean_loaded_stance_latched_length_m[leg];
        }
        std::cout << "] used_length=[";
        for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
            std::cout << (leg == 0 ? "" : ",")
                      << result.mean_loaded_stance_used_length_m[leg];
        }
        std::cout << "] slew_limited=[";
        for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
            std::cout << (leg == 0 ? "" : ",")
                      << result.loaded_stance_slew_limited_samples[leg];
        }
        std::cout << "] workspace_limited=[";
        for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
            std::cout << (leg == 0 ? "" : ",")
                      << result.loaded_stance_workspace_limited_samples[leg];
        }
        std::cout << "] stroke_limited=[";
        for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
            std::cout << (leg == 0 ? "" : ",")
                      << result.loaded_stance_stroke_limited_samples[leg];
        }
        std::cout << "] stability_hold=[";
        for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
            std::cout << (leg == 0 ? "" : ",")
                      << result.loaded_stance_hold_samples[leg];
        }
        std::cout << "] hold_phase=[";
        for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
            const double min_phase = std::isfinite(result.loaded_stance_hold_phase_min[leg])
                ? result.loaded_stance_hold_phase_min[leg]
                : 0.0;
            std::cout << (leg == 0 ? "" : ",") << min_phase << ':'
                      << result.loaded_stance_hold_phase_max[leg];
        }
        std::cout << "] samples=[";
        for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
            std::cout << (leg == 0 ? "" : ",") << result.loaded_stance_samples[leg];
        }
        std::cout << "]\n";
    }

    const Vec3 delta = result.end_position - result.start_position;
    const double horizontal_distance = std::hypot(delta.x, delta.y);
    const double commanded_speed = walk_motion.speed_mps;
    const double average_ratio = commanded_speed > 0.0 ? (result.average_horizontal_speed_mps / commanded_speed) : 0.0;
    const DirectionalTravel direction =
        directionalTravelFromStartYaw(result, delta, walk_motion.heading_rad);

    const double kMinPathLengthM =
        test_limits::getDouble(kWalkDistanceSuite, label, "", "min_path_length_m", 0.08);
    const double kMinNetHorizontalDistanceM =
        test_limits::getDouble(kWalkDistanceSuite, label, "", "min_net_horizontal_distance_m", 0.05);
    const double kMinCommandDirectionProjectionM =
        test_limits::getDouble(kWalkDistanceSuite, label, "", "min_command_direction_projection_m", 0.04);
    const double kMinCommandDirectionCosine =
        test_limits::getDouble(kWalkDistanceSuite, label, "", "min_command_direction_cosine", 0.50);
    const double kMinPeakHorizontalSpeedMps =
        test_limits::getDouble(kWalkDistanceSuite, label, "", "min_peak_horizontal_speed_mps", 0.02);
    const double kMinAverageSpeedRatio =
        test_limits::getDouble(kWalkDistanceSuite, label, "", "min_average_speed_ratio", 0.20);
    const double kMaxAverageSpeedRatio =
        test_limits::getDouble(kWalkDistanceSuite, label, "", "max_average_speed_ratio", 0.90);
    const bool kRequireActiveModeWalk =
        test_limits::getBool(kWalkDistanceSuite, label, "", "require_active_mode_walk", true);
    const std::string limits_walk_json = walkDistanceLimitsWalkEnvelopeJsonDynamic(
        label,
        kMinPathLengthM,
        kMinNetHorizontalDistanceM,
        kMinCommandDirectionProjectionM,
        kMinCommandDirectionCosine,
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
                                      limits_walk_json,
                                      motionRunResultMetricsJson(
                                          result, delta, horizontal_distance, commanded_speed, average_ratio, direction));
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

    if (!expect(horizontal_distance >= kMinNetHorizontalDistanceM,
                label + ": walk should make measurable net horizontal progress")) {
        std::cerr << label << " path=" << result.walk_path_length_m
                  << " net_horiz=" << horizontal_distance
                  << " dx=" << delta.x
                  << " dy=" << delta.y << '\n';
        emit(false);
        return false;
    }

    if (!expect(direction.forward_projection_m >= kMinCommandDirectionProjectionM,
                label + ": walk should progress in the commanded body-frame direction")) {
        std::cerr << label << " projection=" << direction.forward_projection_m
                  << " lateral_projection=" << direction.lateral_projection_m
                  << " alignment_cosine=" << direction.alignment_cosine
                  << " start_yaw=" << result.start_yaw_rad
                  << " heading=" << walk_motion.heading_rad << '\n';
        emit(false);
        return false;
    }
    if (!expect(direction.alignment_cosine >= kMinCommandDirectionCosine,
                label + ": net travel should align with the commanded body-frame heading")) {
        std::cerr << label << " projection=" << direction.forward_projection_m
                  << " lateral_projection=" << direction.lateral_projection_m
                  << " alignment_cosine=" << direction.alignment_cosine << '\n';
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
              << " direction_projection=" << direction.forward_projection_m
              << " direction_cosine=" << direction.alignment_cosine
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
                walkDistanceLimitsStraightJsonDynamic(
                    label, 0.08, 0.20, 0.35, 0.04, 0.50, 0.02, 0.20, 0.90, true),
                std::string("{\"exception\":\"") + locomotion_test::jsonEscape(ex.what()) + "\"}");
        }
        return expect(false, label + ": " + ex.what());
    }

    const Vec3 delta = result.end_position - result.start_position;
    const double horizontal_distance = std::hypot(delta.x, delta.y);
    const double commanded_speed = walk_motion.speed_mps;
    const double average_ratio = commanded_speed > 0.0 ? (result.average_horizontal_speed_mps / commanded_speed) : 0.0;
    const DirectionalTravel direction =
        directionalTravelFromStartYaw(result, delta, walk_motion.heading_rad);

    const double kMinPathLengthM =
        test_limits::getDouble(kWalkDistanceSuite, label, "", "min_path_length_m", 0.08);
    const double kMaxLateralDeviationM =
        test_limits::getDouble(kWalkDistanceSuite, label, "", "max_lateral_deviation_m", 0.20);
    const double kMaxLateralVsPathRatio =
        test_limits::getDouble(kWalkDistanceSuite, label, "", "max_lateral_vs_path_ratio", 0.35);
    const double kMinCommandDirectionProjectionM =
        test_limits::getDouble(kWalkDistanceSuite, label, "", "min_command_direction_projection_m", 0.04);
    const double kMinCommandDirectionCosine =
        test_limits::getDouble(kWalkDistanceSuite, label, "", "min_command_direction_cosine", 0.50);
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
        kMinCommandDirectionProjectionM,
        kMinCommandDirectionCosine,
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
                                          result, delta, horizontal_distance, commanded_speed, average_ratio, direction));
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

    if (!expect(direction.forward_projection_m >= kMinCommandDirectionProjectionM,
                label + ": straight walk should progress in the commanded body-frame direction") ||
        !expect(direction.alignment_cosine >= kMinCommandDirectionCosine,
                label + ": straight-walk travel should align with the commanded body-frame heading")) {
        std::cerr << label << " projection=" << direction.forward_projection_m
                  << " lateral_projection=" << direction.lateral_projection_m
                  << " alignment_cosine=" << direction.alignment_cosine << '\n';
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
              << " direction_projection=" << direction.forward_projection_m
              << " direction_cosine=" << direction.alignment_cosine
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
        result = runMotionSequence(runtime, bridge, stand_motion, turn_motion, bus_loop_period_us, true);
    } catch (const std::exception& ex) {
        if (emit_metrics_json) {
            physics_sim_metrics::emitLine(
                "physics_sim_walk_distance",
                label,
                false,
                walkDistanceLimitsTurnJsonDynamic(label, 2.25, 0.21, 0.02, 0.05, 2.25, 0.05, true, true),
                std::string("{\"exception\":\"") + locomotion_test::jsonEscape(ex.what()) + "\"}");
        }
        return expect(false, label + ": " + ex.what());
    }

    const Vec3 delta = result.end_position - result.start_position;
    const double horizontal_distance = std::hypot(delta.x, delta.y);
    const double yaw_delta = wrapAngleDiff(result.start_yaw_rad, result.end_yaw_rad);
    const PlanarMotionCommand turn_planar = planarMotionCommand(turn_motion);
    const BodyTwist turn_raw = rawLocomotionTwistFromIntent(turn_motion, turn_planar);
    const double commanded_yaw_rate_signed = turn_planar.yaw_rate_radps;
    const double commanded_yaw_rate = std::abs(commanded_yaw_rate_signed);
    const bool commanded_yaw_direction_match = yaw_delta * commanded_yaw_rate_signed > 0.0;
    const double average_ratio =
        commanded_yaw_rate > 0.0 ? (result.average_yaw_rate_radps / commanded_yaw_rate) : 0.0;

    const double abs_yaw = std::abs(yaw_delta);
    const bool radius_valid = abs_yaw > 0.1;
    const double equivalent_radius_m =
        radius_valid ? (horizontal_distance / (2.0 * std::sin(abs_yaw * 0.5))) : 0.0;
    const double path_per_rad_m =
        abs_yaw > 1.0e-9 ? (result.walk_path_length_m / abs_yaw) : 0.0;
    std::cout << label << " turn_command_census cmd_yaw=" << turn_motion.cmd_yaw_radps.value
              << " twist_z=" << turn_motion.twist.twist_vel_radps.z
              << " planar_yaw=" << turn_planar.yaw_rate_radps
              << " raw_wz=" << turn_raw.angular_radps.z
              << " raw_vx=" << turn_raw.linear_mps.x
              << " raw_vy=" << turn_raw.linear_mps.y
              << " intent_planar=" << result.intent_planar_mps
              << " yaw_equiv=" << result.yaw_equiv_mps
              << " yaw_dominant=" << (result.yaw_dominant ? "true" : "false")
              << " start_xy=" << result.start_position.x << "," << result.start_position.y
              << " net_horiz=" << horizontal_distance
              << " path=" << result.walk_path_length_m
              << " yaw_delta=" << yaw_delta
              << " r=" << equivalent_radius_m
              << " path_per_rad=" << path_per_rad_m
              << " held=" << result.solver_held_steps
              << " first_walk_phase0=" << result.first_walk_gait_phase_leg0
              << " first_walk_req_planar=" << result.first_walk_requested_planar_speed_mps
              << " first_walk_gov_planar=" << result.first_walk_governed_planar_speed_mps
              << " first_walk_req_yaw=" << result.first_walk_requested_yaw_rate_radps
              << " stand_end_vxy=" << result.stand_end_body_vx_mps << ","
              << result.stand_end_body_vy_mps
              << " mean_governed_yaw=" << result.mean_governed_yaw_rate_radps
              << " mean_gait_step=" << result.mean_gait_step_length_m << '\n';
    maybeWriteTurnTrajDump(result,
                           horizontal_distance,
                           yaw_delta,
                           equivalent_radius_m,
                           path_per_rad_m);
    maybeWriteTurnEntryDump(result, horizontal_distance);
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
    const bool kRequireCommandedYawDirection =
        test_limits::getBool(kWalkDistanceSuite, label, "", "require_commanded_yaw_direction", true);
    const bool kRequireActiveModeWalkTurn =
        test_limits::getBool(kWalkDistanceSuite, label, "", "require_active_mode_walk", true);
    const std::string limits_turn_json = walkDistanceLimitsTurnJsonDynamic(label,
                                                                           kMaxPathLengthM,
                                                                           kMaxNetHorizontalDistanceM,
                                                                           kMinPeakYawRateRadps,
                                                                           kMinAverageYawRateRatio,
                                                                           kMaxAverageYawRateRatio,
                                                                           kMinYawDeltaRad,
                                                                           kRequireCommandedYawDirection,
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
                                          result,
                                          delta,
                                          horizontal_distance,
                                          yaw_delta,
                                          commanded_yaw_rate,
                                          average_ratio,
                                          commanded_yaw_direction_match));
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

    if (kRequireCommandedYawDirection &&
        !expect(commanded_yaw_direction_match,
                label + ": turn-in-place should rotate in the commanded yaw direction")) {
        std::cerr << label << " yaw_delta=" << yaw_delta
                  << " signed_command_yaw_rate=" << commanded_yaw_rate_signed << '\n';
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
    PhysicsSimSolverSettings solver_settings =
        physics_sim_test_utils::productionProximalSolverSettings();
    if (const char* mode = std::getenv("HEXAPOD_WALK_TEST_SOLVER_MODE")) {
        if (std::string(mode) == "legacy-pgs") {
            solver_settings.mode = physics_sim::PhysicsSolverMode::LegacyPgs;
            solver_settings.iterations = harness.physics_solver_iterations;
        } else if (std::string(mode) == "pinocchio-compliant") {
            solver_settings.mode = physics_sim::PhysicsSolverMode::PinocchioProximalCompliant;
        } else if (std::string(mode) != "pinocchio-proximal") {
            std::cerr << "invalid HEXAPOD_WALK_TEST_SOLVER_MODE=" << mode << '\n';
            return 2;
        }
    }
    if (const char* value = std::getenv("HEXAPOD_WALK_TEST_SOLVER_ITERATIONS")) {
        char* end = nullptr;
        const long parsed = std::strtol(value, &end, 10);
        if (end == value || *end != '\0' || parsed <= 0) {
            std::cerr << "invalid HEXAPOD_WALK_TEST_SOLVER_ITERATIONS=" << value << '\n';
            return 2;
        }
        solver_settings.iterations = static_cast<int>(parsed);
    }
    if (const char* value = std::getenv("HEXAPOD_WALK_TEST_PROXIMAL_MU")) {
        char* end = nullptr;
        const double parsed = std::strtod(value, &end);
        if (end == value || *end != '\0' || !std::isfinite(parsed) || parsed <= 0.0) {
            std::cerr << "invalid HEXAPOD_WALK_TEST_PROXIMAL_MU=" << value << '\n';
            return 2;
        }
        solver_settings.proximal_mu = static_cast<float>(parsed);
    }
    if (const char* value = std::getenv("HEXAPOD_WALK_TEST_ABSOLUTE_TOLERANCE")) {
        char* end = nullptr;
        const double parsed = std::strtod(value, &end);
        if (end == value || *end != '\0' || !std::isfinite(parsed) || parsed <= 0.0) {
            std::cerr << "invalid HEXAPOD_WALK_TEST_ABSOLUTE_TOLERANCE=" << value << '\n';
            return 2;
        }
        solver_settings.absolute_tolerance = static_cast<float>(parsed);
    }
    if (const char* value = std::getenv("HEXAPOD_WALK_TEST_RELATIVE_TOLERANCE")) {
        char* end = nullptr;
        const double parsed = std::strtod(value, &end);
        if (end == value || *end != '\0' || !std::isfinite(parsed) || parsed <= 0.0) {
            std::cerr << "invalid HEXAPOD_WALK_TEST_RELATIVE_TOLERANCE=" << value << '\n';
            return 2;
        }
        solver_settings.relative_tolerance = static_cast<float>(parsed);
    }
    double body_height_m = 0.14;
    if (const char* value = std::getenv("HEXAPOD_WALK_TEST_BODY_HEIGHT_M")) {
        char* end = nullptr;
        const double parsed = std::strtod(value, &end);
        if (end == value || *end != '\0' || !std::isfinite(parsed) || parsed <= 0.0) {
            std::cerr << "invalid HEXAPOD_WALK_TEST_BODY_HEIGHT_M=" << value << '\n';
            return 2;
        }
        body_height_m = parsed;
    }

    pid_t pid = ::fork();
    if (pid < 0) {
        std::cerr << "fork failed\n";
        return 2;
    }
    if (pid == 0) {
        if (const char* value = std::getenv("HEXAPOD_WALK_TEST_CHILD_STDIO");
            value == nullptr || value[0] == '\0' || value[0] == '0') {
            physics_sim_test_utils::quietChildProcessStdIo();
        } else {
            // Keep simulator diagnostics out of the parent's JSON metrics stream.
            ::dup2(STDERR_FILENO, STDOUT_FILENO);
        }
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
        "127.0.0.1", kPort, kBusLoopPeriodUs, solver_settings);
    CapturingPhysicsSimBridge* bridge_ptr = bridge.get();

    control_config::ControlConfig cfg = harness.control_cfg;
    cfg.freshness.estimator.max_allowed_age_us = DurationUs{10'000'000};
    cfg.freshness.intent.max_allowed_age_us = DurationUs{10'000'000};
    // Opt-in screen, default off. Same Bounded FF the locomotion-regression
    // long-walk case already uses (leftover §3.16): take gravity off the PD
    // sag budget. Does not change stall, height, or gait.
    const char* gravity_ff = std::getenv("HEXAPOD_WALK_TEST_GRAVITY_FF");
    if (gravity_ff != nullptr && gravity_ff[0] != '\0' && std::string{gravity_ff} != "0") {
        cfg.gravity_feedforward.enabled = true;
        cfg.gravity_feedforward.mode = control_config::GravityFeedforwardMode::Bounded;
        cfg.gravity_feedforward.scale_coxa = 0.0;
        cfg.gravity_feedforward.scale_femur = 0.30;
        cfg.gravity_feedforward.scale_tibia = 0.30;
        cfg.gravity_feedforward.stiffness_gain_scale = 0.62;
        cfg.gravity_feedforward.delta_lpf_tau_s = 0.08;
        cfg.gravity_feedforward.include_foot_reaction = true;
        cfg.gravity_feedforward.include_self_weight = false;
        std::cerr << "[walk-distance-gravity-ff] enabled scale_femur=0.30 scale_tibia=0.30 "
                     "stiffness=0.62 lpf=0.08\n";
    }

    physics_sim_test_utils::applySelfWeightOnlyScreen(cfg);
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
        body_height_m,
        0.0,
        0.0,
        0.0};
    const ScenarioMotionIntent walk_forward_motion{
        true,
        RobotMode::WALK,
        GaitType::TRIPOD,
        body_height_m,
        0.20,
        0.0,
        0.0};
    const ScenarioMotionIntent walk_slow_forward_motion{
        true,
        RobotMode::WALK,
        GaitType::TRIPOD,
        body_height_m,
        0.06,
        0.0,
        0.0};
    const ScenarioMotionIntent walk_reverse_motion{
        true,
        RobotMode::WALK,
        GaitType::TRIPOD,
        body_height_m,
        0.20,
        kPi,
        0.0};
    MotionIntent turn_in_place_motion =
        makeMotionIntent(RobotMode::WALK, GaitType::TRIPOD, body_height_m);
    // Explicit cmd_yaw, not twist.z. Filling only twist.z makes planarMotionCommand
    // copy it and rawLocomotionTwistFromIntent add it again (0.90 instead of 0.45),
    // and walk-entry seeding misses yaw-dominant Φ=0. Same construction as
    // scenario/replay turn_in_place.
    turn_in_place_motion.cmd_yaw_radps = AngularRateRadPerSec{0.45};

    const char* case_filter = std::getenv("HEXAPOD_WALK_TEST_CASE");
    const std::string case_filter_str =
        (case_filter == nullptr || case_filter[0] == '\0') ? std::string() : std::string(case_filter);
    const auto should_run_case = [&case_filter_str](const char* name) {
        if (case_filter_str.empty() || case_filter_str == name) {
            return true;
        }
        if (case_filter_str == "turn_after_reverse") {
            return std::string(name) == "reverse_walk" || std::string(name) == "turn_in_place";
        }
        if (case_filter_str == "turn_after_reverse_straight") {
            return std::string(name) == "reverse_walk" || std::string(name) == "straight_walk"
                || std::string(name) == "turn_in_place";
        }
        return false;
    };

    if (should_run_case("forward_walk") &&
        !checkWalkCase(
            "forward_walk", runtime, *bridge_ptr, stand_motion, walk_forward_motion, kBusLoopPeriodUs, emit_metrics_json)) {
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return EXIT_FAILURE;
    }

    if (should_run_case("slow_forward_walk") &&
        !checkWalkCase("slow_forward_walk",
                       runtime,
                       *bridge_ptr,
                       stand_motion,
                       walk_slow_forward_motion,
                       kBusLoopPeriodUs,
                       emit_metrics_json)) {
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return EXIT_FAILURE;
    }

    if (should_run_case("reverse_walk") &&
        !checkWalkCase(
            "reverse_walk", runtime, *bridge_ptr, stand_motion, walk_reverse_motion, kBusLoopPeriodUs, emit_metrics_json)) {
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return EXIT_FAILURE;
    }

    if (should_run_case("straight_walk") &&
        !checkStraightWalkCase("straight_walk",
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

    if (should_run_case("turn_in_place") &&
        !checkTurnCase(
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
