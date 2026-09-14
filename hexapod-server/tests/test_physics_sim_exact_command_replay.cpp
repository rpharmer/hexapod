#include "control_config.hpp"
#include "geometry_config.hpp"
#include "leg_fk.hpp"
#include "motion_intent_utils.hpp"
#include "physics_sim_bridge.hpp"
#include "physics_sim_estimator.hpp"
#include "physics_sim_test_argv.hpp"
#include "physics_sim_test_utils.hpp"
#include "robot_runtime.hpp"
#include "scenario_driver.hpp"
#include "stance_progress_metrics.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#if defined(__linux__)
#include <csignal>
#include <sys/wait.h>
#include <unistd.h>
#endif

namespace {

enum class ReplayPhase : std::uint8_t {
    Stand,
    Transition,
    Forward,
    Reverse,
    Strafe,
    Diagonal,
    TurnInPlace,
    Count,
};

constexpr std::size_t kPhaseCount = static_cast<std::size_t>(ReplayPhase::Count);

using IterationHistogram = std::map<std::uint16_t, std::uint64_t>;

constexpr std::array<const char*, kPhaseCount> kPhaseNames{
    "stand", "transition", "forward", "reverse", "strafe", "diagonal", "turn_in_place"};
constexpr std::array<const char*, kNumLegs> kLegNames{
    "R3", "L3", "R2", "L2", "R1", "L1"};

// The behaviour requirement explicitly applies after the acceleration transient. Preserve
// the complete phase metrics for transition diagnosis, and derive the excluded frame count
// from dt so the 120/240/480 Hz equivalence runs measure the same physical interval.
constexpr double kAccelerationTransientDurationS = 0.120;
constexpr double kMinimumCommandProgressRatio = 0.70;
constexpr double kMaximumLateralPathFraction = 0.10;
constexpr double kLateralAllowanceM = 0.010;
constexpr double kMaximumTurnTranslationM = 0.050;

// PERIOD_US on a frozen fixture changes physics dt, not the captured gait.
// Scoring commanded metres at a slower replay dt inflates the 70% bar while
// the servo targets are still the capture stream. Cap at capture_period_us
// so 120 Hz does not fail for extra wall-clock; 240/480 keep replay dt.
int commandScorePeriodUs(const int replay_period_us, const int capture_period_us) {
    if (replay_period_us <= 0) {
        return capture_period_us;
    }
    if (capture_period_us <= 0) {
        return replay_period_us;
    }
    return std::min(replay_period_us, capture_period_us);
}

constexpr std::array<const char*, 15> kFailureReasonNames{
    "none",
    "invalid_dt",
    "read_state",
    "non_finite_state",
    "non_finite_mass",
    "non_finite_acceleration",
    "unsupported_island",
    "solver_not_converged",
    "non_finite_impulse",
    "non_finite_velocity",
    "speed_limit",
    "non_finite_configuration",
    "write_state",
    "non_finite_energy",
    "extreme_penetration",
};

struct CapturedFrame {
    JointTargets targets{};
    ReplayPhase phase{ReplayPhase::Stand};
    bool inhibit_motion{false};
    bool walk_mode{false};
    std::array<bool, kNumLegs> planned_stance{};
    std::array<bool, kNumLegs> hold_stance{};
    std::array<bool, kNumLegs> safe_to_lift{};
    std::array<double, kNumLegs> liftoff_clearance_m{};
    std::array<double, kNumLegs> gait_phase{};
    double duty_factor{0.0};
    double stride_phase_rate_hz{0.0};
    double command_scale{1.0};
    double cadence_scale{1.0};
    std::array<bool, kNumLegs> stroke_clamp_hit{};
    std::array<bool, kNumLegs> workspace_xy_hit{};
    std::array<Vec3, kNumLegs> planned_target_body{};
    std::array<Vec3, kNumLegs> pre_slew_fk_body{};
    std::array<Vec3, kNumLegs> post_clamp_fk_body{};
    std::array<bool, kNumLegs> ik_reach_clamp_hit{};
    std::array<bool, kNumLegs> slew_clamp_hit{};
    std::array<double, kNumLegs> post_clamp_distortion_m{};
    double static_stability_margin_m{0.0};
};

struct CommandFixture {
    int capture_period_us{0};
    int stand_frames{0};
    int motion_frames{0};
    int transition_frames{0};
    double commanded_body_height_m{0.0};
    std::array<double, 3> initial_body_position{};
    std::array<double, 4> initial_body_orientation{1.0, 0.0, 0.0, 0.0};
    int selected_phase{-1};
    physics_sim::PhysicsSolverMode capture_solver_mode{
        physics_sim::PhysicsSolverMode::PinocchioProximal};
    int capture_solver_iterations{0};
    std::vector<CapturedFrame> frames{};
};

struct PhaseResult {
    std::uint64_t frames{0};
    std::uint64_t healthy{0};
    std::uint64_t recovered{0};
    std::uint64_t held{0};
    std::uint64_t unsupported{0};
    std::uint64_t read_failures{0};
    std::uint64_t solver_not_converged{0};
    std::uint64_t topology_changes{0};
    std::uint64_t high_iteration_topology_changes{0};
    std::uint64_t high_iteration_persistent_contacts{0};
    IterationHistogram iteration_histogram{};
    std::uint16_t max_iterations{0};
    double max_ncp_dual_residual{0.0};
    double max_ncp_complementarity_residual{0.0};
    double max_contact_penetration{0.0};
    double max_body_height_error{0.0};
    double max_servo_tracking_error{0.0};
    double terminal_servo_tracking_error{0.0};
    double servo_tracking_error_sq_sum{0.0};
    std::uint64_t servo_tracking_error_samples{0};
    double max_servo_torque_utilization{0.0};
    double servo_torque_utilization_sum{0.0};
    std::uint64_t servo_torque_utilization_samples{0};
    std::uint64_t servo_saturated_frames{0};
    double peak_actuator_impulse{0.0};
    double peak_normal_impulse{0.0};
    double peak_friction_impulse{0.0};
    double peak_preintegration_linear_speed{0.0};
    double peak_preintegration_angular_speed{0.0};
    double actuator_work_sum{0.0};
    double mechanical_energy_delta_sum{0.0};
    double max_foot_tracking_error{0.0};
    double foot_tracking_error_sq_sum{0.0};
    std::uint64_t foot_tracking_error_samples{0};
    double max_contact_foot_world_step{0.0};
    double contact_foot_world_step_sq_sum{0.0};
    std::uint64_t contact_foot_world_step_samples{0};
    double contact_target_opposition_sum{0.0};
    double contact_target_counter_yaw_sum{0.0};
    std::uint64_t contact_target_motion_samples{0};
    double midstance_contact_world_step_sum{0.0};
    std::uint64_t midstance_contact_world_step_samples{0};
    double midstance_tripod_contact_world_step_sum{0.0};
    std::uint64_t midstance_tripod_contact_world_step_samples{0};
    double midstance_overlap_contact_world_step_sum{0.0};
    std::uint64_t midstance_overlap_contact_world_step_samples{0};
    double n_raw_contact_sum{0.0};
    double n_planned_sum{0.0};
    double n_hold_sum{0.0};
    double n_late_swing_extra_sum{0.0};
    double n_l_parked_contacted_sum{0.0};
    std::uint64_t census_frames{0};
    std::uint64_t mixed_parked_stroking_frames{0};
    std::uint64_t n_contact_ge_5_frames{0};
    std::uint64_t clean_tripod_frames{0};
    std::array<std::uint64_t, 7> n_raw_contact_histogram{};
    double abs_pitch_sum{0.0};
    double abs_roll_sum{0.0};
    std::uint64_t attitude_samples{0};
    double peak_normal_impulse_sum{0.0};
    double peak_friction_impulse_sum{0.0};
    double friction_to_normal_ratio_sum{0.0};
    std::uint64_t friction_impulse_samples{0};
    std::uint64_t friction_ratio_samples{0};
    double contact_commanded_world_step_sum{0.0};
    double contact_uncommanded_slip_step_sum{0.0};
    double contact_cmd_body_step_sum{0.0};
    std::uint64_t contact_slip_samples{0};
    double midstance_commanded_world_step_sum{0.0};
    double midstance_uncommanded_slip_step_sum{0.0};
    double midstance_cmd_body_step_sum{0.0};
    std::uint64_t midstance_slip_samples{0};
    double clean_tripod_body_step_sum{0.0};
    std::uint64_t clean_tripod_body_samples{0};
    double clean_tripod_cartesian_opposition_sum{0.0};
    double clean_tripod_cartesian_counter_yaw_sum{0.0};
    std::uint64_t clean_tripod_cartesian_samples{0};
    double clean_tripod_commanded_world_step_sum{0.0};
    double clean_tripod_uncommanded_slip_step_sum{0.0};
    double clean_tripod_contact_world_step_sum{0.0};
    std::uint64_t clean_tripod_slip_samples{0};
    double completed_delta_x_sum{0.0};
    double completed_delta_y_sum{0.0};
    double completed_body_forward_sum{0.0};
    double completed_body_lateral_sum{0.0};
    double completed_yaw_delta_sum{0.0};
    double completed_horizontal_path_sum{0.0};
    double completed_horizontal_displacement_sum{0.0};
    double completed_start_body_forward_velocity_sum{0.0};
    double completed_start_body_lateral_velocity_sum{0.0};
    double completed_end_body_forward_velocity_sum{0.0};
    double completed_end_body_lateral_velocity_sum{0.0};
    double evaluated_body_forward_sum{0.0};
    double evaluated_body_lateral_sum{0.0};
    double evaluated_yaw_delta_sum{0.0};
    double evaluated_horizontal_path_sum{0.0};
    double evaluated_horizontal_displacement_sum{0.0};
    std::uint64_t evaluated_frames{0};
    std::uint64_t evaluated_trajectories{0};
    std::uint64_t completed_trajectories{0};
};

struct PhaseCommand {
    double vx_mps{0.0};
    double vy_mps{0.0};
    double yaw_rate_radps{0.0};
};

PhaseCommand phaseCommand(const ReplayPhase phase) {
    switch (phase) {
    case ReplayPhase::Forward:
        return {0.12, 0.0, 0.0};
    case ReplayPhase::Reverse:
        return {-0.12, 0.0, 0.0};
    case ReplayPhase::Strafe:
        return {0.0, 0.10, 0.0};
    case ReplayPhase::Diagonal:
        return {0.085, 0.085, 0.0};
    case ReplayPhase::TurnInPlace:
        return {0.0, 0.0, 0.45};
    case ReplayPhase::Stand:
    case ReplayPhase::Transition:
    case ReplayPhase::Count:
        return {};
    }
    return {};
}

struct ReplayResult {
    std::array<PhaseResult, kPhaseCount> phases{};
    std::uint64_t frames{0};
    std::uint64_t telemetry_frames{0};
    std::uint64_t healthy{0};
    std::uint64_t recovered{0};
    std::uint64_t held{0};
    std::uint64_t unsupported{0};
    std::uint64_t read_failures{0};
    std::uint64_t solver_not_converged{0};
    std::uint64_t topology_changes{0};
    std::uint64_t high_iteration_topology_changes{0};
    std::uint64_t high_iteration_persistent_contacts{0};
    IterationHistogram iteration_histogram{};
    std::map<std::uint32_t, IterationHistogram> contact_count_iteration_histograms{};
    std::map<std::uint32_t, IterationHistogram> topology_age_iteration_histograms{};
    std::array<std::uint64_t, kFailureReasonNames.size()> failure_reason_histogram{};
    std::uint16_t max_iterations{0};
    std::uint32_t max_contact_constraints{0};
    std::uint64_t max_warm_start_resets{0};
    double p99_step_time_ms{0.0};
    double max_step_time_ms{0.0};
    double healthy_p99_step_time_ms{0.0};
    double healthy_max_step_time_ms{0.0};
    double p99_solver_dynamics_time_ms{0.0};
    double p99_solver_contact_setup_time_ms{0.0};
    double p99_solver_collision_time_ms{0.0};
    double p99_solver_constraint_assembly_time_ms{0.0};
    double p99_solver_delassus_time_ms{0.0};
    double p99_solver_admm_time_ms{0.0};
    double p99_solver_integration_time_ms{0.0};
    double p99_solver_total_step_time_ms{0.0};
};

bool passesBehaviorGates(const ReplayResult& result,
                         const int replay_period_us,
                         const int capture_period_us) {
    constexpr std::array<ReplayPhase, 4> kTranslationPhases{
        ReplayPhase::Forward,
        ReplayPhase::Reverse,
        ReplayPhase::Strafe,
        ReplayPhase::Diagonal,
    };
    bool evaluated_motion = false;
    for (const ReplayPhase replay_phase : kTranslationPhases) {
        const PhaseResult& phase = result.phases[static_cast<std::size_t>(replay_phase)];
        if (phase.completed_trajectories == 0) {
            continue;
        }
        evaluated_motion = true;
        if (phase.evaluated_trajectories == 0 || phase.evaluated_frames == 0) {
            return false;
        }
        const double trajectory_count = static_cast<double>(phase.evaluated_trajectories);
        const PhaseCommand command = phaseCommand(replay_phase);
        const double command_speed = std::hypot(command.vx_mps, command.vy_mps);
        const double frames_per_trajectory =
            static_cast<double>(phase.evaluated_frames) / trajectory_count;
        const double commanded_translation = command_speed * frames_per_trajectory
            * static_cast<double>(
                  commandScorePeriodUs(replay_period_us, capture_period_us))
            * 1.0e-6;
        const double body_forward = phase.evaluated_body_forward_sum / trajectory_count;
        const double body_lateral = phase.evaluated_body_lateral_sum / trajectory_count;
        const double ux = command.vx_mps / command_speed;
        const double uy = command.vy_mps / command_speed;
        const double command_progress = ux * body_forward + uy * body_lateral;
        const double command_lateral = -uy * body_forward + ux * body_lateral;
        const double horizontal_path =
            phase.evaluated_horizontal_path_sum / trajectory_count;
        if (command_progress < kMinimumCommandProgressRatio * commanded_translation
            || std::abs(command_lateral)
                > kMaximumLateralPathFraction * horizontal_path + kLateralAllowanceM) {
            return false;
        }
    }

    const PhaseResult& turn =
        result.phases[static_cast<std::size_t>(ReplayPhase::TurnInPlace)];
    if (turn.completed_trajectories == 0) {
        return evaluated_motion;
    }
    evaluated_motion = true;
    if (turn.evaluated_trajectories == 0 || turn.evaluated_frames == 0) {
        return false;
    }
    const double trajectory_count = static_cast<double>(turn.evaluated_trajectories);
    const double frames_per_trajectory =
        static_cast<double>(turn.evaluated_frames) / trajectory_count;
    const double commanded_yaw = phaseCommand(ReplayPhase::TurnInPlace).yaw_rate_radps
        * frames_per_trajectory
        * static_cast<double>(commandScorePeriodUs(replay_period_us, capture_period_us))
        * 1.0e-6;
    const double yaw_delta = turn.evaluated_yaw_delta_sum / trajectory_count;
    const double horizontal_displacement =
        turn.evaluated_horizontal_displacement_sum / trajectory_count;
    return evaluated_motion
        && yaw_delta >= kMinimumCommandProgressRatio * commanded_yaw
        && horizontal_displacement <= kMaximumTurnTranslationM;
}

class CommandCapturingBridge final : public IHardwareBridge {
public:
    CommandCapturingBridge(std::string host,
                           int port,
                           int bus_loop_period_us,
                           PhysicsSimSolverSettings settings)
        : inner_(std::move(host), port, bus_loop_period_us, settings, nullptr) {}

    bool init() override { return inner_.init(); }

    bool read(RobotState& out) override {
        // Capture the target that this read is about to advance.  The initial
        // all-zero value is intentional: PhysicsSimBridge maps it to the same
        // standing seed used during normal runtime start-up.
        frames_.push_back(CapturedFrame{pending_targets_, phase_});
        return inner_.read(out);
    }

    bool write(const JointTargets& in) override {
        pending_targets_ = in;
        return inner_.write(in);
    }

    std::optional<BridgeCommandResultMetadata> last_bridge_result() const override {
        return inner_.last_bridge_result();
    }

    bool supportsAutomaticBusTimeoutRecovery() const override {
        return inner_.supportsAutomaticBusTimeoutRecovery();
    }

    bool latestSampleHealthyForAutomaticRecovery() const override {
        return inner_.latestSampleHealthyForAutomaticRecovery();
    }

    void setPhase(const ReplayPhase phase) { phase_ = phase; }

    void annotateLastFrame(const SafetyState& safety,
                           const ControlStatus& status,
                           const GaitState& gait,
                           const CommandGovernorState& governor,
                           const telemetry::LocomotionDebugSnapshot& locomotion,
                           const std::array<bool, kNumLegs>& stroke_clamp_hit,
                           const std::array<bool, kNumLegs>& workspace_xy_hit,
                           const std::array<bool, kNumLegs>& ik_reach_clamp_hit,
                           const std::array<bool, kNumLegs>& slew_clamp_hit) {
        if (!frames_.empty()) {
            frames_.back().inhibit_motion = safety.inhibit_motion;
            frames_.back().walk_mode = status.active_mode == RobotMode::WALK;
            frames_.back().planned_stance = gait.in_stance;
            frames_.back().hold_stance = gait.stability_hold_stance;
            frames_.back().safe_to_lift = gait.support_liftoff_safe_to_lift;
            frames_.back().liftoff_clearance_m = gait.support_liftoff_clearance_m;
            frames_.back().gait_phase = gait.phase;
            frames_.back().duty_factor = gait.duty_factor;
            frames_.back().stride_phase_rate_hz = gait.stride_phase_rate_hz.value;
            frames_.back().command_scale = governor.command_scale;
            frames_.back().cadence_scale = governor.cadence_scale;
            frames_.back().stroke_clamp_hit = stroke_clamp_hit;
            frames_.back().workspace_xy_hit = workspace_xy_hit;
            frames_.back().planned_target_body = locomotion.planned_leg_target_body_m;
            frames_.back().pre_slew_fk_body = locomotion.pre_slew_fk_body_m;
            frames_.back().post_clamp_fk_body = locomotion.post_clamp_fk_body_m;
            frames_.back().ik_reach_clamp_hit = ik_reach_clamp_hit;
            frames_.back().slew_clamp_hit = slew_clamp_hit;
            frames_.back().post_clamp_distortion_m = locomotion.post_clamp_distortion_m;
            frames_.back().static_stability_margin_m = gait.static_stability_margin_m;
        }
    }

    const std::vector<CapturedFrame>& frames() const { return frames_; }

private:
    PhysicsSimBridge inner_;
    JointTargets pending_targets_{};
    ReplayPhase phase_{ReplayPhase::Stand};
    std::vector<CapturedFrame> frames_{};
};

bool targetsAreFinite(const JointTargets& targets) {
    for (const LegState& leg : targets.leg_states) {
        for (const JointState& joint : leg.joint_state) {
            if (!std::isfinite(joint.pos_rad.value) || !std::isfinite(joint.vel_radps.value)) {
                return false;
            }
        }
    }
    return true;
}

void hashBytes(std::uint64_t& hash, const void* data, const std::size_t size) {
    const auto* bytes = static_cast<const unsigned char*>(data);
    for (std::size_t i = 0; i < size; ++i) {
        hash ^= static_cast<std::uint64_t>(bytes[i]);
        hash *= 1099511628211ULL;
    }
}

std::uint64_t commandStreamHash(const CommandFixture& fixture) {
    std::uint64_t hash = 1469598103934665603ULL;
    hashBytes(hash, &fixture.capture_period_us, sizeof(fixture.capture_period_us));
    hashBytes(hash, &fixture.stand_frames, sizeof(fixture.stand_frames));
    hashBytes(hash, &fixture.motion_frames, sizeof(fixture.motion_frames));
    hashBytes(hash, &fixture.transition_frames, sizeof(fixture.transition_frames));
    hashBytes(hash,
              &fixture.commanded_body_height_m,
              sizeof(fixture.commanded_body_height_m));
    hashBytes(hash,
              fixture.initial_body_position.data(),
              sizeof(fixture.initial_body_position));
    hashBytes(hash,
              fixture.initial_body_orientation.data(),
              sizeof(fixture.initial_body_orientation));
    hashBytes(hash, &fixture.selected_phase, sizeof(fixture.selected_phase));
    const auto capture_mode = static_cast<std::uint8_t>(fixture.capture_solver_mode);
    hashBytes(hash, &capture_mode, sizeof(capture_mode));
    hashBytes(hash,
              &fixture.capture_solver_iterations,
              sizeof(fixture.capture_solver_iterations));
    for (const CapturedFrame& frame : fixture.frames) {
        const auto phase = static_cast<std::uint8_t>(frame.phase);
        hashBytes(hash, &phase, sizeof(phase));
        const std::uint8_t inhibit_motion = frame.inhibit_motion ? 1U : 0U;
        const std::uint8_t walk_mode = frame.walk_mode ? 1U : 0U;
        hashBytes(hash, &inhibit_motion, sizeof(inhibit_motion));
        hashBytes(hash, &walk_mode, sizeof(walk_mode));
        for (const bool planned_stance : frame.planned_stance) {
            const std::uint8_t value = planned_stance ? 1U : 0U;
            hashBytes(hash, &value, sizeof(value));
        }
        for (const bool hold_stance : frame.hold_stance) {
            const std::uint8_t value = hold_stance ? 1U : 0U;
            hashBytes(hash, &value, sizeof(value));
        }
        for (const bool safe_to_lift : frame.safe_to_lift) {
            const std::uint8_t value = safe_to_lift ? 1U : 0U;
            hashBytes(hash, &value, sizeof(value));
        }
        hashBytes(
            hash, frame.liftoff_clearance_m.data(), sizeof(frame.liftoff_clearance_m));
        hashBytes(hash, frame.gait_phase.data(), sizeof(frame.gait_phase));
        hashBytes(hash, &frame.duty_factor, sizeof(frame.duty_factor));
        hashBytes(hash, &frame.stride_phase_rate_hz, sizeof(frame.stride_phase_rate_hz));
        hashBytes(hash, &frame.command_scale, sizeof(frame.command_scale));
        hashBytes(hash, &frame.cadence_scale, sizeof(frame.cadence_scale));
        for (const bool clamp_hit : frame.stroke_clamp_hit) {
            const std::uint8_t value = clamp_hit ? 1U : 0U;
            hashBytes(hash, &value, sizeof(value));
        }
        for (const bool workspace_hit : frame.workspace_xy_hit) {
            const std::uint8_t value = workspace_hit ? 1U : 0U;
            hashBytes(hash, &value, sizeof(value));
        }
        for (const Vec3& planned : frame.planned_target_body) {
            hashBytes(hash, &planned.x, sizeof(planned.x));
            hashBytes(hash, &planned.y, sizeof(planned.y));
            hashBytes(hash, &planned.z, sizeof(planned.z));
        }
        for (const Vec3& pre_slew : frame.pre_slew_fk_body) {
            hashBytes(hash, &pre_slew.x, sizeof(pre_slew.x));
            hashBytes(hash, &pre_slew.y, sizeof(pre_slew.y));
            hashBytes(hash, &pre_slew.z, sizeof(pre_slew.z));
        }
        for (const Vec3& post_clamp : frame.post_clamp_fk_body) {
            hashBytes(hash, &post_clamp.x, sizeof(post_clamp.x));
            hashBytes(hash, &post_clamp.y, sizeof(post_clamp.y));
            hashBytes(hash, &post_clamp.z, sizeof(post_clamp.z));
        }
        for (const bool reach_hit : frame.ik_reach_clamp_hit) {
            const std::uint8_t value = reach_hit ? 1U : 0U;
            hashBytes(hash, &value, sizeof(value));
        }
        for (const bool slew_hit : frame.slew_clamp_hit) {
            const std::uint8_t value = slew_hit ? 1U : 0U;
            hashBytes(hash, &value, sizeof(value));
        }
        hashBytes(hash,
                  frame.post_clamp_distortion_m.data(),
                  sizeof(frame.post_clamp_distortion_m));
        hashBytes(hash, &frame.static_stability_margin_m,
                  sizeof(frame.static_stability_margin_m));
        for (const LegState& leg : frame.targets.leg_states) {
            for (const JointState& joint : leg.joint_state) {
                hashBytes(hash, &joint.pos_rad.value, sizeof(joint.pos_rad.value));
                hashBytes(hash, &joint.vel_radps.value, sizeof(joint.vel_radps.value));
            }
        }
    }
    return hash;
}

std::uint16_t iterationPercentile(const IterationHistogram& histogram,
                                  const double percentile) {
    std::uint64_t sample_count = 0;
    for (const auto& [iterations, frames] : histogram) {
        (void)iterations;
        sample_count += frames;
    }
    if (sample_count == 0) {
        return 0;
    }
    const std::uint64_t rank = std::max<std::uint64_t>(
        1U,
        static_cast<std::uint64_t>(
            std::ceil(percentile * static_cast<double>(sample_count))));
    std::uint64_t cumulative = 0;
    for (const auto& [iterations, frames] : histogram) {
        cumulative += frames;
        if (cumulative >= rank) {
            return iterations;
        }
    }
    return histogram.rbegin()->first;
}

constexpr const char* kCommandFixtureHeader = "hexapod-exact-command-replay-v8";

void saveCommandFixture(const std::string& path,
                        const CommandFixture& fixture) {
    std::ofstream output(path, std::ios::out | std::ios::trunc);
    if (!output) {
        throw std::runtime_error("failed to open command fixture for writing: " + path);
    }
    output << std::setprecision(std::numeric_limits<double>::max_digits10)
           << kCommandFixtureHeader << '\n'
           << "capture_period_us " << fixture.capture_period_us << '\n'
           << "stand_frames " << fixture.stand_frames << '\n'
           << "motion_frames " << fixture.motion_frames << '\n'
           << "transition_frames " << fixture.transition_frames << '\n'
           << "commanded_body_height_m " << fixture.commanded_body_height_m << '\n'
           << "initial_body_position " << fixture.initial_body_position[0] << ' '
           << fixture.initial_body_position[1] << ' '
           << fixture.initial_body_position[2] << '\n'
           << "initial_body_orientation " << fixture.initial_body_orientation[0] << ' '
           << fixture.initial_body_orientation[1] << ' '
           << fixture.initial_body_orientation[2] << ' '
           << fixture.initial_body_orientation[3] << '\n'
           << "selected_phase " << fixture.selected_phase << '\n'
           << "capture_solver_mode "
           << static_cast<unsigned>(fixture.capture_solver_mode) << '\n'
           << "capture_solver_iterations " << fixture.capture_solver_iterations << '\n'
           << "frame_count " << fixture.frames.size() << '\n';
    for (const CapturedFrame& frame : fixture.frames) {
        output << static_cast<unsigned>(frame.phase) << ' '
               << (frame.inhibit_motion ? 1 : 0) << ' '
               << (frame.walk_mode ? 1 : 0);
        for (const bool planned_stance : frame.planned_stance) {
            output << ' ' << (planned_stance ? 1 : 0);
        }
        for (const bool hold_stance : frame.hold_stance) {
            output << ' ' << (hold_stance ? 1 : 0);
        }
        for (const bool safe_to_lift : frame.safe_to_lift) {
            output << ' ' << (safe_to_lift ? 1 : 0);
        }
        for (const double clearance_m : frame.liftoff_clearance_m) {
            output << ' ' << clearance_m;
        }
        output << ' ' << frame.static_stability_margin_m
               << ' ' << frame.duty_factor
               << ' ' << frame.stride_phase_rate_hz
               << ' ' << frame.command_scale
               << ' ' << frame.cadence_scale;
        for (const bool clamp_hit : frame.stroke_clamp_hit) {
            output << ' ' << (clamp_hit ? 1 : 0);
        }
        for (const bool workspace_hit : frame.workspace_xy_hit) {
            output << ' ' << (workspace_hit ? 1 : 0);
        }
        for (const Vec3& planned : frame.planned_target_body) {
            output << ' ' << planned.x << ' ' << planned.y << ' ' << planned.z;
        }
        for (const Vec3& pre_slew : frame.pre_slew_fk_body) {
            output << ' ' << pre_slew.x << ' ' << pre_slew.y << ' ' << pre_slew.z;
        }
        for (const Vec3& post_clamp : frame.post_clamp_fk_body) {
            output << ' ' << post_clamp.x << ' ' << post_clamp.y << ' ' << post_clamp.z;
        }
        for (const bool reach_hit : frame.ik_reach_clamp_hit) {
            output << ' ' << (reach_hit ? 1 : 0);
        }
        for (const bool slew_hit : frame.slew_clamp_hit) {
            output << ' ' << (slew_hit ? 1 : 0);
        }
        for (const double distortion_m : frame.post_clamp_distortion_m) {
            output << ' ' << distortion_m;
        }
        for (const double phase01 : frame.gait_phase) {
            output << ' ' << phase01;
        }
        for (const LegState& leg : frame.targets.leg_states) {
            for (const JointState& joint : leg.joint_state) {
                output << ' ' << static_cast<double>(joint.pos_rad.value)
                       << ' ' << static_cast<double>(joint.vel_radps.value);
            }
        }
        output << '\n';
    }
    if (!output) {
        throw std::runtime_error("failed while writing command fixture: " + path);
    }
}

CommandFixture loadCommandFixture(const std::string& path) {
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("failed to open command fixture for reading: " + path);
    }
    std::string header;
    std::getline(input, header);
    if (header != kCommandFixtureHeader) {
        throw std::runtime_error("unsupported command fixture header: " + path);
    }
    const auto expectKey = [&](const char* expected) {
        std::string key;
        if (!(input >> key) || key != expected) {
            throw std::runtime_error(
                std::string("missing command fixture field ") + expected + ": " + path);
        }
    };
    CommandFixture fixture{};
    expectKey("capture_period_us");
    input >> fixture.capture_period_us;
    expectKey("stand_frames");
    input >> fixture.stand_frames;
    expectKey("motion_frames");
    input >> fixture.motion_frames;
    expectKey("transition_frames");
    input >> fixture.transition_frames;
    expectKey("commanded_body_height_m");
    input >> fixture.commanded_body_height_m;
    expectKey("initial_body_position");
    input >> fixture.initial_body_position[0]
          >> fixture.initial_body_position[1]
          >> fixture.initial_body_position[2];
    expectKey("initial_body_orientation");
    input >> fixture.initial_body_orientation[0]
          >> fixture.initial_body_orientation[1]
          >> fixture.initial_body_orientation[2]
          >> fixture.initial_body_orientation[3];
    expectKey("selected_phase");
    input >> fixture.selected_phase;
    unsigned capture_solver_mode = 0;
    expectKey("capture_solver_mode");
    input >> capture_solver_mode;
    expectKey("capture_solver_iterations");
    input >> fixture.capture_solver_iterations;
    std::size_t frame_count = 0;
    expectKey("frame_count");
    input >> frame_count;
    const bool finite_pose = std::all_of(
        fixture.initial_body_position.begin(),
        fixture.initial_body_position.end(),
        [](const double value) { return std::isfinite(value); })
        && std::all_of(
            fixture.initial_body_orientation.begin(),
            fixture.initial_body_orientation.end(),
            [](const double value) { return std::isfinite(value); });
    if (!input || fixture.capture_period_us <= 0 || fixture.stand_frames <= 0
        || fixture.motion_frames <= 0 || fixture.transition_frames <= 0
        || !std::isfinite(fixture.commanded_body_height_m)
        || fixture.commanded_body_height_m <= 0.0 || !finite_pose
        || fixture.selected_phase < -1
        || fixture.selected_phase >= static_cast<int>(ReplayPhase::Count)
        || capture_solver_mode > static_cast<unsigned>(
            physics_sim::PhysicsSolverMode::PinocchioProximal)
        || fixture.capture_solver_iterations <= 0 || frame_count > 1'000'000U) {
        throw std::runtime_error("invalid command fixture frame count: " + path);
    }
    fixture.capture_solver_mode = static_cast<physics_sim::PhysicsSolverMode>(
        capture_solver_mode);
    fixture.frames.reserve(frame_count);
    for (std::size_t frame_index = 0; frame_index < frame_count; ++frame_index) {
        unsigned phase = 0;
        int inhibit_motion = 0;
        int walk_mode = 0;
        if (!(input >> phase >> inhibit_motion >> walk_mode)
            || phase >= static_cast<unsigned>(ReplayPhase::Count)
            || (inhibit_motion != 0 && inhibit_motion != 1)
            || (walk_mode != 0 && walk_mode != 1)) {
            throw std::runtime_error(
                "invalid command fixture metadata at frame "
                + std::to_string(frame_index) + ": " + path);
        }
        CapturedFrame frame{};
        frame.phase = static_cast<ReplayPhase>(phase);
        frame.inhibit_motion = inhibit_motion != 0;
        frame.walk_mode = walk_mode != 0;
        for (bool& planned_stance : frame.planned_stance) {
            int value = 0;
            if (!(input >> value) || (value != 0 && value != 1)) {
                throw std::runtime_error(
                    "invalid planned-stance metadata at frame "
                    + std::to_string(frame_index) + ": " + path);
            }
            planned_stance = value != 0;
        }
        for (bool& hold_stance : frame.hold_stance) {
            int value = 0;
            if (!(input >> value) || (value != 0 && value != 1)) {
                throw std::runtime_error(
                    "invalid held-stance metadata at frame "
                    + std::to_string(frame_index) + ": " + path);
            }
            hold_stance = value != 0;
        }
        for (bool& safe_to_lift : frame.safe_to_lift) {
            int value = 0;
            if (!(input >> value) || (value != 0 && value != 1)) {
                throw std::runtime_error(
                    "invalid safe-to-lift metadata at frame "
                    + std::to_string(frame_index) + ": " + path);
            }
            safe_to_lift = value != 0;
        }
        for (double& clearance_m : frame.liftoff_clearance_m) {
            if (!(input >> clearance_m) || !std::isfinite(clearance_m)) {
                throw std::runtime_error(
                    "invalid liftoff-clearance metadata at frame "
                    + std::to_string(frame_index) + ": " + path);
            }
        }
        if (!(input >> frame.static_stability_margin_m)
            || !std::isfinite(frame.static_stability_margin_m)) {
            throw std::runtime_error(
                "invalid static-margin metadata at frame "
                + std::to_string(frame_index) + ": " + path);
        }
        if (!(input >> frame.duty_factor)
            || !std::isfinite(frame.duty_factor)
            || frame.duty_factor < 0.0
            || frame.duty_factor > 1.0) {
            throw std::runtime_error(
                "invalid duty-factor metadata at frame "
                + std::to_string(frame_index) + ": " + path);
        }
        if (!(input >> frame.stride_phase_rate_hz)
            || !std::isfinite(frame.stride_phase_rate_hz)
            || frame.stride_phase_rate_hz < 0.0) {
            throw std::runtime_error(
                "invalid stride-phase-rate metadata at frame "
                + std::to_string(frame_index) + ": " + path);
        }
        if (!(input >> frame.command_scale)
            || !std::isfinite(frame.command_scale)
            || frame.command_scale < 0.0) {
            throw std::runtime_error(
                "invalid command-scale metadata at frame "
                + std::to_string(frame_index) + ": " + path);
        }
        if (!(input >> frame.cadence_scale)
            || !std::isfinite(frame.cadence_scale)
            || frame.cadence_scale < 0.0) {
            throw std::runtime_error(
                "invalid cadence-scale metadata at frame "
                + std::to_string(frame_index) + ": " + path);
        }
        for (bool& clamp_hit : frame.stroke_clamp_hit) {
            int value = 0;
            if (!(input >> value) || (value != 0 && value != 1)) {
                throw std::runtime_error(
                    "invalid stroke-clamp-hit metadata at frame "
                    + std::to_string(frame_index) + ": " + path);
            }
            clamp_hit = value != 0;
        }
        for (bool& workspace_hit : frame.workspace_xy_hit) {
            int value = 0;
            if (!(input >> value) || (value != 0 && value != 1)) {
                throw std::runtime_error(
                    "invalid workspace-xy-hit metadata at frame "
                    + std::to_string(frame_index) + ": " + path);
            }
            workspace_hit = value != 0;
        }
        for (Vec3& planned : frame.planned_target_body) {
            if (!(input >> planned.x >> planned.y >> planned.z)
                || !std::isfinite(planned.x)
                || !std::isfinite(planned.y)
                || !std::isfinite(planned.z)) {
                throw std::runtime_error(
                    "invalid planned-target metadata at frame "
                    + std::to_string(frame_index) + ": " + path);
            }
        }
        for (Vec3& pre_slew : frame.pre_slew_fk_body) {
            if (!(input >> pre_slew.x >> pre_slew.y >> pre_slew.z)
                || !std::isfinite(pre_slew.x)
                || !std::isfinite(pre_slew.y)
                || !std::isfinite(pre_slew.z)) {
                throw std::runtime_error(
                    "invalid pre-slew FK metadata at frame "
                    + std::to_string(frame_index) + ": " + path);
            }
        }
        for (Vec3& post_clamp : frame.post_clamp_fk_body) {
            if (!(input >> post_clamp.x >> post_clamp.y >> post_clamp.z)
                || !std::isfinite(post_clamp.x)
                || !std::isfinite(post_clamp.y)
                || !std::isfinite(post_clamp.z)) {
                throw std::runtime_error(
                    "invalid post-clamp FK metadata at frame "
                    + std::to_string(frame_index) + ": " + path);
            }
        }
        for (bool& reach_hit : frame.ik_reach_clamp_hit) {
            int value = 0;
            if (!(input >> value) || (value != 0 && value != 1)) {
                throw std::runtime_error(
                    "invalid IK reach-clamp metadata at frame "
                    + std::to_string(frame_index) + ": " + path);
            }
            reach_hit = value != 0;
        }
        for (bool& slew_hit : frame.slew_clamp_hit) {
            int value = 0;
            if (!(input >> value) || (value != 0 && value != 1)) {
                throw std::runtime_error(
                    "invalid slew-clamp metadata at frame "
                    + std::to_string(frame_index) + ": " + path);
            }
            slew_hit = value != 0;
        }
        for (double& distortion_m : frame.post_clamp_distortion_m) {
            if (!(input >> distortion_m) || !std::isfinite(distortion_m)) {
                throw std::runtime_error(
                    "invalid post-clamp distortion metadata at frame "
                    + std::to_string(frame_index) + ": " + path);
            }
        }
        for (double& phase01 : frame.gait_phase) {
            if (!(input >> phase01)
                || !std::isfinite(phase01)
                || phase01 < 0.0
                || phase01 > 1.0) {
                throw std::runtime_error(
                    "invalid gait-phase metadata at frame "
                    + std::to_string(frame_index) + ": " + path);
            }
        }
        for (LegState& leg : frame.targets.leg_states) {
            for (JointState& joint : leg.joint_state) {
                double position = 0.0;
                double velocity = 0.0;
                if (!(input >> position >> velocity)
                    || !std::isfinite(position) || !std::isfinite(velocity)) {
                    throw std::runtime_error(
                        "invalid command fixture target at frame "
                        + std::to_string(frame_index) + ": " + path);
                }
                joint.pos_rad.value = position;
                joint.vel_radps.value = velocity;
            }
        }
        fixture.frames.push_back(frame);
    }
    input >> std::ws;
    if (!input.eof()) {
        throw std::runtime_error("unexpected trailing command fixture data: " + path);
    }
    return fixture;
}

void validateCommandFixture(const CommandFixture& fixture) {
    const bool single_phase = fixture.selected_phase >= 0;
    if (single_phase
        && fixture.selected_phase < static_cast<int>(ReplayPhase::Forward)) {
        throw std::runtime_error("command fixture selected phase is not a motion phase");
    }
    const double orientation_norm = std::sqrt(
        fixture.initial_body_orientation[0] * fixture.initial_body_orientation[0]
        + fixture.initial_body_orientation[1] * fixture.initial_body_orientation[1]
        + fixture.initial_body_orientation[2] * fixture.initial_body_orientation[2]
        + fixture.initial_body_orientation[3] * fixture.initial_body_orientation[3]);
    if (!std::isfinite(orientation_norm) || std::abs(orientation_norm - 1.0) > 1.0e-9) {
        throw std::runtime_error("command fixture initial orientation is not normalized");
    }

    std::array<std::size_t, kPhaseCount> phase_counts{};
    std::array<std::size_t, kPhaseCount> inhibited_counts{};
    std::array<std::size_t, kPhaseCount> non_walk_counts{};
    for (const CapturedFrame& frame : fixture.frames) {
        if (!targetsAreFinite(frame.targets)) {
            throw std::runtime_error("command fixture contains non-finite targets");
        }
        const std::size_t phase_index = static_cast<std::size_t>(frame.phase);
        ++phase_counts[phase_index];
        inhibited_counts[phase_index] += frame.inhibit_motion ? 1U : 0U;
        non_walk_counts[phase_index] += frame.walk_mode ? 0U : 1U;
    }
    const std::size_t motion_case_count = single_phase ? 1U : 5U;
    const std::size_t expected_frames = static_cast<std::size_t>(fixture.stand_frames)
        + motion_case_count * static_cast<std::size_t>(
            fixture.motion_frames + fixture.transition_frames);
    if (fixture.frames.size() != expected_frames
        || phase_counts[static_cast<std::size_t>(ReplayPhase::Stand)]
            != static_cast<std::size_t>(fixture.stand_frames)
        || phase_counts[static_cast<std::size_t>(ReplayPhase::Transition)]
            != motion_case_count * static_cast<std::size_t>(fixture.transition_frames)) {
        throw std::runtime_error("command fixture phase counts do not match its metadata");
    }
    constexpr std::array<ReplayPhase, 5> motion_phases{
        ReplayPhase::Forward,
        ReplayPhase::Reverse,
        ReplayPhase::Strafe,
        ReplayPhase::Diagonal,
        ReplayPhase::TurnInPlace,
    };
    for (const ReplayPhase phase : motion_phases) {
        const std::size_t phase_index = static_cast<std::size_t>(phase);
        const bool selected = !single_phase
            || fixture.selected_phase == static_cast<int>(phase);
        const std::size_t expected = selected
            ? static_cast<std::size_t>(fixture.motion_frames)
            : 0U;
        if (phase_counts[phase_index] != expected) {
            throw std::runtime_error(
                std::string("command fixture frame count mismatch for ")
                + kPhaseNames[phase_index]);
        }
        if (selected && (inhibited_counts[phase_index] != 0
                         || non_walk_counts[phase_index] != 0)) {
            throw std::runtime_error(
                std::string("command fixture contains inhibited or non-WALK frames for ")
                + kPhaseNames[phase_index]);
        }
    }
}

MotionIntent makeReplayIntent(const ReplayPhase phase, const double body_height_m) {
    ScenarioMotionIntent scenario{};
    scenario.enabled = true;
    scenario.mode = phase == ReplayPhase::Stand || phase == ReplayPhase::Transition
        ? RobotMode::STAND
        : RobotMode::WALK;
    scenario.gait = GaitType::TRIPOD;
    scenario.body_height_m = body_height_m;
    scenario.has_direct_velocity = true;
    switch (phase) {
    case ReplayPhase::Forward:
        scenario.vx_mps = 0.12;
        break;
    case ReplayPhase::Reverse:
        scenario.vx_mps = -0.12;
        break;
    case ReplayPhase::Strafe:
        scenario.vy_mps = 0.10;
        break;
    case ReplayPhase::Diagonal:
        scenario.vx_mps = 0.085;
        scenario.vy_mps = 0.085;
        break;
    case ReplayPhase::TurnInPlace:
        scenario.yaw_rate_radps = 0.45;
        break;
    case ReplayPhase::Stand:
    case ReplayPhase::Transition:
    case ReplayPhase::Count:
        break;
    }
    return makeMotionIntent(scenario);
}

int positiveEnvOrDefault(const char* name, const int fallback) {
    const char* value = std::getenv(name);
    if (value == nullptr || value[0] == '\0') {
        return fallback;
    }
    char* end = nullptr;
    const long parsed = std::strtol(value, &end, 10);
    if (end == value || *end != '\0' || parsed <= 0 || parsed > 1000000) {
        throw std::runtime_error(std::string("invalid ") + name + "=" + value);
    }
    return static_cast<int>(parsed);
}

int nonnegativeEnvOrDefault(const char* name, const int fallback) {
    const char* value = std::getenv(name);
    if (value == nullptr || value[0] == '\0') {
        return fallback;
    }
    char* end = nullptr;
    const long parsed = std::strtol(value, &end, 10);
    if (end == value || *end != '\0' || parsed < 0 || parsed > 1000000) {
        throw std::runtime_error(std::string("invalid ") + name + "=" + value);
    }
    return static_cast<int>(parsed);
}

double positiveDoubleEnvOrDefault(const char* name, const double fallback) {
    const char* value = std::getenv(name);
    if (value == nullptr || value[0] == '\0') {
        return fallback;
    }
    char* end = nullptr;
    const double parsed = std::strtod(value, &end);
    if (end == value || *end != '\0' || !std::isfinite(parsed) || parsed <= 0.0) {
        throw std::runtime_error(std::string("invalid ") + name + "=" + value);
    }
    return parsed;
}

bool envEnabled(const char* name) {
    const char* value = std::getenv(name);
    return value != nullptr && value[0] != '\0' && value[0] != '0';
}

bool envProvided(const char* name) {
    const char* value = std::getenv(name);
    return value != nullptr && value[0] != '\0';
}

std::uint64_t splitMix64(std::uint64_t value) {
    value += 0x9e3779b97f4a7c15ULL;
    value = (value ^ (value >> 30U)) * 0xbf58476d1ce4e5b9ULL;
    value = (value ^ (value >> 27U)) * 0x94d049bb133111ebULL;
    return value ^ (value >> 31U);
}

double seededUnitInterval(const std::uint64_t seed, const std::uint64_t channel) {
    const std::uint64_t bits = splitMix64(seed ^ (channel * 0x9e3779b97f4a7c15ULL));
    return static_cast<double>(bits >> 11U) * (1.0 / 9007199254740992.0);
}

std::array<double, 4> multiplyQuaternion(const std::array<double, 4>& lhs,
                                         const std::array<double, 4>& rhs) {
    return {
        lhs[0] * rhs[0] - lhs[1] * rhs[1] - lhs[2] * rhs[2] - lhs[3] * rhs[3],
        lhs[0] * rhs[1] + lhs[1] * rhs[0] + lhs[2] * rhs[3] - lhs[3] * rhs[2],
        lhs[0] * rhs[2] - lhs[1] * rhs[3] + lhs[2] * rhs[0] + lhs[3] * rhs[1],
        lhs[0] * rhs[3] + lhs[1] * rhs[2] - lhs[2] * rhs[1] + lhs[3] * rhs[0],
    };
}

physics_sim::StateCorrection makePerturbedStandingCorrection(
    const std::uint64_t seed,
    const std::array<double, 3>& base_position,
    const std::array<double, 4>& base_orientation,
    const double scale) {
    const auto symmetric = [seed](const std::uint64_t channel) {
        return 2.0 * seededUnitInterval(seed, channel) - 1.0;
    };
    const double horizontal_x = seed == 0 ? 0.0 : scale * 0.003 * symmetric(1);
    const double vertical = seed == 0 ? 0.0 : scale * 0.0015 * symmetric(2);
    const double horizontal_z = seed == 0 ? 0.0 : scale * 0.003 * symmetric(3);
    constexpr double kDegreesToRadians = 0.01745329251994329577;
    const double roll = seed == 0 ? 0.0 : scale * 0.75 * kDegreesToRadians * symmetric(4);
    const double yaw = seed == 0 ? 0.0 : scale * 1.0 * kDegreesToRadians * symmetric(5);
    const double pitch = seed == 0 ? 0.0 : scale * 0.75 * kDegreesToRadians * symmetric(6);
    const std::array<double, 4> qx{
        std::cos(0.5 * roll), std::sin(0.5 * roll), 0.0, 0.0};
    const std::array<double, 4> qy{
        std::cos(0.5 * yaw), 0.0, std::sin(0.5 * yaw), 0.0};
    const std::array<double, 4> qz{
        std::cos(0.5 * pitch), 0.0, 0.0, std::sin(0.5 * pitch)};
    const std::array<double, 4> perturbation =
        multiplyQuaternion(qy, multiplyQuaternion(qz, qx));
    const std::array<double, 4> orientation =
        multiplyQuaternion(perturbation, base_orientation);

    physics_sim::StateCorrection correction{};
    correction.message_type = static_cast<std::uint8_t>(physics_sim::MessageType::StateCorrection);
    correction.sequence_id = static_cast<std::uint32_t>(seed);
    correction.timestamp_us = now_us().value;
    correction.flags = physics_sim::kStateCorrectionPoseValid
        | physics_sim::kStateCorrectionTwistValid
        | physics_sim::kStateCorrectionHardReset;
    correction.correction_strength = 1.0f;
    correction.body_position = {
        static_cast<float>(base_position[0] + horizontal_x),
        static_cast<float>(base_position[1] + vertical),
        static_cast<float>(base_position[2] + horizontal_z)};
    correction.body_orientation = {
        static_cast<float>(orientation[0]),
        static_cast<float>(orientation[1]),
        static_cast<float>(orientation[2]),
        static_cast<float>(orientation[3])};
    correction.body_linear_velocity = {0.0f, 0.0f, 0.0f};
    correction.body_angular_velocity = {0.0f, 0.0f, 0.0f};
    return correction;
}

std::optional<ReplayPhase> selectedMotionPhase() {
    const char* value = std::getenv("HEXAPOD_EXACT_REPLAY_MOTION_CASE");
    if (value == nullptr || value[0] == '\0' || std::string(value) == "all") {
        return std::nullopt;
    }
    constexpr std::array<ReplayPhase, 5> motion_phases{
        ReplayPhase::Forward,
        ReplayPhase::Reverse,
        ReplayPhase::Strafe,
        ReplayPhase::Diagonal,
        ReplayPhase::TurnInPlace,
    };
    for (const ReplayPhase phase : motion_phases) {
        if (std::string(value) == kPhaseNames[static_cast<std::size_t>(phase)]) {
            return phase;
        }
    }
    throw std::runtime_error(
        std::string("invalid HEXAPOD_EXACT_REPLAY_MOTION_CASE=") + value);
}

#if defined(__linux__)
pid_t launchSimulator(const char* sim_exe,
                      const int port,
                      const std::uint64_t contact_order_seed = 0) {
    const pid_t pid = ::fork();
    if (pid != 0) {
        return pid;
    }
    if (!envEnabled("HEXAPOD_EXACT_REPLAY_CHILD_STDIO")) {
        physics_sim_test_utils::quietChildProcessStdIo();
    }
    const std::string contact_order_seed_text = std::to_string(contact_order_seed);
    (void)::setenv("HEXAPOD_PINOCCHIO_CONTACT_ORDER_SEED",
                   contact_order_seed_text.c_str(),
                   1);
    const std::string port_text = std::to_string(port);
    ::execl(sim_exe, sim_exe, "--serve", "--serve-port", port_text.c_str(), nullptr);
    std::perror("execl");
    _exit(127);
}

void stopSimulator(const pid_t pid) {
    if (pid > 0) {
        (void)::kill(pid, SIGTERM);
        (void)::waitpid(pid, nullptr, 0);
    }
}
#endif

void runRuntimeFrame(RobotRuntime& runtime,
                     CommandCapturingBridge& bridge,
                     const ReplayPhase phase,
                     MotionIntent intent,
                     TimePointUs& command_time,
                     const int bus_loop_period_us) {
    bridge.setPhase(phase);
    command_time.value += static_cast<std::uint64_t>(bus_loop_period_us);
    intent.timestamp_us = command_time;
    intent.sample_id = 0;
    runtime.setMotionIntent(intent);
    runtime.busStep();
    runtime.estimatorStep();
    runtime.safetyStep();
    runtime.controlStep();
    bridge.annotateLastFrame(
        runtime.getSafetyState(),
        runtime.getStatus(),
        runtime.gaitSnapshot(),
        runtime.commandGovernorSnapshot(),
        runtime.locomotionDebugSnapshot(),
        runtime.strokeClampHitSnapshot(),
        runtime.workspaceXyHitSnapshot(),
        runtime.ikReachClampHitSnapshot(),
        runtime.slewClampHitSnapshot());
}

std::vector<CapturedFrame> captureCommands(const physics_sim_test_utils::HarnessSettings& harness,
                                           const int port,
                                           const int stand_frames,
                                           const int motion_frames,
                                           const int transition_frames,
                                           const double body_height_m,
                                           const std::optional<ReplayPhase> selected_phase,
                                           const physics_sim::PhysicsSolverMode capture_solver_mode,
                                           const int capture_solver_iterations) {
    PhysicsSimSolverSettings capture_solver{};
    capture_solver.mode = capture_solver_mode;
    capture_solver.iterations = capture_solver_iterations;
    auto bridge = std::make_unique<CommandCapturingBridge>(
        "127.0.0.1", port, harness.bus_loop_period_us, capture_solver);
    CommandCapturingBridge* bridge_ptr = bridge.get();

    control_config::ControlConfig config = harness.control_cfg;
    config.freshness.estimator.max_allowed_age_us = DurationUs{10'000'000};
    config.freshness.intent.max_allowed_age_us = DurationUs{10'000'000};
    RobotRuntime runtime(std::move(bridge), std::make_unique<PhysicsSimEstimator>(), nullptr, config);
    if (!runtime.init()) {
        throw std::runtime_error("legacy command-capture runtime failed to initialise");
    }

    TimePointUs command_time{now_us().value + 3'600'000'000ULL};
    const auto run_phase = [&](const ReplayPhase phase, const int frames) {
        const MotionIntent intent = makeReplayIntent(phase, body_height_m);
        for (int i = 0; i < frames; ++i) {
            runRuntimeFrame(runtime,
                            *bridge_ptr,
                            phase,
                            intent,
                            command_time,
                            harness.bus_loop_period_us);
        }
        if (envEnabled("HEXAPOD_EXACT_REPLAY_TRACE_CAPTURE")) {
            const SafetyState safety = runtime.getSafetyState();
            const ControlStatus status = runtime.getStatus();
            const RobotState estimated = runtime.estimatedSnapshot();
            const GaitState gait = runtime.gaitSnapshot();
            const std::size_t held_legs = static_cast<std::size_t>(std::count(
                gait.stability_hold_stance.begin(),
                gait.stability_hold_stance.end(),
                true));
            std::cerr << "capture phase=" << kPhaseNames[static_cast<std::size_t>(phase)]
                      << " frames=" << frames
                      << " inhibit=" << (safety.inhibit_motion ? 1 : 0)
                      << " fault=" << static_cast<unsigned>(safety.active_fault)
                      << " lifecycle=" << static_cast<unsigned>(safety.fault_lifecycle)
                      << " mode=" << static_cast<unsigned>(status.active_mode)
                      << " estimator_valid=" << (status.estimator_valid ? 1 : 0)
                      << " bus_ok=" << (status.bus_ok ? 1 : 0)
                      << " body_roll_rad=" << estimated.body_twist_state.twist_pos_rad.x
                      << " body_pitch_rad=" << estimated.body_twist_state.twist_pos_rad.y
                      << " planar_body_rate_radps="
                      << std::hypot(estimated.imu.gyro_radps.x, estimated.imu.gyro_radps.y)
                      << " held_legs=" << held_legs << '\n';
        }
    };

    run_phase(ReplayPhase::Stand, stand_frames);
    constexpr std::array<ReplayPhase, 5> motion_phases{
        ReplayPhase::Forward,
        ReplayPhase::Reverse,
        ReplayPhase::Strafe,
        ReplayPhase::Diagonal,
        ReplayPhase::TurnInPlace,
    };
    for (const ReplayPhase phase : motion_phases) {
        if (selected_phase.has_value() && phase != *selected_phase) {
            continue;
        }
        run_phase(phase, motion_frames);
        run_phase(ReplayPhase::Transition, transition_frames);
    }
    return bridge_ptr->frames();
}

ReplayResult replayCommands(const std::vector<CapturedFrame>& frames,
                            const int port,
                            const int replay_period_us,
                            const physics_sim::PhysicsSolverMode solver_mode,
                            const int solver_iterations,
                            const double body_height_m,
                            const std::array<double, 3>& initial_body_position,
                            const std::array<double, 4>& initial_body_orientation,
                            const double proximal_mu,
                            const double contact_regularization,
                            const double absolute_tolerance,
                            const double relative_tolerance,
                            const std::uint64_t perturbation_seed,
                            const double perturbation_scale) {
    PhysicsSimSolverSettings solver{};
    solver.mode = solver_mode;
    solver.iterations = solver_iterations;
    solver.proximal_mu = static_cast<float>(proximal_mu);
    solver.contact_regularization = static_cast<float>(contact_regularization);
    solver.absolute_tolerance = static_cast<float>(absolute_tolerance);
    solver.relative_tolerance = static_cast<float>(relative_tolerance);
    PhysicsSimBridge bridge(
        "127.0.0.1", port, replay_period_us, solver, nullptr);
    if (!bridge.init()) {
        throw std::runtime_error("proximal replay bridge failed to initialise");
    }
    if (!bridge.sendStateCorrection(
            makePerturbedStandingCorrection(
                perturbation_seed,
                initial_body_position,
                initial_body_orientation,
                perturbation_scale))) {
        throw std::runtime_error("proximal replay failed to send initial-pose perturbation");
    }

    ReplayResult result{};
    std::vector<double> step_times_ms{};
    std::vector<double> healthy_step_times_ms{};
    std::vector<double> solver_dynamics_times_ms{};
    std::vector<double> solver_contact_setup_times_ms{};
    std::vector<double> solver_collision_times_ms{};
    std::vector<double> solver_constraint_assembly_times_ms{};
    std::vector<double> solver_delassus_times_ms{};
    std::vector<double> solver_admm_times_ms{};
    std::vector<double> solver_integration_times_ms{};
    std::vector<double> solver_total_step_times_ms{};
    step_times_ms.reserve(frames.size());
    healthy_step_times_ms.reserve(frames.size());
    solver_dynamics_times_ms.reserve(frames.size());
    solver_contact_setup_times_ms.reserve(frames.size());
    solver_collision_times_ms.reserve(frames.size());
    solver_constraint_assembly_times_ms.reserve(frames.size());
    solver_delassus_times_ms.reserve(frames.size());
    solver_admm_times_ms.reserve(frames.size());
    solver_integration_times_ms.reserve(frames.size());
    solver_total_step_times_ms.reserve(frames.size());

    struct ActivePhaseSegment {
        ReplayPhase phase{ReplayPhase::Stand};
        std::optional<Vec3> start_position{};
        std::optional<Vec3> last_position{};
        double start_yaw{0.0};
        double last_raw_yaw{0.0};
        double accumulated_yaw{0.0};
        double horizontal_path{0.0};
        Vec3 start_velocity{};
        Vec3 last_velocity{};
        std::uint64_t valid_frames{0};
        std::optional<Vec3> evaluation_start_position{};
        std::optional<Vec3> evaluation_last_position{};
        double evaluation_start_yaw{0.0};
        double evaluation_last_raw_yaw{0.0};
        double evaluation_accumulated_yaw{0.0};
        double evaluation_horizontal_path{0.0};
        std::array<std::uint64_t, kNumLegs> swing_frames{};
        std::array<std::uint64_t, kNumLegs> swing_contact_frames{};
        std::array<double, kNumLegs> swing_target_world_z_sum{};
        std::array<double, kNumLegs> swing_measured_world_z_sum{};
    };
    std::optional<ActivePhaseSegment> active_segment{};
    const std::uint64_t acceleration_transient_frames =
        std::max<std::uint64_t>(1, static_cast<std::uint64_t>(std::ceil(
            kAccelerationTransientDurationS
            / (static_cast<double>(replay_period_us) * 1.0e-6))));
    const HexapodGeometry geometry = defaultHexapodGeometry();
    LegFK leg_fk{};
    std::array<Vec3, kNumLegs> previous_contact_foot_world{};
    std::array<Vec3, kNumLegs> previous_contact_target_body{};
    std::array<Vec3, kNumLegs> previous_cartesian_body{};
    std::array<bool, kNumLegs> have_previous_contact_foot{};
    std::array<bool, kNumLegs> have_previous_raw_contact{};
    std::array<bool, kNumLegs> have_previous_cartesian{};
    std::array<bool, kNumLegs> previous_planned_stance{};
    std::optional<Vec3> previous_body_position{};
    std::optional<std::uint64_t> previous_contact_set_signature{};
    std::uint32_t contact_topology_age = 0;
    const auto finishSegment = [&]() {
        if (!active_segment.has_value()
            || !active_segment->start_position.has_value()
            || !active_segment->last_position.has_value()) {
            active_segment.reset();
            return;
        }
        PhaseResult& phase = result.phases[static_cast<std::size_t>(active_segment->phase)];
        const double dx = active_segment->last_position->x
            - active_segment->start_position->x;
        const double dy = active_segment->last_position->y
            - active_segment->start_position->y;
        const double c = std::cos(active_segment->start_yaw);
        const double s = std::sin(active_segment->start_yaw);
        phase.completed_delta_x_sum += dx;
        phase.completed_delta_y_sum += dy;
        phase.completed_body_forward_sum += c * dx + s * dy;
        phase.completed_body_lateral_sum += -s * dx + c * dy;
        phase.completed_yaw_delta_sum += active_segment->accumulated_yaw;
        phase.completed_horizontal_path_sum += active_segment->horizontal_path;
        phase.completed_horizontal_displacement_sum += std::hypot(dx, dy);
        phase.completed_start_body_forward_velocity_sum +=
            active_segment->start_velocity.x;
        phase.completed_start_body_lateral_velocity_sum +=
            active_segment->start_velocity.y;
        phase.completed_end_body_forward_velocity_sum +=
            active_segment->last_velocity.x;
        phase.completed_end_body_lateral_velocity_sum +=
            active_segment->last_velocity.y;
        if (active_segment->evaluation_start_position.has_value()
            && active_segment->evaluation_last_position.has_value()) {
            const double eval_dx = active_segment->evaluation_last_position->x
                - active_segment->evaluation_start_position->x;
            const double eval_dy = active_segment->evaluation_last_position->y
                - active_segment->evaluation_start_position->y;
            const double eval_c = std::cos(active_segment->evaluation_start_yaw);
            const double eval_s = std::sin(active_segment->evaluation_start_yaw);
            phase.evaluated_body_forward_sum += eval_c * eval_dx + eval_s * eval_dy;
            phase.evaluated_body_lateral_sum += -eval_s * eval_dx + eval_c * eval_dy;
            phase.evaluated_yaw_delta_sum += active_segment->evaluation_accumulated_yaw;
            phase.evaluated_horizontal_path_sum += active_segment->evaluation_horizontal_path;
            phase.evaluated_horizontal_displacement_sum += std::hypot(eval_dx, eval_dy);
            phase.evaluated_frames += active_segment->valid_frames > acceleration_transient_frames
                ? active_segment->valid_frames - acceleration_transient_frames
                : 0;
            ++phase.evaluated_trajectories;
        }
        if (const char* trace = std::getenv("HEXAPOD_EXACT_REPLAY_TRACE_SEGMENTS");
            trace != nullptr && trace[0] != '\0' && trace[0] != '0') {
            std::cerr << "segment " << kPhaseNames[static_cast<std::size_t>(active_segment->phase)]
                      << " delta_body=(" << (c * dx + s * dy) << ','
                      << (-s * dx + c * dy) << ") start_v_body=("
                      << active_segment->start_velocity.x
                      << ','
                      << active_segment->start_velocity.y
                      << ") end_v_body=("
                      << active_segment->last_velocity.x
                      << ','
                      << active_segment->last_velocity.y
                      << ")\n";
        }
        if (const char* trace = std::getenv("HEXAPOD_EXACT_REPLAY_TRACE_LEGS");
            trace != nullptr && trace[0] != '\0' && trace[0] != '0') {
            for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
                const double samples = static_cast<double>(
                    std::max<std::uint64_t>(active_segment->swing_frames[leg], 1));
                std::cerr << "segment_leg "
                          << kPhaseNames[static_cast<std::size_t>(active_segment->phase)]
                          << ' ' << kLegNames[leg]
                          << " swing_frames=" << active_segment->swing_frames[leg]
                          << " contact_fraction="
                          << static_cast<double>(active_segment->swing_contact_frames[leg]) / samples
                          << " target_world_z_mean="
                          << active_segment->swing_target_world_z_sum[leg] / samples
                          << " measured_world_z_mean="
                          << active_segment->swing_measured_world_z_sum[leg] / samples
                          << '\n';
            }
        }
        ++phase.completed_trajectories;
        active_segment.reset();
    };

    for (const CapturedFrame& frame : frames) {
        if (!active_segment.has_value() || active_segment->phase != frame.phase) {
            finishSegment();
            active_segment = ActivePhaseSegment{frame.phase};
            have_previous_contact_foot.fill(false);
            have_previous_raw_contact.fill(false);
            have_previous_cartesian.fill(false);
            previous_planned_stance.fill(false);
            previous_body_position.reset();
        }
        PhaseResult& phase = result.phases[static_cast<std::size_t>(frame.phase)];
        ++result.frames;
        ++phase.frames;
        if (!bridge.write(frame.targets)) {
            ++result.read_failures;
            ++phase.read_failures;
            continue;
        }

        RobotState state{};
        const auto step_start = std::chrono::steady_clock::now();
        const bool read_ok = bridge.read(state);
        const auto step_end = std::chrono::steady_clock::now();
        const double step_time_ms =
            std::chrono::duration<double, std::milli>(step_end - step_start).count();
        step_times_ms.push_back(step_time_ms);
        if (!read_ok) {
            ++result.read_failures;
            ++phase.read_failures;
        } else {
            const Vec3 position{state.body_twist_state.body_trans_m.x,
                                state.body_twist_state.body_trans_m.y,
                                state.body_twist_state.body_trans_m.z};
            const double yaw = state.body_twist_state.twist_pos_rad.z;
            phase.max_body_height_error = std::max(
                phase.max_body_height_error,
                std::abs(position.z - body_height_m));
            double frame_max_servo_tracking_error = 0.0;
            for (std::size_t leg = 0; leg < frame.targets.leg_states.size(); ++leg) {
                for (std::size_t joint = 0;
                     joint < frame.targets.leg_states[leg].joint_state.size();
                     ++joint) {
                    const double error = std::abs(std::remainder(
                        frame.targets.leg_states[leg].joint_state[joint].pos_rad.value
                            - state.leg_states[leg].joint_state[joint].pos_rad.value,
                        6.28318530717958647692));
                    phase.max_servo_tracking_error = std::max(
                        phase.max_servo_tracking_error, error);
                    frame_max_servo_tracking_error = std::max(
                        frame_max_servo_tracking_error, error);
                    phase.servo_tracking_error_sq_sum += error * error;
                    ++phase.servo_tracking_error_samples;
                }
            }
            phase.terminal_servo_tracking_error = frame_max_servo_tracking_error;
            BodyPose body_pose{};
            body_pose.position = state.body_twist_state.body_trans_m;
            body_pose.roll = AngleRad{state.body_twist_state.twist_pos_rad.x};
            body_pose.pitch = AngleRad{state.body_twist_state.twist_pos_rad.y};
            body_pose.yaw = AngleRad{state.body_twist_state.twist_pos_rad.z};
            const PhaseCommand command = phaseCommand(frame.phase);
            const double command_speed = std::hypot(command.vx_mps, command.vy_mps);
            const std::size_t n_planned = plannedStanceCount(frame.planned_stance);
            const std::size_t n_hold = plannedStanceCount(frame.hold_stance);
            std::size_t n_raw_contact = 0;
            std::size_t n_late_swing_extra = 0;
            std::size_t n_l_parked_contacted = 0;
            std::size_t n_stroking_contacted = 0;
            for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
                if (!state.foot_contacts[leg]) {
                    continue;
                }
                ++n_raw_contact;
                const bool late_swing_extra =
                    !frame.planned_stance[leg] && !frame.hold_stance[leg];
                if (late_swing_extra) {
                    ++n_late_swing_extra;
                }
                if (frame.stroke_clamp_hit[leg]) {
                    ++n_l_parked_contacted;
                } else {
                    ++n_stroking_contacted;
                }
            }
            ++phase.census_frames;
            phase.n_raw_contact_sum += static_cast<double>(n_raw_contact);
            phase.n_planned_sum += static_cast<double>(n_planned);
            phase.n_hold_sum += static_cast<double>(n_hold);
            phase.n_late_swing_extra_sum += static_cast<double>(n_late_swing_extra);
            phase.n_l_parked_contacted_sum += static_cast<double>(n_l_parked_contacted);
            phase.n_raw_contact_histogram[std::min(n_raw_contact, phase.n_raw_contact_histogram.size() - 1)] += 1;
            if (n_l_parked_contacted > 0 && n_stroking_contacted > 0) {
                ++phase.mixed_parked_stroking_frames;
            }
            if (n_raw_contact >= 5) {
                ++phase.n_contact_ge_5_frames;
            }
            const bool clean_tripod = isCleanTripodFrame(
                frame.planned_stance, state.foot_contacts, n_l_parked_contacted);
            if (clean_tripod) {
                ++phase.clean_tripod_frames;
            }
            phase.abs_pitch_sum += std::abs(state.body_twist_state.twist_pos_rad.y);
            phase.abs_roll_sum += std::abs(state.body_twist_state.twist_pos_rad.x);
            ++phase.attitude_samples;
            const Vec3 body_step = previous_body_position.has_value()
                ? Vec3{position.x - previous_body_position->x,
                       position.y - previous_body_position->y,
                       0.0}
                : Vec3{};
            const double planar_body_step = std::hypot(body_step.x, body_step.y);
            if (clean_tripod && previous_body_position.has_value()) {
                phase.clean_tripod_body_step_sum += planar_body_step;
                ++phase.clean_tripod_body_samples;
            }
            const Mat3 R_body_to_world = body_pose.rotationBodyToWorld();
            for (std::size_t leg = 0; leg < frame.targets.leg_states.size(); ++leg) {
                const Vec3 target_body = leg_fk.footInBodyFrame(
                    frame.targets.leg_states[leg], geometry.legGeometry[leg])
                    .pos_body_m.raw();
                const Vec3 measured_body = leg_fk.footInBodyFrame(
                    state.leg_states[leg], geometry.legGeometry[leg])
                    .pos_body_m.raw();
                const Vec3 measured_world = leg_fk.footInWorldFrame(
                    state.leg_states[leg], body_pose, geometry.legGeometry[leg])
                    .pos_body_m.raw();
                const double foot_tracking_error = vecNorm(target_body - measured_body);
                phase.max_foot_tracking_error = std::max(
                    phase.max_foot_tracking_error, foot_tracking_error);
                phase.foot_tracking_error_sq_sum += foot_tracking_error * foot_tracking_error;
                ++phase.foot_tracking_error_samples;
                const bool commanded_stance =
                    frame.planned_stance[leg] || frame.hold_stance[leg];
                if (!commanded_stance) {
                    const Vec3 target_world = body_pose.position.raw()
                        + (R_body_to_world * target_body);
                    ++active_segment->swing_frames[leg];
                    active_segment->swing_contact_frames[leg] +=
                        state.foot_contacts[leg] ? 1U : 0U;
                    active_segment->swing_target_world_z_sum[leg] += target_world.z;
                    active_segment->swing_measured_world_z_sum[leg] += measured_world.z;
                }
                const bool onset_stance = isOnsetPlannedStance(
                    frame.planned_stance[leg],
                    previous_planned_stance[leg],
                    frame.gait_phase[leg],
                    frame.duty_factor);
                const bool mid_stance = isMidPlannedStance(
                    frame.planned_stance[leg],
                    frame.gait_phase[leg],
                    frame.duty_factor)
                    && !onset_stance;
                const Vec3 cartesian_body{
                    -frame.planned_target_body[leg].x,
                    frame.planned_target_body[leg].y,
                    frame.planned_target_body[leg].z};
                if (state.foot_contacts[leg] && have_previous_raw_contact[leg]) {
                    const Vec3 measured_delta{
                        measured_world.x - previous_contact_foot_world[leg].x,
                        measured_world.y - previous_contact_foot_world[leg].y,
                        0.0};
                    const Vec3 target_step = target_body - previous_contact_target_body[leg];
                    const Vec3 commanded_delta = body_step + (R_body_to_world * target_step);
                    const double planar_world_step =
                        std::hypot(measured_delta.x, measured_delta.y);
                    const double planar_commanded_world =
                        std::hypot(commanded_delta.x, commanded_delta.y);
                    const double planar_uncommanded = std::hypot(
                        measured_delta.x - commanded_delta.x,
                        measured_delta.y - commanded_delta.y);
                    const double planar_cmd_body = std::hypot(target_step.x, target_step.y);
                    phase.contact_commanded_world_step_sum += planar_commanded_world;
                    phase.contact_uncommanded_slip_step_sum += planar_uncommanded;
                    phase.contact_cmd_body_step_sum += planar_cmd_body;
                    ++phase.contact_slip_samples;
                    if (mid_stance) {
                        phase.midstance_commanded_world_step_sum += planar_commanded_world;
                        phase.midstance_uncommanded_slip_step_sum += planar_uncommanded;
                        phase.midstance_cmd_body_step_sum += planar_cmd_body;
                        ++phase.midstance_slip_samples;
                    }
                    if (clean_tripod) {
                        phase.clean_tripod_commanded_world_step_sum += planar_commanded_world;
                        phase.clean_tripod_uncommanded_slip_step_sum += planar_uncommanded;
                        phase.clean_tripod_contact_world_step_sum += planar_world_step;
                        ++phase.clean_tripod_slip_samples;
                    }
                }
                if (clean_tripod && have_previous_cartesian[leg] && frame.planned_stance[leg]) {
                    const Vec3 cartesian_step =
                        cartesian_body - previous_cartesian_body[leg];
                    if (command_speed > 0.0) {
                        phase.clean_tripod_cartesian_opposition_sum -=
                            (command.vx_mps * cartesian_step.x
                                + command.vy_mps * cartesian_step.y) / command_speed;
                    }
                    if (std::abs(command.yaw_rate_radps) > 0.0) {
                        const double cross =
                            previous_cartesian_body[leg].x * cartesian_body.y
                            - previous_cartesian_body[leg].y * cartesian_body.x;
                        const double dot =
                            previous_cartesian_body[leg].x * cartesian_body.x
                            + previous_cartesian_body[leg].y * cartesian_body.y;
                        phase.clean_tripod_cartesian_counter_yaw_sum -=
                            std::atan2(cross, dot);
                    }
                    ++phase.clean_tripod_cartesian_samples;
                }
                if (state.foot_contacts[leg] && commanded_stance
                    && have_previous_contact_foot[leg]) {
                    const double world_step = vecNorm(
                        measured_world - previous_contact_foot_world[leg]);
                    phase.max_contact_foot_world_step = std::max(
                        phase.max_contact_foot_world_step, world_step);
                    phase.contact_foot_world_step_sq_sum += world_step * world_step;
                    ++phase.contact_foot_world_step_samples;
                    const Vec3 target_step = target_body - previous_contact_target_body[leg];
                    if (command_speed > 0.0) {
                        phase.contact_target_opposition_sum -=
                            (command.vx_mps * target_step.x
                                + command.vy_mps * target_step.y) / command_speed;
                    }
                    if (std::abs(command.yaw_rate_radps) > 0.0) {
                        const double cross = previous_contact_target_body[leg].x * target_body.y
                            - previous_contact_target_body[leg].y * target_body.x;
                        const double dot = previous_contact_target_body[leg].x * target_body.x
                            + previous_contact_target_body[leg].y * target_body.y;
                        phase.contact_target_counter_yaw_sum -= std::atan2(cross, dot);
                    }
                    ++phase.contact_target_motion_samples;
                    if (mid_stance) {
                        const double planar_world_step = std::hypot(
                            measured_world.x - previous_contact_foot_world[leg].x,
                            measured_world.y - previous_contact_foot_world[leg].y);
                        phase.midstance_contact_world_step_sum += planar_world_step;
                        ++phase.midstance_contact_world_step_samples;
                        if (isTripodStanceFrame(n_planned)) {
                            phase.midstance_tripod_contact_world_step_sum +=
                                planar_world_step;
                            ++phase.midstance_tripod_contact_world_step_samples;
                        }
                        if (isOverlapStanceFrame(n_planned)) {
                            phase.midstance_overlap_contact_world_step_sum +=
                                planar_world_step;
                            ++phase.midstance_overlap_contact_world_step_samples;
                        }
                    }
                }
                have_previous_contact_foot[leg] =
                    state.foot_contacts[leg] && commanded_stance;
                have_previous_raw_contact[leg] = state.foot_contacts[leg];
                have_previous_cartesian[leg] = true;
                previous_contact_foot_world[leg] = measured_world;
                previous_contact_target_body[leg] = target_body;
                previous_cartesian_body[leg] = cartesian_body;
            }
            previous_planned_stance = frame.planned_stance;
            previous_body_position = position;
            if (!active_segment->start_position.has_value()) {
                active_segment->start_position = position;
                active_segment->start_yaw = yaw;
                active_segment->last_raw_yaw = yaw;
                active_segment->start_velocity =
                    state.body_twist_state.body_trans_mps.raw();
            } else if (active_segment->last_position.has_value()) {
                active_segment->horizontal_path += std::hypot(
                    position.x - active_segment->last_position->x,
                    position.y - active_segment->last_position->y);
                active_segment->accumulated_yaw += std::remainder(
                    yaw - active_segment->last_raw_yaw,
                    6.28318530717958647692);
                active_segment->last_raw_yaw = yaw;
            }
            active_segment->last_position = position;
            active_segment->last_velocity = state.body_twist_state.body_trans_mps.raw();
            if (frame.phase != ReplayPhase::Stand && frame.phase != ReplayPhase::Transition) {
                if (active_segment->valid_frames == acceleration_transient_frames) {
                    active_segment->evaluation_start_position = position;
                    active_segment->evaluation_last_position = position;
                    active_segment->evaluation_start_yaw = yaw;
                    active_segment->evaluation_last_raw_yaw = yaw;
                } else if (active_segment->evaluation_last_position.has_value()) {
                    active_segment->evaluation_horizontal_path += std::hypot(
                        position.x - active_segment->evaluation_last_position->x,
                        position.y - active_segment->evaluation_last_position->y);
                    active_segment->evaluation_accumulated_yaw += std::remainder(
                        yaw - active_segment->evaluation_last_raw_yaw,
                        6.28318530717958647692);
                    active_segment->evaluation_last_position = position;
                    active_segment->evaluation_last_raw_yaw = yaw;
                }
            }
            ++active_segment->valid_frames;
        }

        const auto telemetry = bridge.latestSolverTelemetry();
        if (!telemetry.has_value()) {
            continue;
        }
        ++result.telemetry_frames;
        const bool topology_changed = previous_contact_set_signature.has_value()
            && *previous_contact_set_signature != telemetry->contact_set_signature;
        contact_topology_age = !previous_contact_set_signature.has_value()
                || topology_changed
            ? 0U
            : std::min<std::uint32_t>(contact_topology_age + 1U, 16U);
        previous_contact_set_signature = telemetry->contact_set_signature;
        if (topology_changed) {
            ++result.topology_changes;
            ++phase.topology_changes;
        }
        constexpr std::uint16_t kIterationAcceptanceTarget = 20;
        if (telemetry->iterations > kIterationAcceptanceTarget) {
            if (topology_changed) {
                ++result.high_iteration_topology_changes;
                ++phase.high_iteration_topology_changes;
            } else {
                ++result.high_iteration_persistent_contacts;
                ++phase.high_iteration_persistent_contacts;
            }
        }
        const std::uint32_t topology_age_bucket = contact_topology_age >= 16U
            ? 16U
            : contact_topology_age >= 8U
                ? 8U
                : contact_topology_age >= 4U
                    ? 4U
                    : contact_topology_age >= 2U ? 2U : contact_topology_age;
        ++result.topology_age_iteration_histograms[topology_age_bucket]
                                                   [telemetry->iterations];
        solver_dynamics_times_ms.push_back(telemetry->dynamics_time_ms);
        solver_contact_setup_times_ms.push_back(telemetry->contact_setup_time_ms);
        solver_collision_times_ms.push_back(telemetry->collision_time_ms);
        solver_constraint_assembly_times_ms.push_back(
            telemetry->constraint_assembly_time_ms);
        solver_delassus_times_ms.push_back(telemetry->delassus_time_ms);
        solver_admm_times_ms.push_back(telemetry->admm_time_ms);
        solver_integration_times_ms.push_back(telemetry->integration_time_ms);
        solver_total_step_times_ms.push_back(telemetry->total_step_time_ms);
        result.max_iterations = std::max(result.max_iterations, telemetry->iterations);
        ++result.iteration_histogram[telemetry->iterations];
        ++result.contact_count_iteration_histograms[telemetry->contact_constraint_count]
                                                   [telemetry->iterations];
        const std::size_t failure_reason = static_cast<std::size_t>(
            telemetry->failure_reason);
        if (failure_reason < result.failure_reason_histogram.size()) {
            ++result.failure_reason_histogram[failure_reason];
        }
        result.max_contact_constraints = std::max(
            result.max_contact_constraints, telemetry->contact_constraint_count);
        result.max_warm_start_resets = std::max(
            result.max_warm_start_resets, telemetry->warm_start_reset_count);
        phase.max_iterations = std::max(phase.max_iterations, telemetry->iterations);
        ++phase.iteration_histogram[telemetry->iterations];
        phase.max_ncp_dual_residual =
            std::max(phase.max_ncp_dual_residual, static_cast<double>(telemetry->ncp_dual_residual));
        phase.max_ncp_complementarity_residual = std::max(
            phase.max_ncp_complementarity_residual,
            static_cast<double>(telemetry->ncp_complementarity_residual));
        phase.max_contact_penetration = std::max(
            phase.max_contact_penetration,
            static_cast<double>(telemetry->max_contact_penetration));
        phase.max_servo_torque_utilization = std::max(
            phase.max_servo_torque_utilization,
            static_cast<double>(telemetry->peak_servo_torque_utilization));
        phase.servo_torque_utilization_sum += telemetry->peak_servo_torque_utilization;
        ++phase.servo_torque_utilization_samples;
        if (telemetry->peak_servo_torque_utilization >= 0.99f) {
            ++phase.servo_saturated_frames;
        }
        phase.peak_actuator_impulse = std::max(
            phase.peak_actuator_impulse,
            static_cast<double>(telemetry->peak_actuator_impulse));
        phase.peak_normal_impulse = std::max(
            phase.peak_normal_impulse,
            static_cast<double>(telemetry->peak_normal_impulse));
        phase.peak_friction_impulse = std::max(
            phase.peak_friction_impulse,
            static_cast<double>(telemetry->peak_friction_impulse));
        phase.peak_normal_impulse_sum += telemetry->peak_normal_impulse;
        phase.peak_friction_impulse_sum += telemetry->peak_friction_impulse;
        ++phase.friction_impulse_samples;
        if (telemetry->peak_normal_impulse > 1.0e-12f) {
            phase.friction_to_normal_ratio_sum +=
                static_cast<double>(telemetry->peak_friction_impulse)
                / static_cast<double>(telemetry->peak_normal_impulse);
            ++phase.friction_ratio_samples;
        }
        phase.peak_preintegration_linear_speed = std::max(
            phase.peak_preintegration_linear_speed,
            static_cast<double>(telemetry->preintegration_linear_speed));
        phase.peak_preintegration_angular_speed = std::max(
            phase.peak_preintegration_angular_speed,
            static_cast<double>(telemetry->preintegration_angular_speed));
        phase.actuator_work_sum += telemetry->actuator_work;
        phase.mechanical_energy_delta_sum += telemetry->mechanical_energy_delta;
        if (telemetry->failure_reason == physics_sim::SolverFailureReason::SolverNotConverged) {
            ++result.solver_not_converged;
            ++phase.solver_not_converged;
        }
        switch (telemetry->status) {
        case physics_sim::SolverStatus::Healthy:
            healthy_step_times_ms.push_back(step_time_ms);
            ++result.healthy;
            ++phase.healthy;
            break;
        case physics_sim::SolverStatus::RecoveredRetry:
            ++result.recovered;
            ++phase.recovered;
            break;
        case physics_sim::SolverStatus::HeldLastGood:
            ++result.held;
            ++phase.held;
            break;
        case physics_sim::SolverStatus::UnsupportedIsland:
            ++result.unsupported;
            ++phase.unsupported;
            break;
        }
    }
    const auto assignTimingSummary = [](std::vector<double>& samples,
                                        double& p99,
                                        double& maximum) {
        if (samples.empty()) {
            return;
        }
        std::sort(samples.begin(), samples.end());
        const std::size_t p99_index = std::min(
            samples.size() - 1,
            static_cast<std::size_t>(
                std::ceil(0.99 * static_cast<double>(samples.size()))) - 1);
        p99 = samples[p99_index];
        maximum = samples.back();
    };
    assignTimingSummary(
        step_times_ms, result.p99_step_time_ms, result.max_step_time_ms);
    assignTimingSummary(healthy_step_times_ms,
                        result.healthy_p99_step_time_ms,
                        result.healthy_max_step_time_ms);
    double ignoredMaximum = 0.0;
    assignTimingSummary(solver_dynamics_times_ms,
                        result.p99_solver_dynamics_time_ms,
                        ignoredMaximum);
    assignTimingSummary(solver_contact_setup_times_ms,
                        result.p99_solver_contact_setup_time_ms,
                        ignoredMaximum);
    assignTimingSummary(solver_collision_times_ms,
                        result.p99_solver_collision_time_ms,
                        ignoredMaximum);
    assignTimingSummary(solver_constraint_assembly_times_ms,
                        result.p99_solver_constraint_assembly_time_ms,
                        ignoredMaximum);
    assignTimingSummary(solver_delassus_times_ms,
                        result.p99_solver_delassus_time_ms,
                        ignoredMaximum);
    assignTimingSummary(solver_admm_times_ms,
                        result.p99_solver_admm_time_ms,
                        ignoredMaximum);
    assignTimingSummary(solver_integration_times_ms,
                        result.p99_solver_integration_time_ms,
                        ignoredMaximum);
    assignTimingSummary(solver_total_step_times_ms,
                        result.p99_solver_total_step_time_ms,
                        ignoredMaximum);
    finishSegment();
    return result;
}

void accumulateReplayResult(ReplayResult& total, const ReplayResult& sample) {
    total.frames += sample.frames;
    total.telemetry_frames += sample.telemetry_frames;
    total.healthy += sample.healthy;
    total.recovered += sample.recovered;
    total.held += sample.held;
    total.unsupported += sample.unsupported;
    total.read_failures += sample.read_failures;
    total.solver_not_converged += sample.solver_not_converged;
    total.topology_changes += sample.topology_changes;
    total.high_iteration_topology_changes +=
        sample.high_iteration_topology_changes;
    total.high_iteration_persistent_contacts +=
        sample.high_iteration_persistent_contacts;
    for (const auto& [iterations, frames] : sample.iteration_histogram) {
        total.iteration_histogram[iterations] += frames;
    }
    for (const auto& [contact_count, histogram] :
         sample.contact_count_iteration_histograms) {
        for (const auto& [iterations, frames] : histogram) {
            total.contact_count_iteration_histograms[contact_count][iterations] += frames;
        }
    }
    for (const auto& [topology_age, histogram] :
         sample.topology_age_iteration_histograms) {
        for (const auto& [iterations, frames] : histogram) {
            total.topology_age_iteration_histograms[topology_age][iterations] += frames;
        }
    }
    for (std::size_t i = 0; i < total.failure_reason_histogram.size(); ++i) {
        total.failure_reason_histogram[i] += sample.failure_reason_histogram[i];
    }
    total.max_iterations = std::max(total.max_iterations, sample.max_iterations);
    total.max_contact_constraints = std::max(
        total.max_contact_constraints, sample.max_contact_constraints);
    total.max_warm_start_resets = std::max(
        total.max_warm_start_resets, sample.max_warm_start_resets);
    const auto accumulateMaximum = [](double& aggregate, const double value) {
        aggregate = std::max(aggregate, value);
    };
    accumulateMaximum(total.p99_step_time_ms, sample.p99_step_time_ms);
    accumulateMaximum(total.max_step_time_ms, sample.max_step_time_ms);
    accumulateMaximum(total.healthy_p99_step_time_ms, sample.healthy_p99_step_time_ms);
    accumulateMaximum(total.healthy_max_step_time_ms, sample.healthy_max_step_time_ms);
    accumulateMaximum(total.p99_solver_dynamics_time_ms, sample.p99_solver_dynamics_time_ms);
    accumulateMaximum(total.p99_solver_contact_setup_time_ms, sample.p99_solver_contact_setup_time_ms);
    accumulateMaximum(total.p99_solver_collision_time_ms, sample.p99_solver_collision_time_ms);
    accumulateMaximum(
        total.p99_solver_constraint_assembly_time_ms,
        sample.p99_solver_constraint_assembly_time_ms);
    accumulateMaximum(total.p99_solver_delassus_time_ms, sample.p99_solver_delassus_time_ms);
    accumulateMaximum(total.p99_solver_admm_time_ms, sample.p99_solver_admm_time_ms);
    accumulateMaximum(
        total.p99_solver_integration_time_ms,
        sample.p99_solver_integration_time_ms);
    accumulateMaximum(
        total.p99_solver_total_step_time_ms,
        sample.p99_solver_total_step_time_ms);
    for (std::size_t i = 0; i < total.phases.size(); ++i) {
        PhaseResult& out = total.phases[i];
        const PhaseResult& in = sample.phases[i];
        out.frames += in.frames;
        out.healthy += in.healthy;
        out.recovered += in.recovered;
        out.held += in.held;
        out.unsupported += in.unsupported;
        out.read_failures += in.read_failures;
        out.solver_not_converged += in.solver_not_converged;
        out.topology_changes += in.topology_changes;
        out.high_iteration_topology_changes +=
            in.high_iteration_topology_changes;
        out.high_iteration_persistent_contacts +=
            in.high_iteration_persistent_contacts;
        for (const auto& [iterations, frames] : in.iteration_histogram) {
            out.iteration_histogram[iterations] += frames;
        }
        out.max_iterations = std::max(out.max_iterations, in.max_iterations);
        out.max_ncp_dual_residual = std::max(
            out.max_ncp_dual_residual, in.max_ncp_dual_residual);
        out.max_ncp_complementarity_residual = std::max(
            out.max_ncp_complementarity_residual,
            in.max_ncp_complementarity_residual);
        out.max_contact_penetration = std::max(
            out.max_contact_penetration, in.max_contact_penetration);
        out.max_body_height_error = std::max(
            out.max_body_height_error, in.max_body_height_error);
        out.max_servo_tracking_error = std::max(
            out.max_servo_tracking_error, in.max_servo_tracking_error);
        out.terminal_servo_tracking_error = std::max(
            out.terminal_servo_tracking_error, in.terminal_servo_tracking_error);
        out.servo_tracking_error_sq_sum += in.servo_tracking_error_sq_sum;
        out.servo_tracking_error_samples += in.servo_tracking_error_samples;
        out.max_servo_torque_utilization = std::max(
            out.max_servo_torque_utilization, in.max_servo_torque_utilization);
        out.servo_torque_utilization_sum += in.servo_torque_utilization_sum;
        out.servo_torque_utilization_samples += in.servo_torque_utilization_samples;
        out.servo_saturated_frames += in.servo_saturated_frames;
        out.peak_actuator_impulse = std::max(
            out.peak_actuator_impulse, in.peak_actuator_impulse);
        out.peak_normal_impulse = std::max(
            out.peak_normal_impulse, in.peak_normal_impulse);
        out.peak_friction_impulse = std::max(
            out.peak_friction_impulse, in.peak_friction_impulse);
        out.peak_preintegration_linear_speed = std::max(
            out.peak_preintegration_linear_speed,
            in.peak_preintegration_linear_speed);
        out.peak_preintegration_angular_speed = std::max(
            out.peak_preintegration_angular_speed,
            in.peak_preintegration_angular_speed);
        out.actuator_work_sum += in.actuator_work_sum;
        out.mechanical_energy_delta_sum += in.mechanical_energy_delta_sum;
        out.max_foot_tracking_error = std::max(
            out.max_foot_tracking_error, in.max_foot_tracking_error);
        out.foot_tracking_error_sq_sum += in.foot_tracking_error_sq_sum;
        out.foot_tracking_error_samples += in.foot_tracking_error_samples;
        out.max_contact_foot_world_step = std::max(
            out.max_contact_foot_world_step, in.max_contact_foot_world_step);
        out.contact_foot_world_step_sq_sum += in.contact_foot_world_step_sq_sum;
        out.contact_foot_world_step_samples += in.contact_foot_world_step_samples;
        out.contact_target_opposition_sum += in.contact_target_opposition_sum;
        out.contact_target_counter_yaw_sum += in.contact_target_counter_yaw_sum;
        out.contact_target_motion_samples += in.contact_target_motion_samples;
        out.midstance_contact_world_step_sum += in.midstance_contact_world_step_sum;
        out.midstance_contact_world_step_samples +=
            in.midstance_contact_world_step_samples;
        out.midstance_tripod_contact_world_step_sum +=
            in.midstance_tripod_contact_world_step_sum;
        out.midstance_tripod_contact_world_step_samples +=
            in.midstance_tripod_contact_world_step_samples;
        out.midstance_overlap_contact_world_step_sum +=
            in.midstance_overlap_contact_world_step_sum;
        out.midstance_overlap_contact_world_step_samples +=
            in.midstance_overlap_contact_world_step_samples;
        out.n_raw_contact_sum += in.n_raw_contact_sum;
        out.n_planned_sum += in.n_planned_sum;
        out.n_hold_sum += in.n_hold_sum;
        out.n_late_swing_extra_sum += in.n_late_swing_extra_sum;
        out.n_l_parked_contacted_sum += in.n_l_parked_contacted_sum;
        out.census_frames += in.census_frames;
        out.mixed_parked_stroking_frames += in.mixed_parked_stroking_frames;
        out.n_contact_ge_5_frames += in.n_contact_ge_5_frames;
        out.clean_tripod_frames += in.clean_tripod_frames;
        for (std::size_t bin = 0; bin < out.n_raw_contact_histogram.size(); ++bin) {
            out.n_raw_contact_histogram[bin] += in.n_raw_contact_histogram[bin];
        }
        out.abs_pitch_sum += in.abs_pitch_sum;
        out.abs_roll_sum += in.abs_roll_sum;
        out.attitude_samples += in.attitude_samples;
        out.peak_normal_impulse_sum += in.peak_normal_impulse_sum;
        out.peak_friction_impulse_sum += in.peak_friction_impulse_sum;
        out.friction_to_normal_ratio_sum += in.friction_to_normal_ratio_sum;
        out.friction_impulse_samples += in.friction_impulse_samples;
        out.friction_ratio_samples += in.friction_ratio_samples;
        out.contact_commanded_world_step_sum += in.contact_commanded_world_step_sum;
        out.contact_uncommanded_slip_step_sum += in.contact_uncommanded_slip_step_sum;
        out.contact_cmd_body_step_sum += in.contact_cmd_body_step_sum;
        out.contact_slip_samples += in.contact_slip_samples;
        out.midstance_commanded_world_step_sum += in.midstance_commanded_world_step_sum;
        out.midstance_uncommanded_slip_step_sum += in.midstance_uncommanded_slip_step_sum;
        out.midstance_cmd_body_step_sum += in.midstance_cmd_body_step_sum;
        out.midstance_slip_samples += in.midstance_slip_samples;
        out.clean_tripod_body_step_sum += in.clean_tripod_body_step_sum;
        out.clean_tripod_body_samples += in.clean_tripod_body_samples;
        out.clean_tripod_cartesian_opposition_sum +=
            in.clean_tripod_cartesian_opposition_sum;
        out.clean_tripod_cartesian_counter_yaw_sum +=
            in.clean_tripod_cartesian_counter_yaw_sum;
        out.clean_tripod_cartesian_samples += in.clean_tripod_cartesian_samples;
        out.clean_tripod_commanded_world_step_sum +=
            in.clean_tripod_commanded_world_step_sum;
        out.clean_tripod_uncommanded_slip_step_sum +=
            in.clean_tripod_uncommanded_slip_step_sum;
        out.clean_tripod_contact_world_step_sum +=
            in.clean_tripod_contact_world_step_sum;
        out.clean_tripod_slip_samples += in.clean_tripod_slip_samples;
        out.completed_delta_x_sum += in.completed_delta_x_sum;
        out.completed_delta_y_sum += in.completed_delta_y_sum;
        out.completed_body_forward_sum += in.completed_body_forward_sum;
        out.completed_body_lateral_sum += in.completed_body_lateral_sum;
        out.completed_yaw_delta_sum += in.completed_yaw_delta_sum;
        out.completed_horizontal_path_sum += in.completed_horizontal_path_sum;
        out.completed_horizontal_displacement_sum +=
            in.completed_horizontal_displacement_sum;
        out.completed_start_body_forward_velocity_sum +=
            in.completed_start_body_forward_velocity_sum;
        out.completed_start_body_lateral_velocity_sum +=
            in.completed_start_body_lateral_velocity_sum;
        out.completed_end_body_forward_velocity_sum +=
            in.completed_end_body_forward_velocity_sum;
        out.completed_end_body_lateral_velocity_sum +=
            in.completed_end_body_lateral_velocity_sum;
        out.evaluated_body_forward_sum += in.evaluated_body_forward_sum;
        out.evaluated_body_lateral_sum += in.evaluated_body_lateral_sum;
        out.evaluated_yaw_delta_sum += in.evaluated_yaw_delta_sum;
        out.evaluated_horizontal_path_sum += in.evaluated_horizontal_path_sum;
        out.evaluated_horizontal_displacement_sum +=
            in.evaluated_horizontal_displacement_sum;
        out.evaluated_frames += in.evaluated_frames;
        out.evaluated_trajectories += in.evaluated_trajectories;
        out.completed_trajectories += in.completed_trajectories;
    }
}

std::string metricsJson(const ReplayResult& result,
                        const std::uint64_t command_hash,
                        const std::vector<CapturedFrame>& captured_frames,
                        const bool command_fixture_loaded,
                        const bool command_fixture_written,
                        const int perturbation_seed_count,
                        const int perturbation_seed_offset,
                        const double perturbation_scale,
                        const bool fixed_initial_pose,
                        const bool fixed_contact_order,
                        const bool dense_admm,
                        const bool contact_precondition,
                        const physics_sim::PhysicsSolverMode capture_solver_mode,
                        const int capture_solver_iterations,
                        const bool behavior_gates_requested,
                        const std::uint64_t behavior_gate_failures,
                        const int replay_period_us,
                        const int capture_period_us,
                        const int solver_iterations,
                        const double body_height_m,
                        const double proximal_mu,
                        const double contact_regularization,
                        const double absolute_tolerance,
                        const double relative_tolerance) {
    std::ostringstream out;
    struct CapturedPhaseMetrics {
        std::array<double, 18> minimum{};
        std::array<double, 18> maximum{};
        std::array<double, 18> previous{};
        bool initialized{false};
        std::uint64_t inhibited_frames{0};
        std::uint64_t walk_mode_frames{0};
        std::uint64_t moving_target_frames{0};
        double max_target_step{0.0};
        double max_target_span{0.0};
        std::array<Vec3, kNumLegs> previous_target_body{};
        std::array<bool, kNumLegs> have_previous_target{};
        std::array<bool, kNumLegs> previous_planned{};
        double stance_target_opposition_sum{0.0};
        double stance_target_counter_yaw_sum{0.0};
        std::uint64_t stance_target_motion_samples{0};
        double midstance_opposition_sum{0.0};
        double midstance_counter_yaw_sum{0.0};
        std::uint64_t midstance_motion_samples{0};
        double midstance_tripod_opposition_sum{0.0};
        double midstance_tripod_counter_yaw_sum{0.0};
        std::uint64_t midstance_tripod_motion_samples{0};
        double midstance_overlap_opposition_sum{0.0};
        double midstance_overlap_counter_yaw_sum{0.0};
        std::uint64_t midstance_overlap_motion_samples{0};
        double midstance_high_duty_opposition_sum{0.0};
        double midstance_high_duty_counter_yaw_sum{0.0};
        std::uint64_t midstance_high_duty_motion_samples{0};
        std::array<Vec3, kNumLegs> previous_cartesian_body{};
        std::array<bool, kNumLegs> have_previous_cartesian{};
        double midstance_cartesian_opposition_sum{0.0};
        std::uint64_t midstance_cartesian_motion_samples{0};
        double midstance_tripod_cartesian_opposition_sum{0.0};
        std::uint64_t midstance_tripod_cartesian_motion_samples{0};
        double midstance_cartesian_counter_yaw_sum{0.0};
        std::uint64_t midstance_cartesian_yaw_samples{0};
        double midstance_tripod_cartesian_counter_yaw_sum{0.0};
        std::uint64_t midstance_tripod_cartesian_yaw_samples{0};
        std::array<Vec3, kNumLegs> previous_ik_body{};
        std::array<bool, kNumLegs> have_previous_ik{};
        double midstance_ik_opposition_sum{0.0};
        std::uint64_t midstance_ik_motion_samples{0};
        double midstance_tripod_ik_opposition_sum{0.0};
        std::uint64_t midstance_tripod_ik_motion_samples{0};
        double midstance_ik_counter_yaw_sum{0.0};
        std::uint64_t midstance_ik_yaw_samples{0};
        double midstance_tripod_ik_counter_yaw_sum{0.0};
        std::uint64_t midstance_tripod_ik_yaw_samples{0};
        std::array<Vec3, kNumLegs> previous_aligned_fk_body{};
        std::array<bool, kNumLegs> have_previous_aligned_fk{};
        double midstance_aligned_fk_opposition_sum{0.0};
        std::uint64_t midstance_aligned_fk_motion_samples{0};
        double midstance_tripod_aligned_fk_opposition_sum{0.0};
        std::uint64_t midstance_tripod_aligned_fk_motion_samples{0};
        double midstance_aligned_fk_counter_yaw_sum{0.0};
        std::uint64_t midstance_aligned_fk_yaw_samples{0};
        double midstance_tripod_aligned_fk_counter_yaw_sum{0.0};
        std::uint64_t midstance_tripod_aligned_fk_yaw_samples{0};
        double midstance_stride_hz_sum{0.0};
        double midstance_command_scale_sum{0.0};
        double midstance_cadence_scale_sum{0.0};
        double midstance_governed_command_speed_sum{0.0};
        std::uint64_t midstance_stroke_clamp_hit_samples{0};
        std::uint64_t midstance_stroke_clamp_samples{0};
        std::uint64_t midstance_workspace_xy_hit_samples{0};
        double midstance_tripod_cartesian_plant_hit_opposition_sum{0.0};
        std::uint64_t midstance_tripod_cartesian_plant_hit_samples{0};
        double midstance_tripod_cartesian_plant_miss_opposition_sum{0.0};
        std::uint64_t midstance_tripod_cartesian_plant_miss_samples{0};
        double midstance_tripod_cartesian_workspace_xy_hit_opposition_sum{0.0};
        std::uint64_t midstance_tripod_cartesian_workspace_xy_hit_samples{0};
        double midstance_tripod_cartesian_workspace_xy_miss_opposition_sum{0.0};
        std::uint64_t midstance_tripod_cartesian_workspace_xy_miss_samples{0};
        double midstance_tripod_cartesian_neither_hit_opposition_sum{0.0};
        std::uint64_t midstance_tripod_cartesian_neither_hit_samples{0};
        std::uint64_t midstance_ik_reach_hit_samples{0};
        std::uint64_t midstance_slew_hit_samples{0};
        double midstance_post_clamp_distortion_sum{0.0};
        double duty_factor_sum{0.0};
        std::uint64_t annotated_frames{0};
        std::uint64_t high_duty_frames{0};
        std::uint64_t tripod_frames{0};
        std::uint64_t overlap_frames{0};
        std::uint64_t planned_stance_samples{0};
        std::uint64_t held_stance_samples{0};
        std::uint64_t held_swing_samples{0};
        std::uint64_t reverse_target_steps{0};
        std::uint64_t reverse_planned_stance_steps{0};
        std::uint64_t reverse_held_swing_steps{0};
        std::uint64_t reverse_held_stance_steps{0};
        std::uint64_t reverse_unclassified_steps{0};
        std::uint64_t onset_reverse_steps{0};
        std::uint64_t midstance_reverse_steps{0};
        std::uint64_t midstance_reverse_f_increase_steps{0};
        std::uint64_t midstance_reverse_phase_drop_steps{0};
        std::uint64_t midstance_reverse_other_steps{0};
        double previous_stride_phase_rate_hz{0.0};
        bool have_previous_stride{false};
        std::array<double, kNumLegs> previous_gait_phase{};
        std::uint64_t safe_to_lift_samples{0};
        double minimum_liftoff_clearance_m{std::numeric_limits<double>::infinity()};
        double maximum_liftoff_clearance_m{-std::numeric_limits<double>::infinity()};
        double minimum_static_margin_m{std::numeric_limits<double>::infinity()};
        double maximum_static_margin_m{-std::numeric_limits<double>::infinity()};
    };
    std::array<CapturedPhaseMetrics, kPhaseCount> captured_phase_metrics{};
    const HexapodGeometry captured_geometry = defaultHexapodGeometry();
    LegFK captured_leg_fk{};
    for (const CapturedFrame& frame : captured_frames) {
        CapturedPhaseMetrics& metrics =
            captured_phase_metrics[static_cast<std::size_t>(frame.phase)];
        metrics.inhibited_frames += frame.inhibit_motion ? 1U : 0U;
        metrics.walk_mode_frames += frame.walk_mode ? 1U : 0U;
        const PhaseCommand command = phaseCommand(frame.phase);
        const double command_speed = std::hypot(command.vx_mps, command.vy_mps);
        metrics.minimum_static_margin_m = std::min(
            metrics.minimum_static_margin_m, frame.static_stability_margin_m);
        metrics.maximum_static_margin_m = std::max(
            metrics.maximum_static_margin_m, frame.static_stability_margin_m);
        ++metrics.annotated_frames;
        metrics.duty_factor_sum += frame.duty_factor;
        if (isHighDuty(frame.duty_factor)) {
            ++metrics.high_duty_frames;
        }
        const std::size_t n_planned = plannedStanceCount(frame.planned_stance);
        if (isTripodStanceFrame(n_planned)) {
            ++metrics.tripod_frames;
        }
        if (isOverlapStanceFrame(n_planned)) {
            ++metrics.overlap_frames;
        }
        const bool high_duty = isHighDuty(frame.duty_factor);
        for (std::size_t leg = 0; leg < frame.targets.leg_states.size(); ++leg) {
            metrics.planned_stance_samples += frame.planned_stance[leg] ? 1U : 0U;
            metrics.held_stance_samples += frame.hold_stance[leg] ? 1U : 0U;
            metrics.held_swing_samples +=
                frame.hold_stance[leg] && !frame.planned_stance[leg] ? 1U : 0U;
            metrics.safe_to_lift_samples += frame.safe_to_lift[leg] ? 1U : 0U;
            metrics.minimum_liftoff_clearance_m = std::min(
                metrics.minimum_liftoff_clearance_m,
                frame.liftoff_clearance_m[leg]);
            metrics.maximum_liftoff_clearance_m = std::max(
                metrics.maximum_liftoff_clearance_m,
                frame.liftoff_clearance_m[leg]);
            const bool planned = frame.planned_stance[leg];
            const bool held = frame.hold_stance[leg];
            const bool stance = planned || held;
            const bool onset = isOnsetPlannedStance(
                planned,
                metrics.previous_planned[leg],
                frame.gait_phase[leg],
                frame.duty_factor);
            const bool mid_stance = isMidPlannedStance(
                planned, frame.gait_phase[leg], frame.duty_factor)
                && !onset;
            const Vec3 legacy_target = captured_leg_fk.footInBodyFrame(
                frame.targets.leg_states[leg], captured_geometry.legGeometry[leg])
                .pos_body_m.raw();
            const Vec3 target_body{-legacy_target.x, legacy_target.y, legacy_target.z};
            if (metrics.have_previous_target[leg]) {
                const Vec3 target_step = target_body - metrics.previous_target_body[leg];
                const double planar_step = std::hypot(target_step.x, target_step.y);
                double opposition_step = 0.0;
                double counter_yaw_step = 0.0;
                if (command_speed > 0.0) {
                    opposition_step = -(command.vx_mps * target_step.x
                        + command.vy_mps * target_step.y) / command_speed;
                }
                if (std::abs(command.yaw_rate_radps) > 0.0) {
                    const Vec3& previous = metrics.previous_target_body[leg];
                    counter_yaw_step = -std::atan2(
                        previous.x * target_body.y - previous.y * target_body.x,
                        previous.x * target_body.x + previous.y * target_body.y);
                }
                if (stance) {
                    metrics.stance_target_opposition_sum += opposition_step;
                    metrics.stance_target_counter_yaw_sum += counter_yaw_step;
                    ++metrics.stance_target_motion_samples;
                }
                if (mid_stance) {
                    metrics.midstance_opposition_sum += opposition_step;
                    metrics.midstance_counter_yaw_sum += counter_yaw_step;
                    ++metrics.midstance_motion_samples;
                    if (isTripodStanceFrame(n_planned)) {
                        metrics.midstance_tripod_opposition_sum += opposition_step;
                        metrics.midstance_tripod_counter_yaw_sum += counter_yaw_step;
                        ++metrics.midstance_tripod_motion_samples;
                    }
                    if (isOverlapStanceFrame(n_planned)) {
                        metrics.midstance_overlap_opposition_sum += opposition_step;
                        metrics.midstance_overlap_counter_yaw_sum += counter_yaw_step;
                        ++metrics.midstance_overlap_motion_samples;
                    }
                    if (high_duty) {
                        metrics.midstance_high_duty_opposition_sum += opposition_step;
                        metrics.midstance_high_duty_counter_yaw_sum += counter_yaw_step;
                        ++metrics.midstance_high_duty_motion_samples;
                    }
                }
                if (planar_step > 0.005) {
                    bool reverse = false;
                    if (command_speed > 0.0) {
                        reverse = (command.vx_mps * target_step.x
                            + command.vy_mps * target_step.y) / command_speed > 0.0;
                    } else if (std::abs(command.yaw_rate_radps) > 0.0) {
                        reverse = counter_yaw_step * command.yaw_rate_radps < 0.0;
                    }
                    if (reverse) {
                        ++metrics.reverse_target_steps;
                        if (planned) {
                            ++metrics.reverse_planned_stance_steps;
                        } else if (held) {
                            ++metrics.reverse_held_swing_steps;
                        } else {
                            ++metrics.reverse_unclassified_steps;
                        }
                        if (held && planned) {
                            ++metrics.reverse_held_stance_steps;
                        }
                        if (onset) {
                            ++metrics.onset_reverse_steps;
                        } else if (mid_stance) {
                            ++metrics.midstance_reverse_steps;
                            const bool f_increase = metrics.have_previous_stride
                                && metrics.previous_stride_phase_rate_hz > 1e-9
                                && (frame.stride_phase_rate_hz
                                        - metrics.previous_stride_phase_rate_hz)
                                        / metrics.previous_stride_phase_rate_hz
                                    > 0.05;
                            const bool phase_drop = metrics.previous_planned[leg]
                                && frame.gait_phase[leg] < metrics.previous_gait_phase[leg];
                            if (f_increase) {
                                ++metrics.midstance_reverse_f_increase_steps;
                            } else if (phase_drop) {
                                ++metrics.midstance_reverse_phase_drop_steps;
                            } else {
                                ++metrics.midstance_reverse_other_steps;
                            }
                        }
                    }
                }
            }
            metrics.have_previous_target[leg] = true;
            metrics.previous_target_body[leg] = target_body;
            metrics.previous_planned[leg] = planned;
            metrics.previous_gait_phase[leg] = frame.gait_phase[leg];
            const Vec3 cartesian_body{
                -frame.planned_target_body[leg].x,
                frame.planned_target_body[leg].y,
                frame.planned_target_body[leg].z};
            const Vec3 ik_body{
                -frame.pre_slew_fk_body[leg].x,
                frame.pre_slew_fk_body[leg].y,
                frame.pre_slew_fk_body[leg].z};
            const Vec3 aligned_fk_body{
                -frame.post_clamp_fk_body[leg].x,
                frame.post_clamp_fk_body[leg].y,
                frame.post_clamp_fk_body[leg].z};
            const auto oppositionStep = [&](const Vec3& step) {
                if (command_speed <= 0.0) {
                    return 0.0;
                }
                return -(command.vx_mps * step.x + command.vy_mps * step.y) / command_speed;
            };
            const auto counterYawStep = [&](const Vec3& previous, const Vec3& current) {
                if (std::abs(command.yaw_rate_radps) <= 0.0) {
                    return 0.0;
                }
                return -std::atan2(
                    previous.x * current.y - previous.y * current.x,
                    previous.x * current.x + previous.y * current.y);
            };
            if (mid_stance) {
                metrics.midstance_stride_hz_sum += frame.stride_phase_rate_hz;
                metrics.midstance_command_scale_sum += frame.command_scale;
                metrics.midstance_cadence_scale_sum += frame.cadence_scale;
                metrics.midstance_governed_command_speed_sum +=
                    command_speed * frame.command_scale;
                ++metrics.midstance_stroke_clamp_samples;
                if (frame.stroke_clamp_hit[leg]) {
                    ++metrics.midstance_stroke_clamp_hit_samples;
                }
                if (frame.workspace_xy_hit[leg]) {
                    ++metrics.midstance_workspace_xy_hit_samples;
                }
                if (frame.ik_reach_clamp_hit[leg]) {
                    ++metrics.midstance_ik_reach_hit_samples;
                }
                if (frame.slew_clamp_hit[leg]) {
                    ++metrics.midstance_slew_hit_samples;
                }
                metrics.midstance_post_clamp_distortion_sum +=
                    frame.post_clamp_distortion_m[leg];
            }
            if (metrics.have_previous_cartesian[leg] && mid_stance) {
                const Vec3 cartesian_step =
                    cartesian_body - metrics.previous_cartesian_body[leg];
                const double cartesian_opposition_step = oppositionStep(cartesian_step);
                const double cartesian_yaw_step =
                    counterYawStep(metrics.previous_cartesian_body[leg], cartesian_body);
                metrics.midstance_cartesian_opposition_sum += cartesian_opposition_step;
                ++metrics.midstance_cartesian_motion_samples;
                metrics.midstance_cartesian_counter_yaw_sum += cartesian_yaw_step;
                ++metrics.midstance_cartesian_yaw_samples;
                if (isTripodStanceFrame(n_planned)) {
                    metrics.midstance_tripod_cartesian_opposition_sum +=
                        cartesian_opposition_step;
                    ++metrics.midstance_tripod_cartesian_motion_samples;
                    metrics.midstance_tripod_cartesian_counter_yaw_sum += cartesian_yaw_step;
                    ++metrics.midstance_tripod_cartesian_yaw_samples;
                    if (frame.stroke_clamp_hit[leg]) {
                        metrics.midstance_tripod_cartesian_plant_hit_opposition_sum +=
                            cartesian_opposition_step;
                        ++metrics.midstance_tripod_cartesian_plant_hit_samples;
                    } else {
                        metrics.midstance_tripod_cartesian_plant_miss_opposition_sum +=
                            cartesian_opposition_step;
                        ++metrics.midstance_tripod_cartesian_plant_miss_samples;
                    }
                    if (frame.workspace_xy_hit[leg]) {
                        metrics.midstance_tripod_cartesian_workspace_xy_hit_opposition_sum +=
                            cartesian_opposition_step;
                        ++metrics.midstance_tripod_cartesian_workspace_xy_hit_samples;
                    } else {
                        metrics.midstance_tripod_cartesian_workspace_xy_miss_opposition_sum +=
                            cartesian_opposition_step;
                        ++metrics.midstance_tripod_cartesian_workspace_xy_miss_samples;
                    }
                    if (!frame.stroke_clamp_hit[leg] && !frame.workspace_xy_hit[leg]) {
                        metrics.midstance_tripod_cartesian_neither_hit_opposition_sum +=
                            cartesian_opposition_step;
                        ++metrics.midstance_tripod_cartesian_neither_hit_samples;
                    }
                }
            }
            if (metrics.have_previous_ik[leg] && mid_stance) {
                const Vec3 ik_step = ik_body - metrics.previous_ik_body[leg];
                const double ik_opposition_step = oppositionStep(ik_step);
                const double ik_yaw_step =
                    counterYawStep(metrics.previous_ik_body[leg], ik_body);
                metrics.midstance_ik_opposition_sum += ik_opposition_step;
                ++metrics.midstance_ik_motion_samples;
                metrics.midstance_ik_counter_yaw_sum += ik_yaw_step;
                ++metrics.midstance_ik_yaw_samples;
                if (isTripodStanceFrame(n_planned)) {
                    metrics.midstance_tripod_ik_opposition_sum += ik_opposition_step;
                    ++metrics.midstance_tripod_ik_motion_samples;
                    metrics.midstance_tripod_ik_counter_yaw_sum += ik_yaw_step;
                    ++metrics.midstance_tripod_ik_yaw_samples;
                }
            }
            if (metrics.have_previous_aligned_fk[leg] && mid_stance) {
                const Vec3 aligned_step =
                    aligned_fk_body - metrics.previous_aligned_fk_body[leg];
                const double aligned_opposition_step = oppositionStep(aligned_step);
                const double aligned_yaw_step =
                    counterYawStep(metrics.previous_aligned_fk_body[leg], aligned_fk_body);
                metrics.midstance_aligned_fk_opposition_sum += aligned_opposition_step;
                ++metrics.midstance_aligned_fk_motion_samples;
                metrics.midstance_aligned_fk_counter_yaw_sum += aligned_yaw_step;
                ++metrics.midstance_aligned_fk_yaw_samples;
                if (isTripodStanceFrame(n_planned)) {
                    metrics.midstance_tripod_aligned_fk_opposition_sum +=
                        aligned_opposition_step;
                    ++metrics.midstance_tripod_aligned_fk_motion_samples;
                    metrics.midstance_tripod_aligned_fk_counter_yaw_sum += aligned_yaw_step;
                    ++metrics.midstance_tripod_aligned_fk_yaw_samples;
                }
            }
            metrics.have_previous_cartesian[leg] = true;
            metrics.previous_cartesian_body[leg] = cartesian_body;
            metrics.have_previous_ik[leg] = true;
            metrics.previous_ik_body[leg] = ik_body;
            metrics.have_previous_aligned_fk[leg] = true;
            metrics.previous_aligned_fk_body[leg] = aligned_fk_body;
        }
        metrics.previous_stride_phase_rate_hz = frame.stride_phase_rate_hz;
        metrics.have_previous_stride = true;
        std::array<double, 18> current{};
        std::size_t joint_index = 0;
        for (const LegState& leg : frame.targets.leg_states) {
            for (const JointState& joint : leg.joint_state) {
                current[joint_index++] = joint.pos_rad.value;
            }
        }
        if (!metrics.initialized) {
            metrics.minimum = current;
            metrics.maximum = current;
            metrics.previous = current;
            metrics.initialized = true;
            continue;
        }
        double frame_max_step = 0.0;
        for (std::size_t i = 0; i < current.size(); ++i) {
            metrics.minimum[i] = std::min(metrics.minimum[i], current[i]);
            metrics.maximum[i] = std::max(metrics.maximum[i], current[i]);
            frame_max_step = std::max(
                frame_max_step, std::abs(current[i] - metrics.previous[i]));
            metrics.previous[i] = current[i];
        }
        metrics.max_target_step = std::max(metrics.max_target_step, frame_max_step);
        metrics.moving_target_frames += frame_max_step > 1.0e-6 ? 1U : 0U;
    }
    for (CapturedPhaseMetrics& metrics : captured_phase_metrics) {
        if (!metrics.initialized) {
            continue;
        }
        for (std::size_t i = 0; i < metrics.minimum.size(); ++i) {
            metrics.max_target_span = std::max(
                metrics.max_target_span, metrics.maximum[i] - metrics.minimum[i]);
        }
    }
    out << std::setprecision(9)
        << "{\"captured_frames\":" << captured_frames.size()
        << ",\"replayed_frames\":" << result.frames
        << ",\"telemetry_frames\":" << result.telemetry_frames
        << ",\"command_hash\":\"" << std::hex << command_hash << std::dec << "\""
        << ",\"command_fixture_loaded\":"
        << (command_fixture_loaded ? "true" : "false")
        << ",\"command_fixture_written\":"
        << (command_fixture_written ? "true" : "false")
        << ",\"perturbation_seed_count\":" << perturbation_seed_count
        << ",\"perturbation_seed_offset\":" << perturbation_seed_offset
        << ",\"perturbation_scale\":" << perturbation_scale
        << ",\"fixed_initial_pose\":" << (fixed_initial_pose ? "true" : "false")
        << ",\"fixed_contact_order\":" << (fixed_contact_order ? "true" : "false")
        << ",\"dense_admm\":" << (dense_admm ? "true" : "false")
        << ",\"contact_precondition\":"
        << (contact_precondition ? "true" : "false")
        << ",\"capture_solver_mode\":\""
        << (capture_solver_mode == physics_sim::PhysicsSolverMode::LegacyPgs
                ? "legacy-pgs"
                : "pinocchio-proximal")
        << "\""
        << ",\"capture_solver_iteration_limit\":" << capture_solver_iterations
        << ",\"behavior_gates_requested\":"
        << (behavior_gates_requested ? "true" : "false")
        << ",\"behavior_gate_failures\":" << behavior_gate_failures
        << ",\"replay_period_us\":" << replay_period_us
        << ",\"capture_period_us\":" << capture_period_us
        << ",\"command_score_period_us\":"
        << commandScorePeriodUs(replay_period_us, capture_period_us)
        << ",\"acceleration_transient_frames\":"
        << std::max<std::uint64_t>(1, static_cast<std::uint64_t>(std::ceil(
            kAccelerationTransientDurationS
            / (static_cast<double>(replay_period_us) * 1.0e-6))))
        << ",\"acceleration_transient_s\":" << kAccelerationTransientDurationS
        << ",\"solver_iteration_limit\":" << solver_iterations
        << ",\"commanded_body_height_m\":" << body_height_m
        << ",\"proximal_mu\":" << proximal_mu
        << ",\"contact_regularization\":" << contact_regularization
        << ",\"absolute_tolerance\":" << absolute_tolerance
        << ",\"relative_tolerance\":" << relative_tolerance
        << ",\"healthy\":" << result.healthy
        << ",\"recovered\":" << result.recovered
        << ",\"held\":" << result.held
        << ",\"unsupported\":" << result.unsupported
        << ",\"read_failures\":" << result.read_failures
        << ",\"solver_not_converged\":" << result.solver_not_converged
        << ",\"topology_changes\":" << result.topology_changes
        << ",\"high_iteration_topology_changes\":"
        << result.high_iteration_topology_changes
        << ",\"high_iteration_persistent_contacts\":"
        << result.high_iteration_persistent_contacts
        << ",\"max_iterations\":" << result.max_iterations
        << ",\"max_contact_constraints\":" << result.max_contact_constraints
        << ",\"max_warm_start_resets\":" << result.max_warm_start_resets
        << ",\"p99_step_time_ms\":" << result.p99_step_time_ms
        << ",\"max_step_time_ms\":" << result.max_step_time_ms
        << ",\"healthy_p99_step_time_ms\":" << result.healthy_p99_step_time_ms
        << ",\"healthy_max_step_time_ms\":" << result.healthy_max_step_time_ms
        << ",\"p99_solver_dynamics_time_ms\":" << result.p99_solver_dynamics_time_ms
        << ",\"p99_solver_contact_setup_time_ms\":"
        << result.p99_solver_contact_setup_time_ms
        << ",\"p99_solver_collision_time_ms\":" << result.p99_solver_collision_time_ms
        << ",\"p99_solver_constraint_assembly_time_ms\":"
        << result.p99_solver_constraint_assembly_time_ms
        << ",\"p99_solver_delassus_time_ms\":" << result.p99_solver_delassus_time_ms
        << ",\"p99_solver_admm_time_ms\":" << result.p99_solver_admm_time_ms
        << ",\"p99_solver_integration_time_ms\":"
        << result.p99_solver_integration_time_ms
        << ",\"p99_solver_total_step_time_ms\":"
        << result.p99_solver_total_step_time_ms
        << ",\"iteration_histogram\":[";
    bool first_iteration_bin = true;
    for (const auto& [iterations, frames] : result.iteration_histogram) {
        if (!first_iteration_bin) {
            out << ',';
        }
        first_iteration_bin = false;
        out << "{\"iterations\":" << iterations << ",\"frames\":" << frames << '}';
    }
    out << "],\"contact_count_iteration_profiles\":[";
    bool first_contact_profile = true;
    for (const auto& [contact_count, histogram] :
         result.contact_count_iteration_histograms) {
        if (!first_contact_profile) {
            out << ',';
        }
        first_contact_profile = false;
        std::uint64_t frame_count = 0;
        for (const auto& [iterations, frames] : histogram) {
            (void)iterations;
            frame_count += frames;
        }
        out << "{\"contacts\":" << contact_count
            << ",\"frames\":" << frame_count
            << ",\"p50_iterations\":" << iterationPercentile(histogram, 0.50)
            << ",\"p90_iterations\":" << iterationPercentile(histogram, 0.90)
            << ",\"p99_iterations\":" << iterationPercentile(histogram, 0.99)
            << ",\"max_iterations\":" << histogram.rbegin()->first << '}';
    }
    out << "],\"topology_age_iteration_profiles\":[";
    bool first_topology_age_profile = true;
    for (const auto& [minimum_age, histogram] :
         result.topology_age_iteration_histograms) {
        if (!first_topology_age_profile) {
            out << ',';
        }
        first_topology_age_profile = false;
        std::uint64_t frame_count = 0;
        for (const auto& [iterations, frames] : histogram) {
            (void)iterations;
            frame_count += frames;
        }
        out << "{\"minimum_age_frames\":" << minimum_age
            << ",\"frames\":" << frame_count
            << ",\"p50_iterations\":" << iterationPercentile(histogram, 0.50)
            << ",\"p90_iterations\":" << iterationPercentile(histogram, 0.90)
            << ",\"p99_iterations\":" << iterationPercentile(histogram, 0.99)
            << ",\"max_iterations\":" << histogram.rbegin()->first << '}';
    }
    out << "],\"failure_reason_histogram\":[";
    bool first_failure_reason = true;
    for (std::size_t i = 1; i < result.failure_reason_histogram.size(); ++i) {
        if (result.failure_reason_histogram[i] == 0) {
            continue;
        }
        if (!first_failure_reason) {
            out << ',';
        }
        first_failure_reason = false;
        out << "{\"reason\":\"" << kFailureReasonNames[i]
            << "\",\"frames\":" << result.failure_reason_histogram[i] << '}';
    }
    out << ']'
        << ",\"phases\":[";
    for (std::size_t i = 0; i < result.phases.size(); ++i) {
        if (i != 0) {
            out << ',';
        }
        const PhaseResult& phase = result.phases[i];
        const CapturedPhaseMetrics& captured = captured_phase_metrics[i];
        const double dx = phase.completed_trajectories == 0 ? 0.0
            : phase.completed_delta_x_sum
                / static_cast<double>(phase.completed_trajectories);
        const double dy = phase.completed_trajectories == 0 ? 0.0
            : phase.completed_delta_y_sum
                / static_cast<double>(phase.completed_trajectories);
        const double body_forward = phase.completed_trajectories == 0 ? 0.0
            : phase.completed_body_forward_sum
                / static_cast<double>(phase.completed_trajectories);
        const double body_lateral = phase.completed_trajectories == 0 ? 0.0
            : phase.completed_body_lateral_sum
                / static_cast<double>(phase.completed_trajectories);
        const double yaw_delta = phase.completed_trajectories == 0 ? 0.0
            : phase.completed_yaw_delta_sum
                / static_cast<double>(phase.completed_trajectories);
        const double horizontal_path = phase.completed_trajectories == 0 ? 0.0
            : phase.completed_horizontal_path_sum
                / static_cast<double>(phase.completed_trajectories);
        const double horizontal_displacement = phase.completed_trajectories == 0 ? 0.0
            : phase.completed_horizontal_displacement_sum
                / static_cast<double>(phase.completed_trajectories);
        const double start_body_forward_velocity = phase.completed_trajectories == 0 ? 0.0
            : phase.completed_start_body_forward_velocity_sum
                / static_cast<double>(phase.completed_trajectories);
        const double start_body_lateral_velocity = phase.completed_trajectories == 0 ? 0.0
            : phase.completed_start_body_lateral_velocity_sum
                / static_cast<double>(phase.completed_trajectories);
        const double end_body_forward_velocity = phase.completed_trajectories == 0 ? 0.0
            : phase.completed_end_body_forward_velocity_sum
                / static_cast<double>(phase.completed_trajectories);
        const double end_body_lateral_velocity = phase.completed_trajectories == 0 ? 0.0
            : phase.completed_end_body_lateral_velocity_sum
                / static_cast<double>(phase.completed_trajectories);
        const double evaluated_body_forward = phase.evaluated_trajectories == 0 ? 0.0
            : phase.evaluated_body_forward_sum
                / static_cast<double>(phase.evaluated_trajectories);
        const double evaluated_body_lateral = phase.evaluated_trajectories == 0 ? 0.0
            : phase.evaluated_body_lateral_sum
                / static_cast<double>(phase.evaluated_trajectories);
        const double evaluated_yaw_delta = phase.evaluated_trajectories == 0 ? 0.0
            : phase.evaluated_yaw_delta_sum
                / static_cast<double>(phase.evaluated_trajectories);
        const double evaluated_horizontal_path = phase.evaluated_trajectories == 0 ? 0.0
            : phase.evaluated_horizontal_path_sum
                / static_cast<double>(phase.evaluated_trajectories);
        const double evaluated_horizontal_displacement =
            phase.evaluated_trajectories == 0 ? 0.0
            : phase.evaluated_horizontal_displacement_sum
                / static_cast<double>(phase.evaluated_trajectories);
        const double rms_servo_tracking_error = phase.servo_tracking_error_samples == 0
            ? 0.0
            : std::sqrt(phase.servo_tracking_error_sq_sum
                / static_cast<double>(phase.servo_tracking_error_samples));
        const double mean_peak_servo_torque_utilization =
            phase.servo_torque_utilization_samples == 0
            ? 0.0
            : phase.servo_torque_utilization_sum
                / static_cast<double>(phase.servo_torque_utilization_samples);
        const double rms_foot_tracking_error = phase.foot_tracking_error_samples == 0
            ? 0.0
            : std::sqrt(phase.foot_tracking_error_sq_sum
                / static_cast<double>(phase.foot_tracking_error_samples));
        const double rms_contact_foot_world_speed =
            phase.contact_foot_world_step_samples == 0
            ? 0.0
            : std::sqrt(phase.contact_foot_world_step_sq_sum
                / static_cast<double>(phase.contact_foot_world_step_samples))
                / (static_cast<double>(replay_period_us) * 1.0e-6);
        const double mean_contact_target_opposition_speed =
            phase.contact_target_motion_samples == 0
            ? 0.0
            : phase.contact_target_opposition_sum
                / static_cast<double>(phase.contact_target_motion_samples)
                / (static_cast<double>(replay_period_us) * 1.0e-6);
        const double mean_contact_target_counter_yaw_rate =
            phase.contact_target_motion_samples == 0
            ? 0.0
            : phase.contact_target_counter_yaw_sum
                / static_cast<double>(phase.contact_target_motion_samples)
                / (static_cast<double>(replay_period_us) * 1.0e-6);
        const double dt_s = static_cast<double>(replay_period_us) * 1.0e-6;
        const auto meanRate = [dt_s](const double sum, const std::uint64_t samples) {
            return samples == 0 ? 0.0 : sum / static_cast<double>(samples) / dt_s;
        };
        const auto meanValue = [](const double sum, const std::uint64_t samples) {
            return samples == 0 ? 0.0 : sum / static_cast<double>(samples);
        };
        const double mean_midstance_contact_world_speed =
            meanRate(phase.midstance_contact_world_step_sum,
                     phase.midstance_contact_world_step_samples);
        const double mean_midstance_tripod_contact_world_speed =
            meanRate(phase.midstance_tripod_contact_world_step_sum,
                     phase.midstance_tripod_contact_world_step_samples);
        const double mean_midstance_overlap_contact_world_speed =
            meanRate(phase.midstance_overlap_contact_world_step_sum,
                     phase.midstance_overlap_contact_world_step_samples);
        const double mean_n_raw_contact =
            meanValue(phase.n_raw_contact_sum, phase.census_frames);
        const double mean_n_planned = meanValue(phase.n_planned_sum, phase.census_frames);
        const double mean_n_hold = meanValue(phase.n_hold_sum, phase.census_frames);
        const double mean_n_late_swing_extra =
            meanValue(phase.n_late_swing_extra_sum, phase.census_frames);
        const double mean_n_l_parked_contacted =
            meanValue(phase.n_l_parked_contacted_sum, phase.census_frames);
        const double fraction_mixed_parked_stroking =
            meanValue(static_cast<double>(phase.mixed_parked_stroking_frames),
                      phase.census_frames);
        const double fraction_n_contact_ge_5 =
            meanValue(static_cast<double>(phase.n_contact_ge_5_frames),
                      phase.census_frames);
        const double clean_tripod_frame_fraction =
            meanValue(static_cast<double>(phase.clean_tripod_frames),
                      phase.census_frames);
        const double mean_abs_body_pitch =
            meanValue(phase.abs_pitch_sum, phase.attitude_samples);
        const double mean_abs_body_roll =
            meanValue(phase.abs_roll_sum, phase.attitude_samples);
        const double mean_peak_normal_impulse =
            meanValue(phase.peak_normal_impulse_sum, phase.friction_impulse_samples);
        const double mean_peak_friction_impulse =
            meanValue(phase.peak_friction_impulse_sum, phase.friction_impulse_samples);
        const double mean_friction_to_normal_ratio =
            meanValue(phase.friction_to_normal_ratio_sum, phase.friction_ratio_samples);
        const double mean_contact_commanded_world_speed =
            meanRate(phase.contact_commanded_world_step_sum, phase.contact_slip_samples);
        const double mean_contact_uncommanded_slip_speed =
            meanRate(phase.contact_uncommanded_slip_step_sum, phase.contact_slip_samples);
        const double mean_midstance_commanded_world_speed =
            meanRate(phase.midstance_commanded_world_step_sum,
                     phase.midstance_slip_samples);
        const double mean_midstance_uncommanded_slip_speed =
            meanRate(phase.midstance_uncommanded_slip_step_sum,
                     phase.midstance_slip_samples);
        const double mean_clean_tripod_body_speed =
            meanRate(phase.clean_tripod_body_step_sum, phase.clean_tripod_body_samples);
        const double mean_clean_tripod_cartesian_opposition_speed =
            meanRate(phase.clean_tripod_cartesian_opposition_sum,
                     phase.clean_tripod_cartesian_samples);
        const double mean_clean_tripod_cartesian_counter_yaw_rate =
            meanRate(phase.clean_tripod_cartesian_counter_yaw_sum,
                     phase.clean_tripod_cartesian_samples);
        const double mean_clean_tripod_commanded_world_speed =
            meanRate(phase.clean_tripod_commanded_world_step_sum,
                     phase.clean_tripod_slip_samples);
        const double mean_clean_tripod_uncommanded_slip_speed =
            meanRate(phase.clean_tripod_uncommanded_slip_step_sum,
                     phase.clean_tripod_slip_samples);
        const double mean_clean_tripod_contact_world_speed =
            meanRate(phase.clean_tripod_contact_world_step_sum,
                     phase.clean_tripod_slip_samples);
        const PhaseCommand command = phaseCommand(static_cast<ReplayPhase>(i));
        const double frames_per_trajectory = phase.completed_trajectories == 0 ? 0.0
            : static_cast<double>(phase.frames)
                / static_cast<double>(phase.completed_trajectories);
        const double commanded_translation = std::hypot(command.vx_mps, command.vy_mps)
            * frames_per_trajectory * static_cast<double>(replay_period_us) * 1.0e-6;
        const double commanded_yaw = command.yaw_rate_radps
            * frames_per_trajectory * static_cast<double>(replay_period_us) * 1.0e-6;
        double command_progress = 0.0;
        double command_lateral = 0.0;
        const double command_speed = std::hypot(command.vx_mps, command.vy_mps);
        if (command_speed > 0.0) {
            const double ux = command.vx_mps / command_speed;
            const double uy = command.vy_mps / command_speed;
            command_progress = ux * body_forward + uy * body_lateral;
            command_lateral = -uy * body_forward + ux * body_lateral;
        }
        double evaluated_command_progress = 0.0;
        double evaluated_command_lateral = 0.0;
        if (command_speed > 0.0) {
            const double ux = command.vx_mps / command_speed;
            const double uy = command.vy_mps / command_speed;
            evaluated_command_progress =
                ux * evaluated_body_forward + uy * evaluated_body_lateral;
            evaluated_command_lateral =
                -uy * evaluated_body_forward + ux * evaluated_body_lateral;
        }
        const double score_period_us = static_cast<double>(
            commandScorePeriodUs(replay_period_us, capture_period_us));
        const double evaluated_commanded_translation = command_speed
            * static_cast<double>(phase.evaluated_frames)
            / static_cast<double>(std::max<std::uint64_t>(phase.evaluated_trajectories, 1))
            * score_period_us * 1.0e-6;
        const double evaluated_commanded_yaw = command.yaw_rate_radps
            * static_cast<double>(phase.evaluated_frames)
            / static_cast<double>(std::max<std::uint64_t>(phase.evaluated_trajectories, 1))
            * score_period_us * 1.0e-6;
        bool behavior_gate_passed = false;
        if (phase.evaluated_trajectories > 0) {
            if (command_speed > 0.0) {
                behavior_gate_passed =
                    evaluated_command_progress
                        >= kMinimumCommandProgressRatio * evaluated_commanded_translation
                    && std::abs(evaluated_command_lateral)
                        <= kMaximumLateralPathFraction * evaluated_horizontal_path
                            + kLateralAllowanceM;
            } else if (static_cast<ReplayPhase>(i) == ReplayPhase::TurnInPlace) {
                behavior_gate_passed =
                    evaluated_yaw_delta >= kMinimumCommandProgressRatio * evaluated_commanded_yaw
                    && evaluated_horizontal_displacement <= kMaximumTurnTranslationM;
            }
        }
        out << "{\"name\":\"" << kPhaseNames[i]
            << "\",\"frames\":" << phase.frames
            << ",\"healthy\":" << phase.healthy
            << ",\"recovered\":" << phase.recovered
            << ",\"held\":" << phase.held
            << ",\"unsupported\":" << phase.unsupported
            << ",\"read_failures\":" << phase.read_failures
            << ",\"captured_inhibited_frames\":" << captured.inhibited_frames
            << ",\"captured_walk_mode_frames\":" << captured.walk_mode_frames
            << ",\"captured_moving_target_frames\":" << captured.moving_target_frames
            << ",\"captured_max_target_step_rad\":" << captured.max_target_step
            << ",\"captured_max_target_span_rad\":" << captured.max_target_span
            << ",\"captured_planned_stance_samples\":"
            << captured.planned_stance_samples
            << ",\"captured_held_stance_samples\":"
            << captured.held_stance_samples
            << ",\"captured_held_swing_samples\":"
            << captured.held_swing_samples
            << ",\"captured_reverse_target_steps\":"
            << captured.reverse_target_steps
            << ",\"captured_reverse_planned_stance_steps\":"
            << captured.reverse_planned_stance_steps
            << ",\"captured_reverse_held_swing_steps\":"
            << captured.reverse_held_swing_steps
            << ",\"captured_reverse_held_stance_steps\":"
            << captured.reverse_held_stance_steps
            << ",\"captured_reverse_unclassified_steps\":"
            << captured.reverse_unclassified_steps
            << ",\"captured_onset_reverse_steps\":"
            << captured.onset_reverse_steps
            << ",\"captured_midstance_reverse_steps\":"
            << captured.midstance_reverse_steps
            << ",\"captured_midstance_reverse_f_increase_steps\":"
            << captured.midstance_reverse_f_increase_steps
            << ",\"captured_midstance_reverse_phase_drop_steps\":"
            << captured.midstance_reverse_phase_drop_steps
            << ",\"captured_midstance_reverse_other_steps\":"
            << captured.midstance_reverse_other_steps
            << ",\"captured_mean_duty_factor\":"
            << (captured.annotated_frames == 0
                    ? 0.0
                    : captured.duty_factor_sum
                        / static_cast<double>(captured.annotated_frames))
            << ",\"captured_high_duty_frames\":" << captured.high_duty_frames
            << ",\"captured_tripod_frames\":" << captured.tripod_frames
            << ",\"captured_overlap_frames\":" << captured.overlap_frames
            << ",\"captured_safe_to_lift_samples\":"
            << captured.safe_to_lift_samples
            << ",\"captured_min_liftoff_clearance_m\":"
            << (std::isfinite(captured.minimum_liftoff_clearance_m)
                    ? captured.minimum_liftoff_clearance_m
                    : 0.0)
            << ",\"captured_max_liftoff_clearance_m\":"
            << (std::isfinite(captured.maximum_liftoff_clearance_m)
                    ? captured.maximum_liftoff_clearance_m
                    : 0.0)
            << ",\"captured_min_static_margin_m\":"
            << (std::isfinite(captured.minimum_static_margin_m)
                    ? captured.minimum_static_margin_m
                    : 0.0)
            << ",\"captured_max_static_margin_m\":"
            << (std::isfinite(captured.maximum_static_margin_m)
                    ? captured.maximum_static_margin_m
                    : 0.0)
            << ",\"captured_mean_stance_target_opposition_speed_mps\":"
            << meanRate(captured.stance_target_opposition_sum,
                        captured.stance_target_motion_samples)
            << ",\"captured_mean_stance_target_counter_yaw_rate_radps\":"
            << meanRate(captured.stance_target_counter_yaw_sum,
                        captured.stance_target_motion_samples)
            << ",\"captured_mean_midstance_opposition_speed_mps\":"
            << meanRate(captured.midstance_opposition_sum,
                        captured.midstance_motion_samples)
            << ",\"captured_mean_midstance_counter_yaw_rate_radps\":"
            << meanRate(captured.midstance_counter_yaw_sum,
                        captured.midstance_motion_samples)
            << ",\"captured_mean_midstance_tripod_opposition_speed_mps\":"
            << meanRate(captured.midstance_tripod_opposition_sum,
                        captured.midstance_tripod_motion_samples)
            << ",\"captured_mean_midstance_tripod_counter_yaw_rate_radps\":"
            << meanRate(captured.midstance_tripod_counter_yaw_sum,
                        captured.midstance_tripod_motion_samples)
            << ",\"captured_mean_midstance_overlap_opposition_speed_mps\":"
            << meanRate(captured.midstance_overlap_opposition_sum,
                        captured.midstance_overlap_motion_samples)
            << ",\"captured_mean_midstance_overlap_counter_yaw_rate_radps\":"
            << meanRate(captured.midstance_overlap_counter_yaw_sum,
                        captured.midstance_overlap_motion_samples)
            << ",\"captured_mean_midstance_high_duty_opposition_speed_mps\":"
            << meanRate(captured.midstance_high_duty_opposition_sum,
                        captured.midstance_high_duty_motion_samples)
            << ",\"captured_mean_midstance_high_duty_counter_yaw_rate_radps\":"
            << meanRate(captured.midstance_high_duty_counter_yaw_sum,
                        captured.midstance_high_duty_motion_samples)
            << ",\"captured_mean_midstance_stride_hz\":"
            << meanValue(captured.midstance_stride_hz_sum,
                         captured.midstance_stroke_clamp_samples)
            << ",\"captured_mean_midstance_command_scale\":"
            << meanValue(captured.midstance_command_scale_sum,
                         captured.midstance_stroke_clamp_samples)
            << ",\"captured_mean_midstance_cadence_scale\":"
            << meanValue(captured.midstance_cadence_scale_sum,
                         captured.midstance_stroke_clamp_samples)
            << ",\"captured_midstance_stroke_clamp_hit_fraction\":"
            << meanValue(static_cast<double>(captured.midstance_stroke_clamp_hit_samples),
                         captured.midstance_stroke_clamp_samples)
            << ",\"captured_midstance_workspace_xy_hit_fraction\":"
            << meanValue(static_cast<double>(captured.midstance_workspace_xy_hit_samples),
                         captured.midstance_stroke_clamp_samples)
            << ",\"captured_mean_midstance_tripod_cartesian_opposition_speed_mps_plant_hit\":"
            << meanRate(captured.midstance_tripod_cartesian_plant_hit_opposition_sum,
                        captured.midstance_tripod_cartesian_plant_hit_samples)
            << ",\"captured_mean_midstance_tripod_cartesian_opposition_speed_mps_plant_miss\":"
            << meanRate(captured.midstance_tripod_cartesian_plant_miss_opposition_sum,
                        captured.midstance_tripod_cartesian_plant_miss_samples)
            << ",\"captured_mean_midstance_tripod_cartesian_opposition_speed_mps_workspace_xy_hit\":"
            << meanRate(captured.midstance_tripod_cartesian_workspace_xy_hit_opposition_sum,
                        captured.midstance_tripod_cartesian_workspace_xy_hit_samples)
            << ",\"captured_mean_midstance_tripod_cartesian_opposition_speed_mps_workspace_xy_miss\":"
            << meanRate(captured.midstance_tripod_cartesian_workspace_xy_miss_opposition_sum,
                        captured.midstance_tripod_cartesian_workspace_xy_miss_samples)
            << ",\"captured_mean_midstance_tripod_cartesian_opposition_speed_mps_neither_hit\":"
            << meanRate(captured.midstance_tripod_cartesian_neither_hit_opposition_sum,
                        captured.midstance_tripod_cartesian_neither_hit_samples)
            << ",\"captured_mean_midstance_cartesian_opposition_speed_mps\":"
            << meanRate(captured.midstance_cartesian_opposition_sum,
                        captured.midstance_cartesian_motion_samples)
            << ",\"captured_mean_midstance_tripod_cartesian_opposition_speed_mps\":"
            << meanRate(captured.midstance_tripod_cartesian_opposition_sum,
                        captured.midstance_tripod_cartesian_motion_samples)
            << ",\"captured_mean_midstance_cartesian_counter_yaw_rate_radps\":"
            << meanRate(captured.midstance_cartesian_counter_yaw_sum,
                        captured.midstance_cartesian_yaw_samples)
            << ",\"captured_mean_midstance_tripod_cartesian_counter_yaw_rate_radps\":"
            << meanRate(captured.midstance_tripod_cartesian_counter_yaw_sum,
                        captured.midstance_tripod_cartesian_yaw_samples)
            << ",\"captured_mean_midstance_ik_opposition_speed_mps\":"
            << meanRate(captured.midstance_ik_opposition_sum,
                        captured.midstance_ik_motion_samples)
            << ",\"captured_mean_midstance_tripod_ik_opposition_speed_mps\":"
            << meanRate(captured.midstance_tripod_ik_opposition_sum,
                        captured.midstance_tripod_ik_motion_samples)
            << ",\"captured_mean_midstance_ik_counter_yaw_rate_radps\":"
            << meanRate(captured.midstance_ik_counter_yaw_sum,
                        captured.midstance_ik_yaw_samples)
            << ",\"captured_mean_midstance_tripod_ik_counter_yaw_rate_radps\":"
            << meanRate(captured.midstance_tripod_ik_counter_yaw_sum,
                        captured.midstance_tripod_ik_yaw_samples)
            << ",\"captured_mean_midstance_aligned_fk_opposition_speed_mps\":"
            << meanRate(captured.midstance_aligned_fk_opposition_sum,
                        captured.midstance_aligned_fk_motion_samples)
            << ",\"captured_mean_midstance_tripod_aligned_fk_opposition_speed_mps\":"
            << meanRate(captured.midstance_tripod_aligned_fk_opposition_sum,
                        captured.midstance_tripod_aligned_fk_motion_samples)
            << ",\"captured_mean_midstance_aligned_fk_counter_yaw_rate_radps\":"
            << meanRate(captured.midstance_aligned_fk_counter_yaw_sum,
                        captured.midstance_aligned_fk_yaw_samples)
            << ",\"captured_mean_midstance_tripod_aligned_fk_counter_yaw_rate_radps\":"
            << meanRate(captured.midstance_tripod_aligned_fk_counter_yaw_sum,
                        captured.midstance_tripod_aligned_fk_yaw_samples)
            << ",\"captured_midstance_ik_reach_hit_fraction\":"
            << meanValue(static_cast<double>(captured.midstance_ik_reach_hit_samples),
                         captured.midstance_stroke_clamp_samples)
            << ",\"captured_midstance_slew_hit_fraction\":"
            << meanValue(static_cast<double>(captured.midstance_slew_hit_samples),
                         captured.midstance_stroke_clamp_samples)
            << ",\"captured_mean_midstance_post_clamp_distortion_m\":"
            << meanValue(captured.midstance_post_clamp_distortion_sum,
                         captured.midstance_stroke_clamp_samples)
            << ",\"captured_mean_midstance_governed_command_speed_mps\":"
            << meanValue(captured.midstance_governed_command_speed_sum,
                         captured.midstance_stroke_clamp_samples)
            << ",\"solver_not_converged\":" << phase.solver_not_converged
            << ",\"topology_changes\":" << phase.topology_changes
            << ",\"high_iteration_topology_changes\":"
            << phase.high_iteration_topology_changes
            << ",\"high_iteration_persistent_contacts\":"
            << phase.high_iteration_persistent_contacts
            << ",\"p50_iterations\":"
            << iterationPercentile(phase.iteration_histogram, 0.50)
            << ",\"p90_iterations\":"
            << iterationPercentile(phase.iteration_histogram, 0.90)
            << ",\"p99_iterations\":"
            << iterationPercentile(phase.iteration_histogram, 0.99)
            << ",\"max_iterations\":" << phase.max_iterations
            << ",\"max_ncp_dual_residual\":" << phase.max_ncp_dual_residual
            << ",\"max_ncp_complementarity_residual\":"
            << phase.max_ncp_complementarity_residual
            << ",\"max_contact_penetration_m\":" << phase.max_contact_penetration
            << ",\"max_body_height_error_m\":" << phase.max_body_height_error
            << ",\"max_servo_tracking_error_rad\":"
            << phase.max_servo_tracking_error
            << ",\"rms_servo_tracking_error_rad\":" << rms_servo_tracking_error
            << ",\"terminal_servo_tracking_error_rad\":"
            << phase.terminal_servo_tracking_error
            << ",\"max_servo_torque_utilization\":"
            << phase.max_servo_torque_utilization
            << ",\"mean_peak_servo_torque_utilization\":"
            << mean_peak_servo_torque_utilization
            << ",\"servo_saturated_frames\":" << phase.servo_saturated_frames
            << ",\"peak_actuator_impulse_ns\":" << phase.peak_actuator_impulse
            << ",\"peak_normal_impulse_ns\":" << phase.peak_normal_impulse
            << ",\"peak_friction_impulse_ns\":" << phase.peak_friction_impulse
            << ",\"peak_preintegration_linear_speed_mps\":"
            << phase.peak_preintegration_linear_speed
            << ",\"peak_preintegration_angular_speed_radps\":"
            << phase.peak_preintegration_angular_speed
            << ",\"actuator_work_j\":" << phase.actuator_work_sum
            << ",\"mechanical_energy_delta_j\":"
            << phase.mechanical_energy_delta_sum
            << ",\"max_foot_tracking_error_m\":" << phase.max_foot_tracking_error
            << ",\"rms_foot_tracking_error_m\":" << rms_foot_tracking_error
            << ",\"max_contact_foot_world_step_m\":"
            << phase.max_contact_foot_world_step
            << ",\"rms_contact_foot_world_speed_mps\":"
            << rms_contact_foot_world_speed
            << ",\"mean_midstance_contact_world_speed_mps\":"
            << mean_midstance_contact_world_speed
            << ",\"mean_midstance_tripod_contact_world_speed_mps\":"
            << mean_midstance_tripod_contact_world_speed
            << ",\"mean_midstance_overlap_contact_world_speed_mps\":"
            << mean_midstance_overlap_contact_world_speed
            << ",\"mean_n_raw_contact\":" << mean_n_raw_contact
            << ",\"mean_n_planned\":" << mean_n_planned
            << ",\"mean_n_hold\":" << mean_n_hold
            << ",\"mean_n_late_swing_extra\":" << mean_n_late_swing_extra
            << ",\"mean_n_L_parked_contacted\":" << mean_n_l_parked_contacted
            << ",\"n_raw_contact_histogram\":[";
        for (std::size_t bin = 0; bin < phase.n_raw_contact_histogram.size(); ++bin) {
            if (bin != 0) {
                out << ',';
            }
            out << phase.n_raw_contact_histogram[bin];
        }
        out << "]"
            << ",\"fraction_mixed_parked_stroking\":" << fraction_mixed_parked_stroking
            << ",\"fraction_n_contact_ge_5\":" << fraction_n_contact_ge_5
            << ",\"clean_tripod_frames\":" << phase.clean_tripod_frames
            << ",\"clean_tripod_frame_fraction\":" << clean_tripod_frame_fraction
            << ",\"mean_abs_body_pitch_rad\":" << mean_abs_body_pitch
            << ",\"mean_abs_body_roll_rad\":" << mean_abs_body_roll
            << ",\"mean_peak_normal_impulse_ns\":" << mean_peak_normal_impulse
            << ",\"mean_peak_friction_impulse_ns\":" << mean_peak_friction_impulse
            << ",\"mean_friction_to_normal_impulse_ratio\":"
            << mean_friction_to_normal_ratio
            << ",\"mean_contact_commanded_world_speed_mps\":"
            << mean_contact_commanded_world_speed
            << ",\"mean_contact_uncommanded_slip_speed_mps\":"
            << mean_contact_uncommanded_slip_speed
            << ",\"mean_midstance_commanded_world_speed_mps\":"
            << mean_midstance_commanded_world_speed
            << ",\"mean_midstance_uncommanded_slip_speed_mps\":"
            << mean_midstance_uncommanded_slip_speed
            << ",\"mean_clean_tripod_body_speed_mps\":" << mean_clean_tripod_body_speed
            << ",\"mean_clean_tripod_cartesian_opposition_speed_mps\":"
            << mean_clean_tripod_cartesian_opposition_speed
            << ",\"mean_clean_tripod_cartesian_counter_yaw_rate_radps\":"
            << mean_clean_tripod_cartesian_counter_yaw_rate
            << ",\"mean_clean_tripod_commanded_world_speed_mps\":"
            << mean_clean_tripod_commanded_world_speed
            << ",\"mean_clean_tripod_uncommanded_slip_speed_mps\":"
            << mean_clean_tripod_uncommanded_slip_speed
            << ",\"mean_clean_tripod_contact_world_speed_mps\":"
            << mean_clean_tripod_contact_world_speed
            << ",\"mean_contact_target_opposition_speed_mps\":"
            << mean_contact_target_opposition_speed
            << ",\"mean_contact_target_counter_yaw_rate_radps\":"
            << mean_contact_target_counter_yaw_rate
            << ",\"valid_delta_x_m\":" << dx
            << ",\"valid_delta_y_m\":" << dy
            << ",\"body_forward_delta_m\":" << body_forward
            << ",\"body_lateral_delta_m\":" << body_lateral
            << ",\"command_progress_m\":" << command_progress
            << ",\"command_lateral_m\":" << command_lateral
            << ",\"commanded_translation_m\":" << commanded_translation
            << ",\"yaw_delta_rad\":" << yaw_delta
            << ",\"commanded_yaw_rad\":" << commanded_yaw
            << ",\"horizontal_path_m\":" << horizontal_path
            << ",\"horizontal_displacement_m\":" << horizontal_displacement
            << ",\"start_body_forward_velocity_mps\":" << start_body_forward_velocity
            << ",\"start_body_lateral_velocity_mps\":" << start_body_lateral_velocity
            << ",\"end_body_forward_velocity_mps\":" << end_body_forward_velocity
            << ",\"end_body_lateral_velocity_mps\":" << end_body_lateral_velocity
            << ",\"evaluated_command_progress_m\":" << evaluated_command_progress
            << ",\"evaluated_command_lateral_m\":" << evaluated_command_lateral
            << ",\"evaluated_commanded_translation_m\":"
            << evaluated_commanded_translation
            << ",\"evaluated_yaw_delta_rad\":" << evaluated_yaw_delta
            << ",\"evaluated_commanded_yaw_rad\":" << evaluated_commanded_yaw
            << ",\"evaluated_horizontal_path_m\":" << evaluated_horizontal_path
            << ",\"evaluated_horizontal_displacement_m\":"
            << evaluated_horizontal_displacement
            << ",\"evaluated_frames\":" << phase.evaluated_frames
            << ",\"behavior_gate_passed\":"
            << (behavior_gate_passed ? "true" : "false")
            << ",\"completed_trajectories\":" << phase.completed_trajectories << '}';
    }
    out << "]}";
    return out.str();
}

} // namespace

int main(int argc, char** argv) {
#if !defined(__linux__)
    std::cout << "skip test_physics_sim_exact_command_replay (Linux-only)\n";
    return 0;
#else
    bool emit_metrics_json = false;
    const char* sim_exe = nullptr;
    physics_sim_test_argv::parse(argc, argv, emit_metrics_json, sim_exe);
    if (sim_exe == nullptr || sim_exe[0] == '\0') {
        std::cout << "skip test_physics_sim_exact_command_replay "
                     "(pass sim path or HEXAPOD_PHYSICS_SIM_EXE)\n";
        return 0;
    }

    try {
        auto harness = physics_sim_test_utils::loadHarnessSettings(true);
        if (const char* blend = std::getenv("HEXAPOD_FOOT_ESTIMATOR_BLEND")) {
            harness.control_cfg.gait.foot_estimator_blend =
                std::clamp(std::atof(blend), 0.0, 1.0);
        }
        if (const char* swing_scale =
                std::getenv("HEXAPOD_EXACT_REPLAY_SWING_HEIGHT_SCALE")) {
            harness.control_cfg.gait.swing_height_scale =
                std::clamp(std::atof(swing_scale), 0.25, 3.0);
        }
        int stand_frames = positiveEnvOrDefault("HEXAPOD_EXACT_REPLAY_STAND_FRAMES", 240);
        int motion_frames = positiveEnvOrDefault("HEXAPOD_EXACT_REPLAY_MOTION_FRAMES", 72);
        int transition_frames =
            positiveEnvOrDefault("HEXAPOD_EXACT_REPLAY_TRANSITION_FRAMES", 24);
        const int solver_iterations =
            positiveEnvOrDefault("HEXAPOD_EXACT_REPLAY_SOLVER_ITERATIONS", 50);
        int replay_period_us = positiveEnvOrDefault(
            "HEXAPOD_EXACT_REPLAY_PERIOD_US", harness.bus_loop_period_us);
        double body_height_m = positiveDoubleEnvOrDefault(
            "HEXAPOD_EXACT_REPLAY_BODY_HEIGHT_M", 0.14);
        const double proximal_mu = positiveDoubleEnvOrDefault(
            "HEXAPOD_EXACT_REPLAY_PROXIMAL_MU", 1.0e-6);
        const double contact_regularization = positiveDoubleEnvOrDefault(
            "HEXAPOD_EXACT_REPLAY_CONTACT_REGULARIZATION", 1.0e-10);
        const double absolute_tolerance = positiveDoubleEnvOrDefault(
            "HEXAPOD_EXACT_REPLAY_ABSOLUTE_TOLERANCE", 1.0e-8);
        const double relative_tolerance = positiveDoubleEnvOrDefault(
            "HEXAPOD_EXACT_REPLAY_RELATIVE_TOLERANCE", 1.0e-6);
        const int perturbation_seed_count = positiveEnvOrDefault(
            "HEXAPOD_EXACT_REPLAY_PERTURBATION_SEEDS", 1);
        const int perturbation_seed_offset = nonnegativeEnvOrDefault(
            "HEXAPOD_EXACT_REPLAY_PERTURBATION_SEED_OFFSET", 0);
        const double perturbation_scale = positiveDoubleEnvOrDefault(
            "HEXAPOD_EXACT_REPLAY_PERTURBATION_SCALE", 0.25);
        const bool fixed_initial_pose = envEnabled(
            "HEXAPOD_EXACT_REPLAY_FIXED_INITIAL_POSE");
        const bool fixed_contact_order = envEnabled(
            "HEXAPOD_EXACT_REPLAY_FIXED_CONTACT_ORDER");
        const bool dense_admm = envEnabled("HEXAPOD_PINOCCHIO_DENSE_ADMM");
        const bool contact_precondition = envEnabled(
            "HEXAPOD_PINOCCHIO_CONTACT_PRECONDITION");
        const char* command_fixture_input =
            std::getenv("HEXAPOD_EXACT_REPLAY_COMMANDS_IN");
        const char* command_fixture_output =
            std::getenv("HEXAPOD_EXACT_REPLAY_COMMANDS_OUT");
        const bool command_fixture_loaded = command_fixture_input != nullptr
            && command_fixture_input[0] != '\0';
        const bool command_fixture_written = command_fixture_output != nullptr
            && command_fixture_output[0] != '\0';
        const physics_sim::PhysicsSolverMode replay_solver_mode =
            envEnabled("HEXAPOD_EXACT_REPLAY_LEGACY")
                ? physics_sim::PhysicsSolverMode::LegacyPgs
                : physics_sim::PhysicsSolverMode::PinocchioProximal;
        const physics_sim::PhysicsSolverMode capture_solver_mode =
            envEnabled("HEXAPOD_EXACT_REPLAY_CAPTURE_LEGACY")
                ? physics_sim::PhysicsSolverMode::LegacyPgs
                : physics_sim::PhysicsSolverMode::PinocchioProximal;
        const int capture_solver_iterations =
            capture_solver_mode == physics_sim::PhysicsSolverMode::LegacyPgs
                ? harness.physics_solver_iterations
                : positiveEnvOrDefault(
                    "HEXAPOD_EXACT_REPLAY_CAPTURE_SOLVER_ITERATIONS", 500);
        if (perturbation_seed_count > 1000) {
            throw std::runtime_error(
                "HEXAPOD_EXACT_REPLAY_PERTURBATION_SEEDS must be at most 1000");
        }
        std::optional<ReplayPhase> selected_phase = selectedMotionPhase();
        const int base_port = 23500 + (static_cast<int>(::getpid()) % 4000);

        CommandFixture fixture{};
        if (command_fixture_loaded) {
            fixture = loadCommandFixture(command_fixture_input);
            if ((envProvided("HEXAPOD_EXACT_REPLAY_STAND_FRAMES")
                 && stand_frames != fixture.stand_frames)
                || (envProvided("HEXAPOD_EXACT_REPLAY_MOTION_FRAMES")
                    && motion_frames != fixture.motion_frames)
                || (envProvided("HEXAPOD_EXACT_REPLAY_TRANSITION_FRAMES")
                    && transition_frames != fixture.transition_frames)
                || (envProvided("HEXAPOD_EXACT_REPLAY_BODY_HEIGHT_M")
                    && std::abs(body_height_m - fixture.commanded_body_height_m) > 1.0e-12)) {
                throw std::runtime_error(
                    "command fixture metadata conflicts with an explicit replay override");
            }
            const int requested_phase = selected_phase.has_value()
                ? static_cast<int>(*selected_phase)
                : -1;
            if (envProvided("HEXAPOD_EXACT_REPLAY_MOTION_CASE")
                && requested_phase != fixture.selected_phase) {
                throw std::runtime_error(
                    "command fixture motion phase conflicts with the replay selection");
            }
            stand_frames = fixture.stand_frames;
            motion_frames = fixture.motion_frames;
            transition_frames = fixture.transition_frames;
            body_height_m = fixture.commanded_body_height_m;
            selected_phase = fixture.selected_phase < 0
                ? std::nullopt
                : std::optional<ReplayPhase>{
                    static_cast<ReplayPhase>(fixture.selected_phase)};
            if (!envProvided("HEXAPOD_EXACT_REPLAY_PERIOD_US")) {
                replay_period_us = fixture.capture_period_us;
            }
        } else {
            fixture.capture_period_us = harness.bus_loop_period_us;
            fixture.stand_frames = stand_frames;
            fixture.motion_frames = motion_frames;
            fixture.transition_frames = transition_frames;
            fixture.commanded_body_height_m = body_height_m;
            fixture.initial_body_position = {
                0.0, physicsSimStandingBodyHeightM(), 0.0};
            fixture.initial_body_orientation = {1.0, 0.0, 0.0, 0.0};
            fixture.selected_phase = selected_phase.has_value()
                ? static_cast<int>(*selected_phase)
                : -1;
            fixture.capture_solver_mode = capture_solver_mode;
            fixture.capture_solver_iterations = capture_solver_iterations;
            pid_t capture_pid = launchSimulator(sim_exe, base_port);
            if (capture_pid < 0) {
                throw std::runtime_error("failed to fork reference capture simulator");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds{250});
            try {
                fixture.frames = captureCommands(
                    harness,
                    base_port,
                    stand_frames,
                    motion_frames,
                    transition_frames,
                    body_height_m,
                    selected_phase,
                    capture_solver_mode,
                    capture_solver_iterations);
            } catch (...) {
                stopSimulator(capture_pid);
                throw;
            }
            stopSimulator(capture_pid);
        }
        validateCommandFixture(fixture);
        if (command_fixture_written) {
            saveCommandFixture(command_fixture_output, fixture);
        }
        const std::vector<CapturedFrame>& frames = fixture.frames;
        const std::uint64_t command_hash = commandStreamHash(fixture);

        ReplayResult result{};
        std::uint64_t behavior_gate_failures = 0;
        for (int seed = 0; seed < perturbation_seed_count; ++seed) {
            const std::uint64_t absolute_seed = static_cast<std::uint64_t>(
                perturbation_seed_offset + seed);
            const std::uint64_t pose_seed = fixed_initial_pose ? 0 : absolute_seed;
            const std::uint64_t contact_order_seed = fixed_contact_order ? 0 : absolute_seed;
            const int replay_port = base_port + 1 + seed;
            pid_t replay_pid = launchSimulator(
                sim_exe, replay_port, contact_order_seed);
            if (replay_pid < 0) {
                throw std::runtime_error("failed to fork proximal replay simulator");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds{250});
            ReplayResult seed_result{};
            try {
                seed_result = replayCommands(
                    frames,
                    replay_port,
                    replay_period_us,
                    replay_solver_mode,
                    solver_iterations,
                    body_height_m,
                    fixture.initial_body_position,
                    fixture.initial_body_orientation,
                    proximal_mu,
                    contact_regularization,
                    absolute_tolerance,
                    relative_tolerance,
                    pose_seed,
                    perturbation_scale);
            } catch (...) {
                stopSimulator(replay_pid);
                throw;
            }
            stopSimulator(replay_pid);
            if (seed_result.recovered != 0 || seed_result.held != 0
                || seed_result.unsupported != 0 || seed_result.read_failures != 0) {
                std::cerr << "replay seed " << absolute_seed
                          << " healthy=" << seed_result.healthy
                          << " recovered=" << seed_result.recovered
                          << " held=" << seed_result.held
                          << " unsupported=" << seed_result.unsupported
                          << " read_failures=" << seed_result.read_failures
                          << " max_iterations=" << seed_result.max_iterations << '\n';
            }
            if (!passesBehaviorGates(
                    seed_result, replay_period_us, fixture.capture_period_us)) {
                ++behavior_gate_failures;
            }
            accumulateReplayResult(result, seed_result);
        }

        const std::size_t expected_replayed_frames =
            frames.size() * static_cast<std::size_t>(perturbation_seed_count);
        const bool accounting_ok = result.frames == expected_replayed_frames
            && result.telemetry_frames == expected_replayed_frames
            && result.healthy + result.recovered + result.held + result.unsupported
                == result.telemetry_frames;
        const bool gates_requested = envEnabled("HEXAPOD_EXACT_REPLAY_ENFORCE_GATES");
        const bool safety_gates_requested = envEnabled(
            "HEXAPOD_EXACT_REPLAY_ENFORCE_SAFETY_GATES");
        const bool behavior_gates_requested = envEnabled(
            "HEXAPOD_EXACT_REPLAY_ENFORCE_BEHAVIOR_GATES");
        const bool gates_ok = !gates_requested
            || (result.recovered == 0 && result.held == 0 && result.unsupported == 0
                && result.read_failures == 0);
        const bool safety_gates_ok = !safety_gates_requested
            || (result.held == 0 && result.unsupported == 0
                && result.read_failures == 0);
        const bool behavior_gates_ok = !behavior_gates_requested
            || behavior_gate_failures == 0;
        const bool passed = accounting_ok && gates_ok && safety_gates_ok
            && behavior_gates_ok;
        const std::string metrics = metricsJson(result,
                                                command_hash,
                                                frames,
                                                command_fixture_loaded,
                                                command_fixture_written,
                                                perturbation_seed_count,
                                                perturbation_seed_offset,
                                                perturbation_scale,
                                                fixed_initial_pose,
                                                fixed_contact_order,
                                                dense_admm,
                                                contact_precondition,
                                                fixture.capture_solver_mode,
                                                fixture.capture_solver_iterations,
                                                behavior_gates_requested,
                                                behavior_gate_failures,
                                                replay_period_us,
                                                fixture.capture_period_us,
                                                solver_iterations,
                                                body_height_m,
                                                proximal_mu,
                                                contact_regularization,
                                                absolute_tolerance,
                                                relative_tolerance);
        if (emit_metrics_json) {
            std::cout << "{\"suite\":\"physics_sim_exact_command_replay\","
                         "\"case\":\"deterministic_capture_replay\",\"solver_mode\":\""
                      << (replay_solver_mode == physics_sim::PhysicsSolverMode::LegacyPgs
                              ? "legacy-pgs"
                              : "pinocchio-proximal")
                      << "\",\"passed\":" << (passed ? "true" : "false")
                      << ",\"metrics\":" << metrics << "}\n";
        } else {
            std::cout << "exact command replay: " << (passed ? "PASS" : "FAIL")
                      << " captured=" << frames.size() << " hash=" << std::hex << command_hash
                      << std::dec << " seeds=" << perturbation_seed_count
                      << " healthy=" << result.healthy
                      << " recovered=" << result.recovered << " held=" << result.held
                      << " unsupported=" << result.unsupported
                      << " max_iterations=" << result.max_iterations << '\n';
        }
        return passed ? EXIT_SUCCESS : EXIT_FAILURE;
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
#endif
}
