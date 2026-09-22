#pragma once

#include "body_controller.hpp"
#include "command_governor.hpp"
#include "control_config.hpp"
#include "gait_scheduler.hpp"
#include "leg_ik.hpp"
#include "local_map.hpp"
#include "locomotion_command.hpp"
#include "locomotion_feasibility.hpp"
#include "locomotion_stability.hpp"
#include "runtime_resource_monitoring.hpp"
#include "types.hpp"
#include "in_place_turn_hold.hpp"

struct PipelineStepResult {
    LegTargets leg_targets{};
    JointTargets joint_targets{};
    ControlStatus status{};
    GaitState gait_state{};
    CommandGovernorState command_governor{};
    LocomotionFeasibility locomotion_feasibility{};
    std::array<bool, kNumLegs> stroke_clamp_hit{};
    std::array<bool, kNumLegs> workspace_xy_hit{};
    std::array<bool, kNumLegs> ik_reach_clamp_hit{};
    std::array<double, kNumLegs> latched_stroke_length_m{};
    std::array<Vec3, kNumLegs> latched_plant_position_m{};
    R2SwingDecompSnapshot r2_swing_decomp{};
};

class ControlPipeline {
public:
    explicit ControlPipeline(control_config::GaitConfig gait_config = {},
                             control_config::LocomotionCommandConfig loco_config = {},
                             control_config::SafetyConfig safety_config = {},
                             control_config::CommandGovernorConfig governor_config = {},
                             control_config::FootTerrainConfig foot_terrain_config = {},
                             control_config::GravityFeedforwardConfig gravity_feedforward_config = {},
                             control_config::LocomotionRedesignConfig locomotion_redesign_config = {},
                             runtime_resource_monitoring::Profiler* profiler = nullptr,
                             bool absolute_position_feedback = false);

    void reset();
    /// Test-only: restore last gait / stride integrator from a turn-entry dump.
    void debugRestoreGaitHistory(const GaitState& gait);
    [[nodiscard]] const control_config::CommandGovernorConfig& commandGovernorConfig() const;
    PipelineStepResult runStep(const RobotState& estimated,
                               const MotionIntent& intent,
                               const SafetyState& safety_state,
                               bool bus_ok,
                               uint64_t loop_counter,
                               const LocalMapSnapshot* terrain_snapshot = nullptr,
                               double control_dt_s = 0.004);

private:
    /**
     * A pure in-place turn commands zero planar velocity, so nothing closes a
     * loop on body translation: any per-stride asymmetry integrates straight
     * into the scored `net_horizontal_distance_m`. Leftover §4.2 measures
     * 0.04 m/s of parasitic translation and a 144 mm migration of the rotation
     * centre. This adds a bounded body-frame planar command opposing measured
     * drift from the latched turn origin, so "in place" is regulated rather
     * than assumed. Gait timing, duty and stroke are untouched.
     */
    void applyInPlaceTurnTranslationHold(const RobotState& estimated,
                                         const MotionIntent& intent,
                                         BodyTwist& cmd_twist);

    runtime_resource_monitoring::Profiler* profiler_{nullptr};
    CommandGovernor command_governor_{};
    GaitScheduler gait_;
    LocomotionCommandProcessor loco_cmd_{};
    LocomotionStability locomotion_stability_{};
    BodyController body_;
    LegIK ik_;
    control_config::GravityFeedforwardConfig gravity_feedforward_{};
    control_config::LocomotionRedesignConfig locomotion_redesign_{};
    GaitState last_gait_state_{};
    bool have_last_gait_state_{false};
    InPlaceTurnHold turn_hold_{};
    bool absolute_position_feedback_{false};
};
