#pragma once

#include "control_config.hpp"
#include "geometry_config.hpp"
#include "local_map.hpp"
#include "swing_trajectory.hpp"
#include "locomotion_feasibility.hpp"
#include "twist_field.hpp"
#include "types.hpp"

/** Diagnostic split of R2 walking swing: untilted Bezier vs origin/coxa rotation vs clamp. */
struct R2SwingDecompSnapshot {
    bool valid{false};
    Vec3 anchor{};
    Vec3 stance_end{};
    Vec3 v_liftoff_body{};
    double tau01{0.0};
    double swing_span{0.0};
    double f_hz{0.0};
    double step_length_m{0.0};
    double swing_height_m{0.0};
    double cmd_accel_body_x_mps2{0.0};
    double cmd_accel_body_y_mps2{0.0};
    double stance_lookahead_s{0.0};
    double static_stability_margin_m{0.0};
    double swing_time_ease_01{0.0};
    BodyTwist kinematic_twist{};
    bool est_valid{false};
    bool est_has_body_twist{false};
    Vec3 est_linear_mps{};
    Vec3 est_angular_radps{};
    Vec3 planned_pre_rot{};
    Vec3 after_terrain{};
    Vec3 origin_rot{};
    Vec3 coxa_rot{};
    Vec3 coxa{};
    Vec3 target_clamped{};
    Vec3 foothold_nominal{};
    Vec3 capture_body{};
    Vec3 foothold_final{};
    Vec3 terrain_xy_delta{};
    double capture_limit_m{0.0};
    double clamp_dxy{0.0};
    double roll_rad{0.0};
    double pitch_rad{0.0};
    double yaw_rad{0.0};
};

class BodyController {
public:
    explicit BodyController(control_config::GaitConfig gait_cfg = {},
                            control_config::FootTerrainConfig foot_terrain_cfg = {});

    LegTargets update(const RobotState& est,
                      const MotionIntent& intent,
                      const GaitState& gait,
                      const SafetyState& safety,
                      const BodyTwist& cmd_twist,
                      const LocalMapSnapshot* terrain_snapshot = nullptr,
                      const std::array<LegContactDecision, kNumLegs>* contact_modes = nullptr);

    void reset();

    [[nodiscard]] std::array<bool, kNumLegs> lastStrokeClampHit() const {
        return last_stroke_clamp_hit_;
    }

    [[nodiscard]] std::array<bool, kNumLegs> lastWorkspaceXyHit() const {
        return last_workspace_xy_hit_;
    }

    [[nodiscard]] std::array<double, kNumLegs> latchedStrokeLengthM() const {
        return latched_stroke_l_m_;
    }

    [[nodiscard]] std::array<Vec3, kNumLegs> latchedPlantPositionM() const {
        return latched_plant_pos_;
    }

    [[nodiscard]] R2SwingDecompSnapshot lastR2SwingDecomp() const {
        return last_r2_swing_decomp_;
    }

private:
    std::array<Vec3, kNumLegs> nominalStance(double body_height_m) const;

    /**
     * Resolve this leg's swing plan once per swing and reuse it afterwards, so the
     * commanded foot path is continuous. The live foothold capture term integrates
     * the measured body twist over ~0.95 s and `stance_end` adds a further
     * `duty / f_hz`; re-resolving per sample let estimator noise move the whole
     * Bezier by up to 92 mm between samples (leftover §3.15).
     */
    const swing_trajectory::SwingPlanCommit& commitSwingPlan(std::size_t leg_index,
                                                            const RobotState& est,
                                                            const BodyTwist& nominal_body,
                                                            const struct SwingFootInputs& in);

    HexapodGeometry geometry_{defaultHexapodGeometry()};
    double foot_estimator_blend_{control_config::kDefaultFootEstimatorBlend};
    control_config::FootTerrainConfig foot_terrain_cfg_{};
    double height_hold_integral_m_{0.0};
    TimePointUs last_intent_timestamp_us_{};
    std::array<bool, kNumLegs> have_stance_pos_{};
    std::array<Vec3, kNumLegs> latched_stance_pos_{};
    std::array<Vec3, kNumLegs> latched_plant_pos_{};
    std::array<double, kNumLegs> latched_stroke_l_m_{};
    std::array<bool, kNumLegs> last_planned_stance_{};
    std::array<bool, kNumLegs> last_stroke_clamp_hit_{};
    std::array<bool, kNumLegs> last_workspace_xy_hit_{};
    std::array<bool, kNumLegs> have_last_clamped_stance_{};
    std::array<Vec3, kNumLegs> last_clamped_stance_body_{};
    // One swing plan committed per swing. Re-resolving every control sample made
    // the commanded foot path discontinuous; see `SwingPlanCommit`.
    std::array<swing_trajectory::SwingPlanCommit, kNumLegs> committed_swing_plan_{};
    std::array<Vec3, kNumLegs> last_emitted_target_{};
    std::array<bool, kNumLegs> have_last_emitted_target_{};
    R2SwingDecompSnapshot last_r2_swing_decomp_{};
    int stand_untilt_ticks_{0};
};

/** Nominal stance placement for a given body height and leg geometry. */
std::array<Vec3, kNumLegs> computeNominalStance(const HexapodGeometry& geometry, double body_height_m);

namespace body_controller_detail {

/** Internal/test seam for the height-hold integrator update. */
double updateBodyHeightHoldIntegralM(double current_integral_m,
                                     double commanded_body_height_m,
                                     bool has_measured_body_height,
                                     double measured_body_height_m);

} // namespace body_controller_detail
