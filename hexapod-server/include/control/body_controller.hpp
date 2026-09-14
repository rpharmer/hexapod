#pragma once

#include "control_config.hpp"
#include "geometry_config.hpp"
#include "local_map.hpp"
#include "locomotion_feasibility.hpp"
#include "twist_field.hpp"
#include "types.hpp"

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

private:
    std::array<Vec3, kNumLegs> nominalStance(double body_height_m) const;

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
