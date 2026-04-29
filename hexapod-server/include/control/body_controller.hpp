#pragma once

#include "control_config.hpp"
#include "geometry_config.hpp"
#include "local_map.hpp"
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
                      const LocalMapSnapshot* terrain_snapshot = nullptr);

private:
    std::array<Vec3, kNumLegs> nominalStance(double body_height_m) const;

    HexapodGeometry geometry_{defaultHexapodGeometry()};
    double foot_estimator_blend_{control_config::kDefaultFootEstimatorBlend};
    control_config::FootTerrainConfig foot_terrain_cfg_{};
};

/** Nominal stance placement for a given body height and leg geometry. */
std::array<Vec3, kNumLegs> computeNominalStance(const HexapodGeometry& geometry, double body_height_m);
