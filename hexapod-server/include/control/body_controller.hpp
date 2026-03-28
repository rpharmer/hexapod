#pragma once

#include "control_config.hpp"
#include "foothold_planner.hpp"
#include "gait_policy_planner.hpp"
#include "geometry_config.hpp"
#include "types.hpp"

class BodyController {
public:
    struct MotionLimiterTelemetry {
        bool enabled{true};
        bool hard_clamp_reach{false};
    };

    explicit BodyController(control_config::MotionLimiterConfig config = {});

    LegTargets update(const RobotState& est,
                      const MotionIntent& intent,
                      const GaitState& gait,
                      const SafetyState& safety);

    LegTargets update(const RobotState& est,
                      const MotionIntent& intent,
                      const GaitState& gait,
                      const RuntimeGaitPolicy& policy,
                      const SafetyState& safety);

    void setYawCommandSlewEnabled(bool enabled);

    const MotionLimiterTelemetry& lastMotionLimiterTelemetry() const {
        return last_motion_limiter_telemetry_;
    }

private:
    struct LevelHoldFilterState {
        bool initialized{false};
        std::array<double, 2> roll_pitch_rad{};
    };

    std::array<Vec3, kNumLegs> nominalStance() const;
    Vec3 clampToReachEnvelope(int leg, const Vec3& target_body) const;

    HexapodGeometry geometry_{defaultHexapodGeometry()};
    FootholdPlanner foothold_planner_{geometry_};
    control_config::MotionLimiterConfig limiter_config_{};
    MotionLimiterTelemetry last_motion_limiter_telemetry_{};
    LevelHoldFilterState level_hold_filter_state_{};
};
