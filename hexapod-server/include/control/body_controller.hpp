#pragma once

#include "foothold_planner.hpp"
#include "gait_policy_planner.hpp"
#include "geometry_config.hpp"
#include "types.hpp"

class BodyController {
public:
    struct MotionLimiterTelemetry {
        bool enabled{true};
        uint8_t phase{0};
        uint8_t constraint_reason{0};
        double adaptation_scale_linear{1.0};
        double adaptation_scale_yaw{1.0};
        bool hard_clamp_linear{false};
        bool hard_clamp_yaw{false};
        bool hard_clamp_reach{false};
    };

    LegTargets update(const RobotState& est,
                      const MotionIntent& intent,
                      const GaitState& gait,
                      const SafetyState& safety);

    LegTargets update(const RobotState& est,
                      const MotionIntent& intent,
                      const GaitState& gait,
                      const RuntimeGaitPolicy& policy,
                      const SafetyState& safety);

    const MotionLimiterTelemetry& lastMotionLimiterTelemetry() const {
        return last_motion_limiter_telemetry_;
    }

private:
    std::array<Vec3, kNumLegs> nominalStance() const;
    Vec3 clampToReachEnvelope(int leg, const Vec3& target_body) const;

    HexapodGeometry geometry_{defaultHexapodGeometry()};
    FootholdPlanner foothold_planner_{geometry_};
    double filtered_yaw_cmd_rad_{0.0};
    bool yaw_filter_initialized_{false};
    TimePointUs last_update_timestamp_{};
    bool has_previous_targets_{false};
    LegTargets previous_targets_{};
    bool previous_walking_{false};
    uint32_t transition_slew_steps_remaining_{0};
    MotionLimiterTelemetry last_motion_limiter_telemetry_{};
};
