#pragma once

#include "types.hpp"

#include <algorithm>
#include <array>
#include <cmath>

struct ServoDynamicsClampResult {
    JointTargets targets{};
    std::array<bool, kNumLegs> leg_limited{};
};

inline ServoDynamicsClampResult clampJointTargetsToServoDynamics(const JointTargets& previous,
                                                                const JointTargets& requested,
                                                                const HexapodGeometry& geometry,
                                                                const double dt_s) {
    ServoDynamicsClampResult result{};
    result.targets = requested;
    if (dt_s <= 0.0) {
        return result;
    }

    for (int leg = 0; leg < kNumLegs; ++leg) {
        const LegGeometry& leg_geometry = geometry.legGeometry[leg];
        bool limited = false;
        for (int joint = 0; joint < kJointsPerLeg; ++joint) {
            const AngleRad prev = previous.leg_states[leg].joint_state[joint].pos_rad;
            const AngleRad req = requested.leg_states[leg].joint_state[joint].pos_rad;
            const double error = req.value - prev.value;
            const ServoJointDynamics& dynamics = leg_geometry.servoDynamics[joint];
            const ServoDirectionDynamics& direction =
                (error >= 0.0) ? dynamics.positive_direction : dynamics.negative_direction;
            const double max_delta = std::max(direction.vmax_radps, 0.0) * dt_s;
            double limited_error = error;
            if (max_delta > 0.0) {
                limited_error = std::clamp(error, -max_delta, max_delta);
            }
            if (std::abs(limited_error - error) > 1e-12) {
                limited = true;
            }

            result.targets.leg_states[leg].joint_state[joint].pos_rad =
                AngleRad{prev.value + limited_error};
            result.targets.leg_states[leg].joint_state[joint].vel_radps =
                AngularRateRadPerSec{limited_error / dt_s};
        }
        result.leg_limited[static_cast<std::size_t>(leg)] = limited;
    }
    return result;
}
