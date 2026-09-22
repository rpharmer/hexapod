#pragma once

#include "types.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <string>

struct ServoDynamicsClampResult {
    JointTargets targets{};
    std::array<bool, kNumLegs> leg_limited{};
};

/**
 * Refresh descriptive target rates after all angle-only safety interventions.
 * This is not another slew clamp: keep every final angle exactly as selected,
 * including a safety snap, and report its actual discrete command derivative.
 * Use the same unwrapped servo coordinates as the reference slew limiter;
 * wrapping here would hide a discontinuous command. Invalid inputs leave the
 * complete target untouched and return false.
 */
inline bool refreshJointTargetVelocities(const JointTargets& previous,
                                        JointTargets& emitted,
                                        const double dt_s) {
    if (!std::isfinite(dt_s) || dt_s <= 0.0) return false;
    JointTargets updated = emitted;
    for (int leg = 0; leg < kNumLegs; ++leg) {
        for (int joint = 0; joint < kJointsPerLeg; ++joint) {
            const double before = previous.leg_states[leg].joint_state[joint].pos_rad.value;
            const double after = emitted.leg_states[leg].joint_state[joint].pos_rad.value;
            const double rate = (after - before) / dt_s;
            if (!std::isfinite(before) || !std::isfinite(after) || !std::isfinite(rate)) return false;
            updated.leg_states[leg].joint_state[joint].vel_radps = AngularRateRadPerSec{rate};
        }
    }
    emitted = updated;
    return true;
}

/**
 * Modes that actively position the legs must keep successive targets inside
 * the configured actuator-rate envelope.  In particular, STAND is included so
 * a recovery from WALK cannot bypass the envelope on its first sample.
 */
inline constexpr bool servoDynamicsClampApplies(const RobotMode mode) {
    return mode == RobotMode::WALK || mode == RobotMode::STAND;
}

/**
 * Successive-command slew does not bound PD error once the plant lags. The
 * post-STAND-untilt abort had a 2.98 rad swing-tibia error and ABA-over at
 * 10 rad/s; healthy sequential peaks sit near 1.3 rad. Cap remainder error so
 * the command cannot sit on the far side of the circle. Not a 10 rad/s×dt
 * clip (that starves walking PD) and not a `vNew` clamp.
 */
inline constexpr double kMaxJointTrackingErrorRad = 1.5;

inline ServoDynamicsClampResult clampJointTargetsTowardMeasured(
    const JointTargets& requested,
    const RobotState& est,
    const double max_error_rad = kMaxJointTrackingErrorRad) {
    ServoDynamicsClampResult result{};
    result.targets = requested;
    if (!(max_error_rad > 0.0)) {
        return result;
    }

    constexpr double kTwoPi = 6.28318530717958647692;
    for (int leg = 0; leg < kNumLegs; ++leg) {
        if (!est.joint_state_quality[static_cast<std::size_t>(leg)].position_valid) {
            continue;
        }
        bool limited = false;
        for (int joint = 0; joint < kJointsPerLeg; ++joint) {
            const double live = est.leg_states[static_cast<std::size_t>(leg)].joint_state[joint].pos_rad.value;
            const double req = requested.leg_states[static_cast<std::size_t>(leg)].joint_state[joint].pos_rad.value;
            if (!std::isfinite(live) || !std::isfinite(req)) {
                continue;
            }
            const double error = std::remainder(req - live, kTwoPi);
            double limited_error = error;
            if (std::abs(error) > max_error_rad) {
                limited_error = std::copysign(max_error_rad, error);
                limited = true;
            }
            result.targets.leg_states[static_cast<std::size_t>(leg)].joint_state[joint].pos_rad =
                AngleRad{live + limited_error};
        }
        result.leg_limited[static_cast<std::size_t>(leg)] = limited;
    }
    return result;
}

/**
 * A servo commanded at its no-load rate has no torque left: available torque is
 * `stall · (1 − ω / ω_noload)`, which is zero at `ω_noload`. Leftover §3.15
 * measured the walking reference pinned at 100% of no-load for 22-35% of
 * samples while 30-64% of planned swing stayed on the ground, and §3.16 showed
 * stance needs 0.435 N·m of the 1.471 N·m stall on three feet. Capping the
 * walking reference at a fraction of no-load reserves `stall · (1 − fraction)`
 * for tracking, so the commanded gait is inside the torque-speed envelope
 * instead of on its far edge.
 *
 * `HEXAPOD_WALK_SLEW_FRACTION` selects the fraction while this is screened;
 * an unset or invalid value keeps the previous 100% command.
 */
inline double walkSlewNoLoadFraction() {
    static const double fraction = [] {
        const char* value = std::getenv("HEXAPOD_WALK_SLEW_FRACTION");
        if (value == nullptr || value[0] == '\0') {
            return 1.0;
        }
        char* end = nullptr;
        const double parsed = std::strtod(value, &end);
        if (end == value || *end != '\0' || !std::isfinite(parsed) || parsed <= 0.0
            || parsed > 1.0) {
            return 1.0;
        }
        return parsed;
    }();
    return fraction;
}

/**
 * The near-cap swing snap assigns a planned-swing leg's targets straight to the
 * measured angles, and it runs *after* the actuator-rate clamp, so it can emit a
 * step the servo cannot execute (measured peak commanded rate up to 6.27× no-load,
 * leftover §3.17). Its firing also tracks the abort count. This gate screens
 * whether it earns its keep. Default on, i.e. production behaviour.
 */
inline bool nearCapSnapEnabled() {
    static const bool enabled = [] {
        const char* value = std::getenv("HEXAPOD_NEAR_CAP_SNAP");
        return value == nullptr || value[0] == '\0' || std::string{value} != "0";
    }();
    return enabled;
}

/**
 * Re-applying the actuator-rate clamp after every target-modifying stage makes
 * "the emitted command never steps further than the servo envelope allows in one
 * sample" an invariant of the whole pipeline rather than of one stage. Without
 * it, the near-cap snap and the remainder cap can both bypass the envelope.
 * Default off while screened.
 */
inline bool finalSlewClampEnabled() {
    static const bool enabled = [] {
        const char* value = std::getenv("HEXAPOD_WALK_FINAL_SLEW_CLAMP");
        return value != nullptr && value[0] != '\0' && std::string{value} != "0";
    }();
    return enabled;
}

inline ServoDynamicsClampResult clampJointTargetsToServoDynamics(const JointTargets& previous,
                                                                const JointTargets& requested,
                                                                const HexapodGeometry& geometry,
                                                                const double dt_s,
                                                                const double rate_scale = 1.0) {
    ServoDynamicsClampResult result{};
    result.targets = requested;
    if (dt_s <= 0.0) {
        return result;
    }
    const double scale = std::clamp(rate_scale, 0.0, 1.0);

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
            const double max_delta = std::max(direction.vmax_radps, 0.0) * scale * dt_s;
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
