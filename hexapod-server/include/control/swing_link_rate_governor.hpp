#pragma once

#include "leg_link_angular_velocity.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

enum class SwingLinkRateStatus { Inactive, Unavailable, Infeasible, Unchanged, Limited };

struct SwingLinkRateDiagnostics {
    SwingLinkRateStatus status{SwingLinkRateStatus::Inactive};
    double scale{1.0};
    int limiting_link{-1};
    std::array<double, kJointsPerLeg> requested_rates{};
    std::array<Vec3, kJointsPerLeg> predicted_before{};
    std::array<Vec3, kJointsPerLeg> predicted_after{};
};

struct SwingLinkRateResult {
    JointTargets targets{};
    std::array<SwingLinkRateDiagnostics, kNumLegs> legs{};
};

inline bool finiteLinkRateVector(const Vec3& value) {
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

// Maximize s along the requested direction, not over independently clipped
// joint rates. With the base inside the ball, each quadratic feasible interval
// contains zero; intersecting their upper bounds is exact and deterministic.
inline SwingLinkRateDiagnostics projectSwingLinkRates(
    const LegGeometry& geometry, const LegState& measured,
    const std::array<double, kJointsPerLeg>& rates,
    const Vec3& body_angular_rate, const double budget) {
    SwingLinkRateDiagnostics out{};
    out.requested_rates = rates;
    out.status = SwingLinkRateStatus::Unavailable;
    if (!std::isfinite(budget) || budget <= 0.0 || !finiteLinkRateVector(body_angular_rate)
        || !std::isfinite(geometry.mountAngle.value)
        || !std::isfinite(geometry.servo.coxaSign) || !std::isfinite(geometry.servo.femurSign)
        || !std::isfinite(geometry.servo.tibiaSign)
        || !std::isfinite(geometry.servo.coxaOffset.value)
        || !std::isfinite(geometry.servo.femurOffset.value)
        || !std::isfinite(geometry.servo.tibiaOffset.value)) return out;
    for (int joint = 0; joint < kJointsPerLeg; ++joint) {
        if (!std::isfinite(rates[joint]) || !std::isfinite(measured.joint_state[joint].pos_rad.value)) return out;
    }
    const auto relative = legRelativeLinkAngularVelocities(geometry, measured, rates);
    for (int link = 0; link < kJointsPerLeg; ++link) {
        out.predicted_before[link] = body_angular_rate + relative[link];
        out.predicted_after[link] = out.predicted_before[link];
        if (!finiteLinkRateVector(relative[link]) || !finiteLinkRateVector(out.predicted_before[link])) return out;
    }
    const auto dot = [](const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; };
    const double base_norm = vecNorm(body_angular_rate);
    if (!std::isfinite(base_norm)) return out;
    if (base_norm > budget) { out.status = SwingLinkRateStatus::Infeasible; return out; }
    // Factored difference avoids cancellation near the boundary.
    const double slack = (budget - base_norm) * (budget + base_norm);
    for (int link = 0; link < kJointsPerLeg; ++link) {
        const double a = dot(relative[link], relative[link]);
        const double b = 2.0 * dot(body_angular_rate, relative[link]);
        if (!std::isfinite(a) || !std::isfinite(b)) return out;
        if (a == 0.0) continue;
        const double root = std::hypot(b, 2.0 * std::sqrt(a) * std::sqrt(slack));
        if (!std::isfinite(root)) return out;
        const double upper = b >= 0.0
            ? ((root + b) == 0.0 ? 0.0 : 2.0 * slack / (root + b))
            : (root - b) / (2.0 * a);
        const double scale = std::clamp(upper, 0.0, 1.0);
        if (scale < out.scale) { out.scale = scale; out.limiting_link = link; }
    }
    out.status = out.scale < 1.0 ? SwingLinkRateStatus::Limited : SwingLinkRateStatus::Unchanged;
    for (int link = 0; link < kJointsPerLeg; ++link) out.predicted_after[link] = body_angular_rate + relative[link] * out.scale;
    return out;
}

// PhysicsSimBridge-convention experiment: caller verifies the bridge capability;
// simulated provenance additionally prevents estimator/hardware activation.
// No target snap. Inputs must already be clamped to the motor slew envelope.
inline SwingLinkRateResult governSwingLinkRates(
    const JointTargets& previous, const JointTargets& slew_limited,
    const HexapodGeometry& geometry, const RobotState& measured,
    const RobotMode mode, const std::array<bool, kNumLegs>& planned_stance,
    const double dt_s, const bool enabled, const double budget = 10.0) {
    SwingLinkRateResult out{};
    out.targets = slew_limited;
    if (!enabled || mode != RobotMode::WALK) return out;
    for (int leg = 0; leg < kNumLegs; ++leg) {
        if (planned_stance[leg]) continue;
        auto& diagnostic = out.legs[leg];
        diagnostic.status = SwingLinkRateStatus::Unavailable;
        if (!std::isfinite(dt_s) || dt_s <= 0.0 || !measured.bus_ok || !measured.has_body_twist_state
            || !measured.joint_state_quality[leg].position_valid
            || measured.joint_state_quality[leg].source != JointStateSource::Simulated) continue;
        std::array<double, kJointsPerLeg> rates{};
        bool finite = true;
        for (int joint = 0; joint < kJointsPerLeg; ++joint) {
            const double from = previous.leg_states[leg].joint_state[joint].pos_rad.value;
            const double to = slew_limited.leg_states[leg].joint_state[joint].pos_rad.value;
            finite = finite && std::isfinite(from) && std::isfinite(to);
            rates[joint] = (to - from) / dt_s;
        }
        if (!finite) continue;
        diagnostic = projectSwingLinkRates(geometry.legGeometry[leg], measured.leg_states[leg], rates,
                                          measured.body_twist_state.twist_vel_radps.raw(), budget);
        if (diagnostic.status != SwingLinkRateStatus::Limited) continue;
        for (int joint = 0; joint < kJointsPerLeg; ++joint) {
            auto& target = out.targets.leg_states[leg].joint_state[joint];
            const double from = previous.leg_states[leg].joint_state[joint].pos_rad.value;
            target.pos_rad = AngleRad{from + diagnostic.scale * (target.pos_rad.value - from)};
            target.vel_radps = AngularRateRadPerSec{diagnostic.scale * rates[joint]};
        }
    }
    return out;
}

/**
 * Production near-cap latch for the after-1.5 femur ABA-over dump
 * (`sl-abort-after-1p5-v1`: incoming 9.996, free 10.210, PD 0.71).
 * Holding the successive-command increment does not cut PD torque; snap that
 * swing leg to the live angle so ABA is not still driven into the guard.
 * Engage only when measured link ω is already within the captured ABA
 * overshoot of 10 rad/s. Healthy walking at 7–8 rad/s is unchanged. Not the
 * always-on swing-link-rate governor. Gait `in_stance` skip is required:
 * measured-unload and gait-stance/plant-unload coupling snaps
 * (`sl-abort-near-cap-v1`) both regressed sequential.
 *
 * `sl-abort-near-cap-v1` tibia (wire 17) is coupling: requested-at-q 1.89,
 * PD 0.63, zeroing that τ alone lands `speed_free` under 10. Cap tibia
 * remainder only; do not freeze coxa/femur.
 */
inline constexpr double kSpeedGuardLinkAngularRadps = 10.0;
inline constexpr double kCapturedSwingFemurAbaOvershootRadps = 0.214;
inline constexpr double kSwingLinkNearCapRadps =
    kSpeedGuardLinkAngularRadps - kCapturedSwingFemurAbaOvershootRadps;
inline constexpr double kNearCapTibiaTrackingErrorRad = 0.25;

inline double swingLinkPeakRadps(const LegGeometry& geometry,
                                const LegState& configuration,
                                const std::array<double, kJointsPerLeg>& joint_rates,
                                const Vec3& body_angular_rate) {
    for (int joint = 0; joint < kJointsPerLeg; ++joint) {
        if (!std::isfinite(joint_rates[static_cast<std::size_t>(joint)])) {
            return std::numeric_limits<double>::quiet_NaN();
        }
    }
    const auto relative = legRelativeLinkAngularVelocities(geometry, configuration, joint_rates);
    double peak = vecNorm(body_angular_rate);
    for (int link = 0; link < kJointsPerLeg; ++link) {
        if (!finiteLinkRateVector(relative[static_cast<std::size_t>(link)])) {
            return std::numeric_limits<double>::quiet_NaN();
        }
        peak = std::max(peak, vecNorm(body_angular_rate + relative[static_cast<std::size_t>(link)]));
    }
    return peak;
}

inline double measuredSwingLinkPeakRadps(const LegGeometry& geometry,
                                        const LegState& measured,
                                        const Vec3& body_angular_rate) {
    std::array<double, kJointsPerLeg> live_rates{};
    for (int joint = 0; joint < kJointsPerLeg; ++joint) {
        live_rates[static_cast<std::size_t>(joint)] =
            measured.joint_state[static_cast<std::size_t>(joint)].vel_radps.value;
    }
    return swingLinkPeakRadps(geometry, measured, live_rates, body_angular_rate);
}

inline SwingLinkRateResult snapSwingTargetsNearMeasuredLinkCap(
    const JointTargets& requested,
    const HexapodGeometry& geometry,
    const RobotState& measured,
    const RobotMode mode,
    const std::array<bool, kNumLegs>& planned_stance,
    const bool enabled) {
    SwingLinkRateResult out{};
    out.targets = requested;
    if (!enabled || mode != RobotMode::WALK) {
        return out;
    }
    for (int leg = 0; leg < kNumLegs; ++leg) {
        auto& diagnostic = out.legs[static_cast<std::size_t>(leg)];
        if (planned_stance[static_cast<std::size_t>(leg)]) {
            continue;
        }
        diagnostic.status = SwingLinkRateStatus::Unavailable;
        if (!measured.bus_ok || !measured.has_body_twist_state
            || !measured.joint_state_quality[static_cast<std::size_t>(leg)].position_valid
            || !measured.joint_state_quality[static_cast<std::size_t>(leg)].velocity_valid
            || measured.joint_state_quality[static_cast<std::size_t>(leg)].source
                   != JointStateSource::Simulated) {
            continue;
        }
        const Vec3 body = measured.body_twist_state.twist_vel_radps.raw();
        const double peak = measuredSwingLinkPeakRadps(
            geometry.legGeometry[static_cast<std::size_t>(leg)],
            measured.leg_states[static_cast<std::size_t>(leg)],
            body);
        if (!std::isfinite(peak)) {
            continue;
        }
        diagnostic.predicted_before[0] = body;
        if (peak < kSwingLinkNearCapRadps) {
            diagnostic.status = SwingLinkRateStatus::Unchanged;
            continue;
        }
        diagnostic.status = SwingLinkRateStatus::Limited;
        diagnostic.scale = 0.0;
        diagnostic.limiting_link = 1;
        for (int joint = 0; joint < kJointsPerLeg; ++joint) {
            out.targets.leg_states[static_cast<std::size_t>(leg)].joint_state[static_cast<std::size_t>(joint)].pos_rad =
                measured.leg_states[static_cast<std::size_t>(leg)].joint_state[static_cast<std::size_t>(joint)].pos_rad;
        }
    }
    return out;
}

/**
 * Contact-aware liftoff bound.
 *
 * Measured on `support-divergence-default-straight-v1`: legs 2/4/5 keep ground
 * contact 155-250 ms past planned liftoff because the foot lags its command by
 * 41-79 mm while the commanded swing clearance is only 24-50 mm. A blocked leg
 * accumulates PD position error the floor will not let it work off; when the
 * contact finally breaks the stored error discharges into free space. The
 * winner in that dump went 8.6 -> 12.8 rad/s over six ticks while its tibia
 * error bled 1.031 -> 0.962 rad, and the guard tripped two ticks later. Every
 * frozen first-trip fixture shows the same precondition: `peak_pd_abs_error`
 * 0.63 (`near-cap`), 0.71 (`after-1p5`), 1.08 (`default-straight`), 2.98
 * (`stand-untilt`) rad.
 *
 * The bound is derived from the servo model rather than tuned. A critically
 * damped second-order position loop answers a step `e` with peak velocity
 * `ωn e / e¹ = 0.3679 ωn e`. Allowing three joints of one leg to align, no
 * single loaded-swing leg may store more than
 * `guard / (kJointsPerLeg · 0.3679 ωn)`. At ωn = 25 and a 10 rad/s guard that
 * is 0.362 rad; the same formula returns 1.09 rad for a single joint, which is
 * the 1.08 rad actually observed at the `default-straight` trip.
 *
 * Distinct from the reverted measured-unload snap: this only ever touches a leg
 * gait has already scheduled to leave the ground, so it cannot withdraw support
 * from a leg that is meant to bear load. It bounds stored error instead of
 * freezing the target, so the leg keeps a constant share of lifting torque and
 * the swing still completes.
 */
inline constexpr double kServoCriticalPeakVelocityGain = 0.36787944117144233;
inline constexpr double kLoadedSwingTrackingErrorRad =
    kSpeedGuardLinkAngularRadps
    / (static_cast<double>(kJointsPerLeg) * kServoCriticalPeakVelocityGain
       * hexapod_dynamics::kServoOmegaN);

inline SwingLinkRateResult clampLoadedSwingTargetsTowardMeasured(
    const JointTargets& requested,
    const RobotState& measured,
    const RobotMode mode,
    const std::array<bool, kNumLegs>& planned_stance,
    const std::array<bool, kNumLegs>& load_bearing,
    const bool enabled,
    const double max_error_rad = kLoadedSwingTrackingErrorRad) {
    SwingLinkRateResult out{};
    out.targets = requested;
    if (!enabled || mode != RobotMode::WALK || !(max_error_rad > 0.0)) {
        return out;
    }
    constexpr double kTwoPi = 6.28318530717958647692;
    for (int leg = 0; leg < kNumLegs; ++leg) {
        const std::size_t leg_index = static_cast<std::size_t>(leg);
        auto& diagnostic = out.legs[leg_index];
        if (planned_stance[leg_index] || !load_bearing[leg_index]) {
            continue;
        }
        diagnostic.status = SwingLinkRateStatus::Unavailable;
        if (!measured.joint_state_quality[leg_index].position_valid) {
            continue;
        }
        diagnostic.status = SwingLinkRateStatus::Unchanged;
        for (int joint = 0; joint < kJointsPerLeg; ++joint) {
            const std::size_t joint_index = static_cast<std::size_t>(joint);
            const double live =
                measured.leg_states[leg_index].joint_state[joint_index].pos_rad.value;
            const double req =
                requested.leg_states[leg_index].joint_state[joint_index].pos_rad.value;
            if (!std::isfinite(live) || !std::isfinite(req)) {
                continue;
            }
            const double error = std::remainder(req - live, kTwoPi);
            if (std::abs(error) <= max_error_rad) {
                continue;
            }
            diagnostic.status = SwingLinkRateStatus::Limited;
            diagnostic.scale = std::min(diagnostic.scale, max_error_rad / std::abs(error));
            diagnostic.limiting_link = joint;
            out.targets.leg_states[leg_index].joint_state[joint_index].pos_rad =
                AngleRad{live + std::copysign(max_error_rad, error)};
        }
    }
    return out;
}

/**
 * Dump-named leftover for `sl-abort-near-cap-v1`: winner tibia τ (wire 17)
 * is necessary for dense `speed_free` > 10; femur-16 is not; contact Δω is
 * 0.005. Cap tibia remainder only when measured link ω is already near the
 * guard and the command at current q is itself under that near-cap. Does not
 * use gait `in_stance` (that skip missed the dump). Does not move coxa/femur.
 * Screened in production and **reverted**: isolated reverse returned a 613-hold
 * tibia SpeedLimit cascade. Host tests keep the helper; `robot_runtime` does
 * not call it.
 */
inline SwingLinkRateResult clampNearCapTibiaTowardMeasured(
    const JointTargets& requested,
    const HexapodGeometry& geometry,
    const RobotState& measured,
    const RobotMode mode,
    const bool enabled,
    const double max_tibia_error_rad = kNearCapTibiaTrackingErrorRad) {
    SwingLinkRateResult out{};
    out.targets = requested;
    if (!enabled || mode != RobotMode::WALK || !(max_tibia_error_rad > 0.0)) {
        return out;
    }
    constexpr double kTwoPi = 6.28318530717958647692;
    for (int leg = 0; leg < kNumLegs; ++leg) {
        auto& diagnostic = out.legs[static_cast<std::size_t>(leg)];
        diagnostic.status = SwingLinkRateStatus::Unavailable;
        if (!measured.bus_ok || !measured.has_body_twist_state
            || !measured.joint_state_quality[static_cast<std::size_t>(leg)].position_valid
            || !measured.joint_state_quality[static_cast<std::size_t>(leg)].velocity_valid
            || measured.joint_state_quality[static_cast<std::size_t>(leg)].source
                   != JointStateSource::Simulated) {
            continue;
        }
        const Vec3 body = measured.body_twist_state.twist_vel_radps.raw();
        const double measured_peak = measuredSwingLinkPeakRadps(
            geometry.legGeometry[static_cast<std::size_t>(leg)],
            measured.leg_states[static_cast<std::size_t>(leg)],
            body);
        if (!std::isfinite(measured_peak) || measured_peak < kSwingLinkNearCapRadps) {
            diagnostic.status = SwingLinkRateStatus::Unchanged;
            continue;
        }
        std::array<double, kJointsPerLeg> requested_rates{};
        for (int joint = 0; joint < kJointsPerLeg; ++joint) {
            requested_rates[static_cast<std::size_t>(joint)] =
                requested.leg_states[static_cast<std::size_t>(leg)]
                    .joint_state[static_cast<std::size_t>(joint)]
                    .vel_radps.value;
        }
        const double requested_peak = swingLinkPeakRadps(
            geometry.legGeometry[static_cast<std::size_t>(leg)],
            measured.leg_states[static_cast<std::size_t>(leg)],
            requested_rates,
            body);
        if (!std::isfinite(requested_peak) || requested_peak >= kSwingLinkNearCapRadps) {
            diagnostic.status = SwingLinkRateStatus::Unchanged;
            continue;
        }
        const double live =
            measured.leg_states[static_cast<std::size_t>(leg)].joint_state[TIBIA].pos_rad.value;
        const double req =
            requested.leg_states[static_cast<std::size_t>(leg)].joint_state[TIBIA].pos_rad.value;
        if (!std::isfinite(live) || !std::isfinite(req)) {
            continue;
        }
        const double error = std::remainder(req - live, kTwoPi);
        if (std::abs(error) <= max_tibia_error_rad) {
            diagnostic.status = SwingLinkRateStatus::Unchanged;
            continue;
        }
        diagnostic.status = SwingLinkRateStatus::Limited;
        diagnostic.scale = max_tibia_error_rad / std::abs(error);
        diagnostic.limiting_link = TIBIA;
        out.targets.leg_states[static_cast<std::size_t>(leg)].joint_state[TIBIA].pos_rad =
            AngleRad{live + std::copysign(max_tibia_error_rad, error)};
    }
    return out;
}
