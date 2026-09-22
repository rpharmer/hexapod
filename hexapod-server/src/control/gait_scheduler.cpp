#include "gait_scheduler.hpp"

#include "gait_params.hpp"
#include "motion_intent_utils.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <string>

namespace {

UnifiedGaitDescription walkEntryStance(const UnifiedGaitDescription& target) {
    // Start the gait blend from a stationary, all-stance description.  In particular, do not
    // start a tripod at phase 0.5: BodyController interprets that as mid-swing and generates a
    // discontinuous foot target immediately after STAND.
    UnifiedGaitDescription out = target;
    out.duty_factor = 0.94;
    out.phase_offset.fill(0.0);
    out.step_length_m = 0.0;
    out.swing_height_m = 0.0;
    const double hz = std::max(out.step_frequency_hz, 1e-6);
    out.stance_duration_s = out.duty_factor / hz;
    out.swing_duration_s = (1.0 - out.duty_factor) / hz;
    return out;
}

// Ordered replay bursts are 0.36 s. Starting Φ at 0 with ~0.9 Hz cadence never lets
// Group A cross duty, so only the 2-left/1-right tripod swings and the body crabs.
// Seed Φ far enough that both groups appear at the adaptive cadence, without a 2 Hz
// floor that starved the stance stroke. BodyController plants a new stance spell at
// φ=0 so this seed does not jump the feet to a mid-stroke target.
constexpr double kFirstStridePhaseSeed = 0.35;

/**
 * Load-aware phase hold. Leftover §3.15 measured planned-swing feet still in
 * contact for 30-64% of their swing and realised support of 4.2-4.5 feet
 * against 3.0 planned: cadence keeps advancing while the plant has not
 * executed the previous liftoff, so the next stance is scheduled onto a leg
 * that never left the ground. Slowing the stride integrator while a scheduled
 * swing is still loaded makes cadence a function of what the plant achieved.
 *
 * Early swing legitimately still touches, so the grace fraction ignores the
 * first quarter of swing. The hold reuses the governor's existing 0.25 cadence
 * floor rather than freezing, and it is budgeted to one swing duration so a
 * permanently loaded foot cannot deadlock the gait.
 */
constexpr double kLoadPhaseGraceFraction = 0.25;
constexpr double kLoadPhaseHoldScale = 0.25;
constexpr double kLoadPhaseMaxHoldFraction = 1.0;

bool loadAwarePhaseEnabled() {
    static const bool enabled = [] {
        const char* value = std::getenv("HEXAPOD_WALK_LOAD_PHASE");
        return value != nullptr && value[0] != '\0' && std::string{value} != "0";
    }();
    return enabled;
}

} // namespace

GaitScheduler::GaitScheduler(control_config::GaitConfig config)
    : config_(config) {}

double GaitScheduler::wrap01(const double x) const {
    return gaitWrap01(x);
}

void GaitScheduler::reset() {
    phase_accum_ = 0.0;
    last_update_us_ = TimePointUs{};
    committed_gait_ = GaitType::TRIPOD;
    committed_initialized_ = false;
    transition_from_snap_ = UnifiedGaitDescription{};
    last_blended_ = UnifiedGaitDescription{};
    transition_start_us_ = TimePointUs{};
    have_last_blended_ = false;
    walk_entry_blend_ = false;
    was_walking_ = false;
    last_cmd_vx_mps_ = 0.0;
    last_cmd_vy_mps_ = 0.0;
}

void GaitScheduler::debugRestore(const GaitState& gait) {
    phase_accum_ = gait.phase.empty() ? 0.0 : gait.phase[0];
    last_update_us_ = gait.timestamp_us;
    committed_gait_ = GaitType::TRIPOD;
    committed_initialized_ = true;
    have_last_blended_ = false;
    walk_entry_blend_ = false;
    was_walking_ = true;
}

GaitState GaitScheduler::preview(const RobotState& est,
                                 const MotionIntent& intent,
                                 const SafetyState& safety,
                                 const BodyTwist& cmd_twist,
                                 const CommandGovernorState& governor) {
    return compute(est, intent, safety, cmd_twist, governor, false);
}

GaitState GaitScheduler::update(const RobotState& est,
                                const MotionIntent& intent,
                                const SafetyState& safety,
                                const BodyTwist& cmd_twist,
                                const CommandGovernorState& governor) {
    return compute(est, intent, safety, cmd_twist, governor, true);
}

GaitState GaitScheduler::compute(const RobotState& est,
                                 const MotionIntent& intent,
                                 const SafetyState& safety,
                                 const BodyTwist& cmd_twist,
                                 const CommandGovernorState& governor,
                                 const bool commit_state) {
    GaitState out{};
    const TimePointUs now = intent.timestamp_us.isZero() ? now_us() : intent.timestamp_us;
    out.timestamp_us = now;

    double phase_accum = phase_accum_;
    double drag_hold_s = drag_hold_s_;
    TimePointUs last_update_us = last_update_us_;
    GaitType committed_gait = committed_gait_;
    bool committed_initialized = committed_initialized_;
    UnifiedGaitDescription transition_from_snap = transition_from_snap_;
    UnifiedGaitDescription last_blended = last_blended_;
    TimePointUs transition_start_us = transition_start_us_;
    bool have_last_blended = have_last_blended_;
    bool walk_entry_blend = walk_entry_blend_;
    bool was_walking = was_walking_;
    double last_cmd_vx_mps = last_cmd_vx_mps_;
    double last_cmd_vy_mps = last_cmd_vy_mps_;

    const bool walking =
        (intent.requested_mode == RobotMode::WALK) &&
        !safety.inhibit_motion &&
        !safety.torque_cut;

    if (!walking) {
        was_walking = false;
        drag_hold_s = 0.0;
        last_cmd_vx_mps = cmd_twist.linear_mps.x;
        last_cmd_vy_mps = cmd_twist.linear_mps.y;
        for (int i = 0; i < kNumLegs; ++i) {
            out.phase[i] = 0.0;
            out.in_stance[i] = true;
            out.phase_offset[static_cast<std::size_t>(i)] = 0.0;
            out.stability_hold_stance[static_cast<std::size_t>(i)] = false;
            out.support_liftoff_clearance_m[static_cast<std::size_t>(i)] = 0.0;
            out.support_liftoff_safe_to_lift[static_cast<std::size_t>(i)] = false;
        }
        out.duty_factor = 0.5;
        out.step_length_m = 0.06;
        out.swing_height_m = 0.03;
        out.stance_duration_s = 0.5;
        out.swing_duration_s = 0.5;
        out.stride_phase_rate_hz = FrequencyHz{1.0};
        out.swing_time_ease_01 = 1.0;
        out.static_stability_margin_m = 0.0;
        out.cmd_accel_body_x_mps2 = 0.0;
        out.cmd_accel_body_y_mps2 = 0.0;
        if (commit_state) {
            last_cmd_vx_mps_ = last_cmd_vx_mps;
            last_cmd_vy_mps_ = last_cmd_vy_mps;
            was_walking_ = was_walking;
            drag_hold_s_ = drag_hold_s;
            last_update_us_ = out.timestamp_us;
        }
        return out;
    }

    if (last_update_us.isZero()) {
        last_update_us = now;
    }

    const DurationSec dt{static_cast<double>((now - last_update_us).value) * 1e-6};
    last_update_us = now;

    const PlanarMotionCommand cmd = planarMotionFromCommandTwist(cmd_twist);
    double cmd_ax = 0.0;
    double cmd_ay = 0.0;
    if (dt.value > 1e-9) {
        const double inv_dt = 1.0 / dt.value;
        cmd_ax = std::clamp((cmd.vx_mps - last_cmd_vx_mps) * inv_dt, -8.0, 8.0);
        cmd_ay = std::clamp((cmd.vy_mps - last_cmd_vy_mps) * inv_dt, -8.0, 8.0);
    }
    last_cmd_vx_mps = cmd.vx_mps;
    last_cmd_vy_mps = cmd.vy_mps;

    UnifiedGaitDescription target{};
    if (intent.gait == GaitType::TRIPOD) {
        target = buildAdaptiveTripodCrawlGait(cmd.vx_mps, cmd.vy_mps, cmd.yaw_rate_radps, cmd_ax, cmd_ay, config_);
    } else if (intent.gait == GaitType::RIPPLE) {
        target = buildAdaptiveRippleCrawlGait(cmd.vx_mps, cmd.vy_mps, cmd.yaw_rate_radps, cmd_ax, cmd_ay, config_);
    } else if (intent.gait == GaitType::WAVE) {
        target = buildAdaptiveWaveCrawlGait(cmd.vx_mps, cmd.vy_mps, cmd.yaw_rate_radps, cmd_ax, cmd_ay, config_);
    } else {
        target = buildTargetUnifiedGait(
            intent.gait, cmd.vx_mps, cmd.vy_mps, cmd.yaw_rate_radps, config_, cmd_ax, cmd_ay);
    }

    const bool walk_entry = !was_walking;
    if (walk_entry) {
        // The scheduler is dormant in STAND.  Blend into WALK from a common all-stance phase,
        // just as we blend when changing gait types, to preserve foot-target continuity.
        // Planar bursts seed Φ so both tripods appear in 0.36 s. Yaw-dominant in-place
        // turns keep Φ=0. Use the intent command, not the slewed cmd_twist: loco-cmd
        // ramps yaw from 0, so the first STAND→WALK frame would otherwise look planar
        // and eat the counter-yaw stroke.
        const double intent_planar_mps =
            std::hypot(intent.cmd_vx_mps.value, intent.cmd_vy_mps.value);
        const double yaw_equiv_mps = std::abs(intent.cmd_yaw_radps.value) * 0.11;
        const bool yaw_dominant = yaw_equiv_mps > intent_planar_mps + 1e-6;
        phase_accum = yaw_dominant ? 0.0 : kFirstStridePhaseSeed;
        transition_from_snap = walkEntryStance(target);
        transition_start_us = now;
        have_last_blended = true;
        walk_entry_blend = true;
    } else if (committed_initialized && intent.gait != committed_gait) {
        transition_from_snap = have_last_blended ? last_blended : target;
        transition_start_us = now;
        walk_entry_blend = false;
    }
    if (!committed_initialized && !walk_entry) {
        transition_from_snap = target;
    }
    committed_gait = intent.gait;
    committed_initialized = true;
    was_walking = true;

    double alpha = 1.0;
    if (!transition_start_us.isZero()) {
        const double elapsed_s = static_cast<double>((now - transition_start_us).value) * 1e-6;
        const double blend_s = walk_entry_blend
            ? config_.walk_entry_blend_s
            : config_.transition_blend_s;
        alpha = std::clamp(elapsed_s / std::max(blend_s, 1e-4), 0.0, 1.0);
    }

    const UnifiedGaitDescription blended =
        (alpha >= 1.0 - 1e-9) ? target : blendUnifiedGait(transition_from_snap, target, alpha);
    if (alpha >= 1.0 - 1e-9) {
        transition_start_us = {};
    }

    last_blended = blended;
    have_last_blended = true;

    const double governor_cadence_scale =
        governor.freeze_phase ? 0.0 : std::clamp(governor.cadence_scale, 0.25, 1.0);
    const double governor_swing_floor_m = std::max(0.0, governor.swing_height_floor_m);
    const double step_hz =
        governor.freeze_phase ? 0.0 : std::max(blended.step_frequency_hz * governor_cadence_scale, 1e-6);
    out.stride_phase_rate_hz = FrequencyHz{step_hz};
    out.duty_factor = blended.duty_factor;
    out.step_length_m = blended.step_length_m;
    out.swing_height_m = std::max(blended.swing_height_m, governor_swing_floor_m);
    out.swing_time_ease_01 = blended.swing_time_ease;
    out.stance_duration_s = governor.freeze_phase ? 0.0 : out.duty_factor / step_hz;
    out.swing_duration_s = governor.freeze_phase ? 0.0 : (1.0 - out.duty_factor) / step_hz;
    out.phase_offset = blended.phase_offset;

    // Hold the stride integrator while a scheduled swing is still carrying load,
    // so the next stance is not scheduled onto a foot that never lifted.
    double load_phase_scale = 1.0;
    if (loadAwarePhaseEnabled() && !governor.freeze_phase) {
        const double duty = std::clamp(blended.duty_factor, 0.0, 1.0);
        const double swing_span = std::max(1.0 - duty, 1e-6);
        bool loaded_swing = false;
        for (int leg = 0; leg < kNumLegs; ++leg) {
            const std::size_t leg_index = static_cast<std::size_t>(leg);
            const double p = wrap01(phase_accum + blended.phase_offset[leg_index]);
            if (p < duty) {
                continue;
            }
            const double swing_progress = (p - duty) / swing_span;
            if (swing_progress > kLoadPhaseGraceFraction && est.foot_contacts[leg_index]) {
                loaded_swing = true;
                break;
            }
        }
        const double budget_s = kLoadPhaseMaxHoldFraction * out.swing_duration_s;
        if (!loaded_swing) {
            drag_hold_s = 0.0;
        } else if (drag_hold_s < budget_s) {
            load_phase_scale = kLoadPhaseHoldScale;
            drag_hold_s += dt.value;
        }
    }

    if (!governor.freeze_phase) {
        phase_accum = wrap01(phase_accum + dt.value * step_hz * load_phase_scale);
    }

    for (int leg = 0; leg < kNumLegs; ++leg) {
        const double off = blended.phase_offset[static_cast<std::size_t>(leg)];
        const double p = wrap01(phase_accum + off);
        out.phase[leg] = p;
        out.in_stance[leg] = (p < blended.duty_factor);
        out.stability_hold_stance[static_cast<std::size_t>(leg)] = false;
        out.support_liftoff_clearance_m[static_cast<std::size_t>(leg)] = 0.0;
        out.support_liftoff_safe_to_lift[static_cast<std::size_t>(leg)] = false;
    }
    out.static_stability_margin_m = 0.0;
    out.cmd_accel_body_x_mps2 = cmd_ax;
    out.cmd_accel_body_y_mps2 = cmd_ay;

    if (commit_state) {
        phase_accum_ = phase_accum;
        drag_hold_s_ = drag_hold_s;
        last_update_us_ = last_update_us;
        committed_gait_ = committed_gait;
        committed_initialized_ = committed_initialized;
        transition_from_snap_ = transition_from_snap;
        last_blended_ = last_blended;
        transition_start_us_ = transition_start_us;
        have_last_blended_ = have_last_blended;
        walk_entry_blend_ = walk_entry_blend;
        was_walking_ = was_walking;
        last_cmd_vx_mps_ = last_cmd_vx_mps;
        last_cmd_vy_mps_ = last_cmd_vy_mps;
    }

    return out;
}
