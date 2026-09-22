#include "body_controller.hpp"
#include "geometry_config.hpp"
#include "locomotion_command.hpp"
#include "motion_intent_utils.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {

bool expect(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

bool nearlyEqual(double lhs, double rhs, double eps = 1e-6) {
    return std::abs(lhs - rhs) <= eps;
}

bool standTargetsRemainReachable(const LegTargets& targets) {
    const HexapodGeometry geometry = geometry_config::buildDefaultHexapodGeometry();
    constexpr double kReachEps = 1e-9;
    for (int leg = 0; leg < kNumLegs; ++leg) {
        const LegGeometry& leg_geo = geometry.legGeometry[leg];
        const Vec3 rel_body = targets.feet[leg].pos_body_m - leg_geo.bodyCoxaOffset;
        const Mat3 leg_from_body = legFromBodyFrame(leg_geo);
        const Vec3 rel_leg = leg_from_body * rel_body;
        const double r = std::hypot(rel_leg.x, rel_leg.y);
        const double rho = r - leg_geo.coxaLength.value;
        const double d = std::hypot(rho, rel_leg.z);
        const double max_reach = leg_geo.femurLength.value + leg_geo.tibiaLength.value;
        if (d > max_reach + kReachEps) {
            return false;
        }
    }
    return true;
}

} // namespace

int main() {
    BodyController controller{};
    RobotState est{};
    SafetyState safety{};
    safety.inhibit_motion = false;
    safety.torque_cut = false;

    MotionIntent stand_intent{};
    stand_intent.requested_mode = RobotMode::STAND;
    stand_intent.twist.body_trans_mps = Vec3{0.10, -0.20, 0.05};
    stand_intent.twist.twist_vel_radps = Vec3{0.0, 0.0, 0.3};
    stand_intent.twist.body_trans_m.z = 0.12;

    GaitState gait{};
    const BodyTwist stand_twist =
        rawLocomotionTwistFromIntent(stand_intent, planarMotionCommand(stand_intent));
    const LegTargets stand_targets = controller.update(est, stand_intent, gait, safety, stand_twist);

    for (int leg = 0; leg < kNumLegs; ++leg) {
        const Vec3 expected = (Vec3{-stand_intent.twist.body_trans_mps.x,
                                    -stand_intent.twist.body_trans_mps.y,
                                    -stand_intent.twist.body_trans_mps.z}) +
                              cross(stand_intent.twist.twist_vel_radps, stand_targets.feet[leg].pos_body_m);
        const Vec3 actual = stand_targets.feet[leg].vel_body_mps;
        if (!expect(nearlyEqual(actual.x, expected.x) &&
                        nearlyEqual(actual.y, expected.y) &&
                        nearlyEqual(actual.z, expected.z),
                    "stand mode velocity should include body translation and angular feed-forward")) {
            return EXIT_FAILURE;
        }
    }
    if (!expect(standTargetsRemainReachable(stand_targets),
                "stand mode nominal targets should stay within leg reach")) {
        return EXIT_FAILURE;
    }

    {
        BodyController untilt_controller{};
        RobotState untilt_est{};
        untilt_est.has_body_twist_state = true;
        untilt_est.body_twist_state.body_trans_m.z = 0.14;
        untilt_est.body_twist_state.twist_pos_rad.x = 0.08;
        MotionIntent untilt_intent{};
        untilt_intent.requested_mode = RobotMode::STAND;
        untilt_intent.twist.body_trans_m.z = 0.14;
        GaitState untilt_gait{};
        const BodyTwist untilt_twist =
            rawLocomotionTwistFromIntent(untilt_intent, planarMotionCommand(untilt_intent));
        RobotState level_est = untilt_est;
        level_est.body_twist_state.twist_pos_rad.x = 0.0;
        const LegTargets level_targets =
            untilt_controller.update(level_est, untilt_intent, untilt_gait, safety, untilt_twist);
        BodyController tilted_controller{};
        const LegTargets tilted_targets =
            tilted_controller.update(untilt_est, untilt_intent, untilt_gait, safety, untilt_twist);
        double peak_dz = 0.0;
        int opposite_sign_pairs = 0;
        for (int leg = 0; leg < kNumLegs; ++leg) {
            const double dz =
                tilted_targets.feet[leg].pos_body_m.z - level_targets.feet[leg].pos_body_m.z;
            peak_dz = std::max(peak_dz, std::abs(dz));
            const double y = level_targets.feet[leg].pos_body_m.y;
            if (std::abs(y) > 0.05 && dz * y < 0.0) {
                ++opposite_sign_pairs;
            }
        }
        if (!expect(peak_dz > 0.012,
                    "STAND should untilt prefix roll by more than terrain-leveling's 14 mm cap")) {
            return EXIT_FAILURE;
        }
        if (!expect(opposite_sign_pairs >= 2,
                    "STAND untilt should drop the high-side feet and raise the low-side feet")) {
            return EXIT_FAILURE;
        }
        if (!expect(standTargetsRemainReachable(tilted_targets),
                    "STAND untilt targets should stay within leg reach")) {
            return EXIT_FAILURE;
        }
        for (int i = 0; i < 320; ++i) {
            (void)tilted_controller.update(untilt_est, untilt_intent, untilt_gait, safety, untilt_twist);
        }
        const LegTargets settled_targets =
            tilted_controller.update(untilt_est, untilt_intent, untilt_gait, safety, untilt_twist);
        double mean_dz = 0.0;
        std::array<double, kNumLegs> settled_dz{};
        for (int leg = 0; leg < kNumLegs; ++leg) {
            settled_dz[static_cast<std::size_t>(leg)] =
                settled_targets.feet[leg].pos_body_m.z - level_targets.feet[leg].pos_body_m.z;
            mean_dz += settled_dz[static_cast<std::size_t>(leg)];
        }
        mean_dz /= static_cast<double>(kNumLegs);
        double residual_dz = 0.0;
        for (int leg = 0; leg < kNumLegs; ++leg) {
            residual_dz = std::max(residual_dz, std::abs(settled_dz[static_cast<std::size_t>(leg)] - mean_dz));
        }
        if (!expect(residual_dz < 0.003,
                    "STAND untilt should fade back to identity before WALK")) {
            return EXIT_FAILURE;
        }
    }

    MotionIntent walk_intent{};
    walk_intent.requested_mode = RobotMode::WALK;
    walk_intent.speed_mps = LinearRateMps{0.2};
    walk_intent.heading_rad = AngleRad{0.0};
    walk_intent.cmd_vx_mps = LinearRateMps{0.2};
    walk_intent.cmd_vy_mps = LinearRateMps{0.0};
    walk_intent.twist.body_trans_mps = Vec3{0.05, -0.03, 0.0};
    walk_intent.twist.body_trans_m.z = 0.12;

    GaitState walk_gait{};
    walk_gait.phase[0] = 0.25;
    walk_gait.in_stance[0] = true;
    walk_gait.duty_factor = 0.5;
    walk_gait.stride_phase_rate_hz = FrequencyHz{1.0};
    walk_gait.step_length_m = 0.06;
    walk_gait.swing_height_m = 0.03;
    const BodyTwist walk_twist =
        rawLocomotionTwistFromIntent(walk_intent, planarMotionCommand(walk_intent));
    const LegTargets walk_targets = controller.update(est, walk_intent, walk_gait, safety, walk_twist);

    if (!expect(nearlyEqual(walk_targets.feet[0].vel_body_mps.x, -0.25, 2e-3) &&
                    nearlyEqual(walk_targets.feet[0].vel_body_mps.y, 0.03, 2e-3) &&
                    nearlyEqual(walk_targets.feet[0].vel_body_mps.z, 0.0, 2e-3),
                "walking stance foot velocity should follow the composed body motion without extra sway")) {
        return EXIT_FAILURE;
    }
    if (!expect(nearlyEqual(walk_targets.feet[0].vel_body_mps.x, -0.25, 2e-3),
                "walking stance foot velocity should include sway exactly once")) {
        return EXIT_FAILURE;
    }

    // Geometry, commands and IK now share the canonical server frame. Check the
    // command/IK boundary explicitly so forward and lateral inputs cannot silently
    // exchange or reverse their stance sweeps.
    BodyController cardinal_controller{};
    MotionIntent cardinal_intent = walk_intent;
    cardinal_intent.twist.body_trans_mps = Vec3{0.0, 0.0, 0.0};
    GaitState cardinal_gait = walk_gait;
    cardinal_gait.phase[0] = 0.25;
    cardinal_gait.in_stance[0] = true;
    const BodyTwist forward_command{Vec3{0.06, 0.0, 0.0}, Vec3{}};
    const LegTargets forward_targets =
        cardinal_controller.update(est, cardinal_intent, cardinal_gait, safety, forward_command);
    if (!expect(forward_targets.feet[0].vel_body_mps.x < 0.0 &&
                    std::abs(forward_targets.feet[0].vel_body_mps.y) < 1e-9,
                "forward command should produce a pure negative-X stance sweep at the IK boundary")) {
        return EXIT_FAILURE;
    }
    const BodyTwist left_command{Vec3{0.0, 0.06, 0.0}, Vec3{}};
    const LegTargets left_targets =
        cardinal_controller.update(est, cardinal_intent, cardinal_gait, safety, left_command);
    if (!expect(left_targets.feet[0].vel_body_mps.y < 0.0 &&
                    std::abs(left_targets.feet[0].vel_body_mps.x) < 1e-9,
                "left command should produce a pure negative-Y stance sweep at the IK boundary")) {
        return EXIT_FAILURE;
    }

    GaitState swing_gait = walk_gait;
    swing_gait.phase[0] = 0.60;
    swing_gait.in_stance[0] = false;
    const LegTargets swing_targets = controller.update(est, walk_intent, swing_gait, safety, walk_twist);

    GaitState held_gait = swing_gait;
    held_gait.stability_hold_stance[0] = true;
    RobotState airborne_est = est;
    airborne_est.foot_contacts[0] = false;
    airborne_est.foot_contact_fusion[0].phase = ContactPhase::Search;
    const LegTargets held_airborne_targets =
        controller.update(airborne_est, walk_intent, held_gait, safety, walk_twist);
    if (!expect(nearlyEqual(held_airborne_targets.feet[0].pos_body_m.z, swing_targets.feet[0].pos_body_m.z, 2e-3),
                "stability hold should not pin an unsupported swing leg back into stance")) {
        return EXIT_FAILURE;
    }
    if (!expect(nearlyEqual(held_airborne_targets.feet[0].vel_body_mps.z, swing_targets.feet[0].vel_body_mps.z, 2e-3),
                "stability hold should preserve swing vertical motion when contact support is gone")) {
        return EXIT_FAILURE;
    }

    GaitState recovery_hold_gait = held_gait;
    recovery_hold_gait.stride_phase_rate_hz = FrequencyHz{0.0};
    const LegTargets recovery_touchdown_targets =
        controller.update(airborne_est, walk_intent, recovery_hold_gait, safety, walk_twist);
    if (!expect(recovery_touchdown_targets.feet[0].pos_body_m.z < held_airborne_targets.feet[0].pos_body_m.z - 1e-4,
                "recovery hold should drive unsupported swing legs toward touchdown instead of freezing them mid-air")) {
        return EXIT_FAILURE;
    }

    RobotState supported_est = est;
    supported_est.foot_contacts[0] = true;
    supported_est.foot_contact_fusion[0].phase = ContactPhase::ConfirmedStance;
    const LegTargets held_supported_targets =
        controller.update(supported_est, walk_intent, held_gait, safety, walk_twist);
    if (!expect(held_supported_targets.feet[0].pos_body_m.z < held_airborne_targets.feet[0].pos_body_m.z - 0.003,
                "supported swing legs should stay in stance kinematics until support is released")) {
        return EXIT_FAILURE;
    }
    if (!expect(nearlyEqual(held_supported_targets.feet[0].vel_body_mps.z, 0.0, 2e-3),
                "supported swing legs kept in stance should suppress vertical swing motion")) {
        return EXIT_FAILURE;
    }

    RobotState touchdown_est = est;
    touchdown_est.foot_contacts[0] = false;
    touchdown_est.foot_contact_fusion[0].phase = ContactPhase::LostCandidate;
    const LegTargets held_touchdown_targets =
        controller.update(touchdown_est, walk_intent, held_gait, safety, walk_twist);
    if (!expect(held_touchdown_targets.feet[0].pos_body_m.z < held_airborne_targets.feet[0].pos_body_m.z - 0.003,
                "recovery hold should keep touchdown-completing supported legs in stance until load is released")) {
        return EXIT_FAILURE;
    }

    RobotState low_body_est = est;
    low_body_est.has_body_twist_state = true;
    low_body_est.has_fusion_diagnostics = true;
    low_body_est.fusion.model_trust = 1.0;
    low_body_est.body_twist_state.body_trans_m.z = 0.03;
    const LegTargets held_height_targets =
        controller.update(low_body_est, walk_intent, walk_gait, safety, walk_twist);
    if (!expect(held_height_targets.feet[0].pos_body_m.z < walk_targets.feet[0].pos_body_m.z - 0.008,
                "walking height hold should materially lower stance feet when the body sags")) {
        return EXIT_FAILURE;
    }

    // Height hold is a stance preload, not part of swing clearance. A sagging body may
    // lower planted targets, but the same correction must be released when that leg swings.
    {
        BodyController nominal_swing_controller{};
        BodyController sagged_swing_controller{};
        GaitState swing_height_gait = walk_gait;
        swing_height_gait.phase[0] = 0.75;
        swing_height_gait.in_stance[0] = false;
        RobotState nominal_height_est = est;
        nominal_height_est.valid = true;
        nominal_height_est.has_body_twist_state = true;
        nominal_height_est.body_twist_state.body_trans_m.z = 0.12;
        RobotState sagged_height_est = nominal_height_est;
        sagged_height_est.body_twist_state.body_trans_m.z = 0.03;
        const LegTargets nominal_swing = nominal_swing_controller.update(
            nominal_height_est, walk_intent, swing_height_gait, safety, walk_twist);
        const LegTargets sagged_swing = sagged_swing_controller.update(
            sagged_height_est, walk_intent, swing_height_gait, safety, walk_twist);
        if (!expect(nearlyEqual(sagged_swing.feet[0].pos_body_m.z,
                                nominal_swing.feet[0].pos_body_m.z,
                                1e-6),
                    "swing target must release stance height-hold preload")) {
            return EXIT_FAILURE;
        }
    }

    MotionIntent idle_intent = stand_intent;
    idle_intent.requested_mode = RobotMode::SAFE_IDLE;
    const LegTargets idle_targets = controller.update(low_body_est, idle_intent, gait, safety, stand_twist);
    if (!expect(idle_targets.feet[0].pos_body_m.z < stand_targets.feet[0].pos_body_m.z - 0.008,
                "idle height hold should also materially lower stance feet when the body sags")) {
        return EXIT_FAILURE;
    }

    bool clock_ok = true;

    // Late-swing contact after grace keeps swing kinematics so a lingering plant cannot
    // re-lock the foot into extra-stance (H1). Hold-based extra-stance is unchanged.
    {
        BodyController late_controller{};
        BodyController duty_controller{};
        MotionIntent late_intent = walk_intent;
        late_intent.twist.body_trans_mps = Vec3{0.0, 0.0, 0.0};
        RobotState late_est{};
        late_est.foot_contacts[0] = true;
        late_est.foot_contact_fusion[0].phase = ContactPhase::ConfirmedStance;
        GaitState late_gait = walk_gait;
        late_gait.duty_factor = 0.5;
        late_gait.stride_phase_rate_hz = FrequencyHz{1.0};
        late_gait.stability_hold_stance[0] = false;
        const BodyTwist late_cmd{Vec3{0.12, 0.0, 0.0}, Vec3{}};
        GaitState duty_gait = late_gait;
        duty_gait.phase[0] = 0.499;
        duty_gait.in_stance[0] = true;
        late_gait.phase[0] = 0.80;
        late_gait.in_stance[0] = false;
        const LegTargets duty_targets =
            duty_controller.update(late_est, late_intent, duty_gait, safety, late_cmd);
        const LegTargets late_targets =
            late_controller.update(late_est, late_intent, late_gait, safety, late_cmd);
        RobotState swing_est = late_est;
        swing_est.foot_contacts[0] = false;
        swing_est.foot_contact_fusion[0].phase = ContactPhase::Search;
        BodyController swing_controller{};
        const LegTargets swing_targets =
            swing_controller.update(swing_est, late_intent, late_gait, safety, late_cmd);
        const double vs_swing_xy = std::hypot(
            late_targets.feet[0].pos_body_m.x - swing_targets.feet[0].pos_body_m.x,
            late_targets.feet[0].pos_body_m.y - swing_targets.feet[0].pos_body_m.y);
        clock_ok = expect(vs_swing_xy < 0.002,
                          "late-swing contact must keep swing kinematics, not extra-stance plant")
            && clock_ok;
        clock_ok = expect(late_targets.feet[0].pos_body_m.z > duty_targets.feet[0].pos_body_m.z + 0.005,
                          "late-swing contact should command a raised swing foot")
            && clock_ok;
    }

    // A planted foot that stays in stance kinematics across a phase wrap must not jump back
    // to the phase-zero anchor. Advance intent time by one 5 ms frame so dt integration can
    // replace the wrapping φ/f closed form.
    {
        BodyController wrap_controller{};
        MotionIntent wrap_intent{};
        wrap_intent.requested_mode = RobotMode::WALK;
        wrap_intent.twist.body_trans_m.z = 0.12;
        wrap_intent.timestamp_us = TimePointUs{1'000'000};
        RobotState wrap_est{};
        wrap_est.foot_contacts[0] = true;
        wrap_est.foot_contact_fusion[0].phase = ContactPhase::ConfirmedStance;
        GaitState wrap_gait{};
        wrap_gait.duty_factor = 0.5;
        wrap_gait.stride_phase_rate_hz = FrequencyHz{1.0};
        wrap_gait.step_length_m = 0.06;
        wrap_gait.swing_height_m = 0.03;
        wrap_gait.phase[0] = 0.99;
        wrap_gait.in_stance[0] = false;
        wrap_gait.stability_hold_stance[0] = true;
        const BodyTwist wrap_cmd{Vec3{0.12, 0.0, 0.0}, Vec3{}};
        const LegTargets before =
            wrap_controller.update(wrap_est, wrap_intent, wrap_gait, safety, wrap_cmd);
        wrap_intent.timestamp_us = TimePointUs{1'005'000};
        wrap_gait.phase[0] = 0.01;
        wrap_gait.in_stance[0] = true;
        const LegTargets after =
            wrap_controller.update(wrap_est, wrap_intent, wrap_gait, safety, wrap_cmd);
        const Vec3 step = after.feet[0].pos_body_m - before.feet[0].pos_body_m;
        const double dt_s = 0.005;
        const double planar = std::hypot(step.x, step.y);
        const double along = step.x * before.feet[0].vel_body_mps.x
            + step.y * before.feet[0].vel_body_mps.y;
        const double v_planar =
            std::hypot(before.feet[0].vel_body_mps.x, before.feet[0].vel_body_mps.y);
        clock_ok = expect(planar <= std::max(0.005, 8.0 * v_planar * dt_s),
                          "held stance wrap must not reset the foot to the phase-zero anchor")
            && clock_ok;
        clock_ok = expect(along >= -1e-4,
                          "held stance wrap must not reverse the support sweep")
            && clock_ok;
    }

    // Continuing planned stance must integrate time. Re-evaluating φ/f when cadence or
    // command speed changes contracts the stroke toward the anchor (with the command).
    {
        const auto serverStepOpposes = [](const Vec3& step, const BodyTwist& cmd) {
            return cmd.linear_mps.x * step.x + cmd.linear_mps.y * step.y < -1e-9;
        };
        const auto runPair = [&](BodyController& bc,
                                 GaitState gait_a,
                                 GaitState gait_b,
                                 const BodyTwist& cmd_a,
                                 const BodyTwist& cmd_b) {
            MotionIntent intent{};
            intent.requested_mode = RobotMode::WALK;
            intent.twist.body_trans_m.z = 0.12;
            intent.timestamp_us = TimePointUs{1'000'000};
            RobotState planted{};
            planted.foot_contacts[0] = true;
            planted.foot_contact_fusion[0].phase = ContactPhase::ConfirmedStance;
            const LegTargets before = bc.update(planted, intent, gait_a, safety, cmd_a);
            intent.timestamp_us = TimePointUs{1'005'000};
            const LegTargets after = bc.update(planted, intent, gait_b, safety, cmd_b);
            return after.feet[0].pos_body_m - before.feet[0].pos_body_m;
        };

        GaitState planned_gait{};
        planned_gait.duty_factor = 0.5;
        planned_gait.stride_phase_rate_hz = FrequencyHz{1.0};
        planned_gait.step_length_m = 0.06;
        planned_gait.swing_height_m = 0.03;
        planned_gait.phase[0] = 0.25;
        planned_gait.in_stance[0] = true;
        const BodyTwist forward_cmd{Vec3{0.12, 0.0, 0.0}, Vec3{}};
        const double dt_s = 0.005;

        {
            BodyController constant_f{};
            GaitState second = planned_gait;
            second.phase[0] = 0.25 + 1.0 * dt_s;
            const Vec3 step = runPair(constant_f, planned_gait, second, forward_cmd, forward_cmd);
            const double planar = std::hypot(step.x, step.y);
            clock_ok = expect(serverStepOpposes(step, forward_cmd),
                              "constant-f planned stance must sweep opposite the server command")
                && clock_ok;
            clock_ok = expect(planar > 0.1 * 0.12 * dt_s && planar < 8.0 * 0.12 * dt_s,
                              "constant-f planned stance step should be about |v| dt")
                && clock_ok;
        }

        {
            BodyController cadence_jump{};
            GaitState second = planned_gait;
            second.phase[0] = 0.25 + 1.0 * dt_s;
            second.stride_phase_rate_hz = FrequencyHz{2.0};
            const Vec3 step = runPair(cadence_jump, planned_gait, second, forward_cmd, forward_cmd);
            const double planar = std::hypot(step.x, step.y);
            clock_ok = expect(serverStepOpposes(step, forward_cmd),
                              "cadence jump must not contract φ/f toward the anchor")
                && clock_ok;
            clock_ok = expect(planar < 8.0 * 0.12 * dt_s,
                              "cadence jump must not produce a φ/f contraction much larger than |v| dt")
                && clock_ok;
        }

        {
            BodyController scale_jump{};
            GaitState second = planned_gait;
            second.phase[0] = 0.25 + 1.0 * dt_s;
            const BodyTwist half_cmd{Vec3{0.06, 0.0, 0.0}, Vec3{}};
            const Vec3 step = runPair(scale_jump, planned_gait, second, forward_cmd, half_cmd);
            clock_ok = expect(serverStepOpposes(step, forward_cmd),
                              "command-scale drop must not snap the closed form toward the anchor")
                && clock_ok;
        }

        {
            BodyController entry{};
            MotionIntent intent{};
            intent.requested_mode = RobotMode::WALK;
            intent.twist.body_trans_m.z = 0.12;
            intent.timestamp_us = TimePointUs{1'000'000};
            RobotState planted{};
            planted.foot_contacts.fill(true);
            for (auto& fusion : planted.foot_contact_fusion) {
                fusion.phase = ContactPhase::ConfirmedStance;
            }
            GaitState entry_gait{};
            entry_gait.duty_factor = 0.94;
            entry_gait.stride_phase_rate_hz = FrequencyHz{1.0};
            entry_gait.step_length_m = 0.06;
            entry_gait.swing_height_m = 0.03;
            entry_gait.phase.fill(0.0);
            entry_gait.in_stance.fill(true);
            const BodyTwist entry_cmd{Vec3{0.12, 0.0, 0.0}, Vec3{}};
            const LegTargets first = entry.update(planted, intent, entry_gait, safety, entry_cmd);
            Vec3 last = first.feet[0].pos_body_m;
            double max_stroke = 0.0;
            for (int frame = 1; frame < 80; ++frame) {
                intent.timestamp_us.value += 5'000;
                entry_gait.phase[0] = std::min(0.93, static_cast<double>(frame) * dt_s);
                const LegTargets now = entry.update(planted, intent, entry_gait, safety, entry_cmd);
                last = now.feet[0].pos_body_m;
                const double stroke = std::hypot(
                    last.x - first.feet[0].pos_body_m.x, last.y - first.feet[0].pos_body_m.y);
                max_stroke = std::max(max_stroke, stroke);
            }
            const double v_xy = std::hypot(first.feet[0].vel_body_mps.x, first.feet[0].vel_body_mps.y);
            clock_ok = expect(max_stroke <= v_xy * 0.94 / 1.0 + 0.01,
                              "high-duty walk-entry must not integrate past the latched stroke budget")
                && clock_ok;
        }

        {
            BodyController seeded{};
            BodyController origin{};
            MotionIntent intent{};
            intent.requested_mode = RobotMode::WALK;
            intent.twist.body_trans_m.z = 0.12;
            intent.timestamp_us = TimePointUs{1'000'000};
            RobotState planted{};
            planted.foot_contacts.fill(true);
            for (auto& fusion : planted.foot_contact_fusion) {
                fusion.phase = ContactPhase::ConfirmedStance;
            }
            GaitState origin_gait{};
            origin_gait.duty_factor = 0.94;
            origin_gait.stride_phase_rate_hz = FrequencyHz{1.0};
            origin_gait.step_length_m = 0.06;
            origin_gait.swing_height_m = 0.03;
            origin_gait.phase.fill(0.0);
            origin_gait.in_stance.fill(true);
            GaitState seeded_gait = origin_gait;
            seeded_gait.phase.fill(0.35);
            const BodyTwist entry_cmd{Vec3{0.12, 0.0, 0.0}, Vec3{}};
            const LegTargets origin_targets =
                origin.update(planted, intent, origin_gait, safety, entry_cmd);
            const LegTargets seeded_targets =
                seeded.update(planted, intent, seeded_gait, safety, entry_cmd);
            const double jump = std::hypot(
                seeded_targets.feet[0].pos_body_m.x - origin_targets.feet[0].pos_body_m.x,
                seeded_targets.feet[0].pos_body_m.y - origin_targets.feet[0].pos_body_m.y);
            clock_ok = expect(jump < 0.002,
                              "a new plant at φ=0.35 must start at the stance origin, not mid-stroke")
                && clock_ok;
        }

        {
            BodyController identity{};
            MotionIntent intent{};
            intent.requested_mode = RobotMode::WALK;
            intent.twist.body_trans_m.z = 0.12;
            intent.timestamp_us = TimePointUs{1'000'000};
            RobotState planted{};
            planted.foot_contacts[0] = true;
            planted.foot_contact_fusion[0].phase = ContactPhase::ConfirmedStance;
            GaitState identity_gait{};
            identity_gait.duty_factor = 0.5;
            identity_gait.stride_phase_rate_hz = FrequencyHz{1.0};
            identity_gait.step_length_m = 0.06;
            identity_gait.swing_height_m = 0.03;
            identity_gait.phase[0] = 0.10;
            identity_gait.in_stance[0] = true;
            const BodyTwist identity_cmd{Vec3{0.12, 0.0, 0.0}, Vec3{}};
            const LegTargets first = identity.update(planted, intent, identity_gait, safety, identity_cmd);
            Vec3 last = first.feet[0].pos_body_m;
            double opposition_sum = 0.0;
            int opposition_samples = 0;
            bool phase_in_window = true;
            for (int frame = 1; frame < 40; ++frame) {
                intent.timestamp_us.value += 5'000;
                identity_gait.phase[0] = 0.10 + static_cast<double>(frame) * dt_s;
                phase_in_window =
                    phase_in_window && identity_gait.phase[0] > 0.05 && identity_gait.phase[0] < 0.45;
                const LegTargets now = identity.update(planted, intent, identity_gait, safety, identity_cmd);
                const Vec3 step = now.feet[0].pos_body_m - last;
                last = now.feet[0].pos_body_m;
                const double opposition_speed =
                    -(identity_cmd.linear_mps.x * step.x + identity_cmd.linear_mps.y * step.y)
                    / (0.12 * dt_s);
                opposition_sum += opposition_speed;
                ++opposition_samples;
            }
            const double mean_opposition = opposition_sum / static_cast<double>(opposition_samples);
            clock_ok = expect(phase_in_window,
                              "identity walk should keep φ in (0.05, 0.45)")
                && clock_ok;
            clock_ok = expect(std::abs(mean_opposition - 0.12) <= 0.15 * 0.12,
                              "mid-stance Cartesian opposition should match |v| within 15%")
                && clock_ok;
        }

        {
            BodyController saturating{};
            MotionIntent intent{};
            intent.requested_mode = RobotMode::WALK;
            intent.twist.body_trans_m.z = 0.12;
            intent.timestamp_us = TimePointUs{1'000'000};
            RobotState planted{};
            planted.foot_contacts[0] = true;
            planted.foot_contact_fusion[0].phase = ContactPhase::ConfirmedStance;
            GaitState clamp_gait{};
            clamp_gait.duty_factor = 0.20;
            clamp_gait.stride_phase_rate_hz = FrequencyHz{2.0};
            clamp_gait.step_length_m = 0.06;
            clamp_gait.swing_height_m = 0.03;
            clamp_gait.phase[0] = 0.10;
            clamp_gait.in_stance[0] = true;
            const BodyTwist clamp_cmd{Vec3{0.12, 0.0, 0.0}, Vec3{}};
            const LegTargets first = saturating.update(planted, intent, clamp_gait, safety, clamp_cmd);
            const Vec3 plant = first.feet[0].pos_body_m;
            const double v_xy = std::hypot(first.feet[0].vel_body_mps.x, first.feet[0].vel_body_mps.y);
            const double stroke_l_m = v_xy * 0.20 / 2.0;
            bool saw_clamp_hit = false;
            double max_stroke = 0.0;
            for (int frame = 1; frame < 80; ++frame) {
                intent.timestamp_us.value += 5'000;
                const LegTargets now = saturating.update(planted, intent, clamp_gait, safety, clamp_cmd);
                saw_clamp_hit = saw_clamp_hit || saturating.lastStrokeClampHit()[0];
                max_stroke = std::max(
                    max_stroke,
                    std::hypot(now.feet[0].pos_body_m.x - plant.x, now.feet[0].pos_body_m.y - plant.y));
            }
            clock_ok = expect(stroke_l_m > 0.005,
                              "clamp-saturation budget should be a few millimetres")
                && clock_ok;
            clock_ok = expect(saw_clamp_hit, "tight stroke budget must hit the plant clamp") && clock_ok;
            clock_ok = expect(max_stroke <= stroke_l_m + 0.002,
                              "planar stroke from the plant must stop at L + 2 mm")
                && clock_ok;
        }

        {
            BodyController late_swing{};
            MotionIntent intent{};
            intent.requested_mode = RobotMode::WALK;
            intent.twist.body_trans_m.z = 0.12;
            intent.timestamp_us = TimePointUs{1'000'000};
            RobotState planted{};
            planted.foot_contacts[0] = true;
            planted.foot_contact_fusion[0].phase = ContactPhase::ConfirmedStance;
            GaitState late_gait{};
            late_gait.duty_factor = 0.5;
            late_gait.stride_phase_rate_hz = FrequencyHz{1.0};
            late_gait.step_length_m = 0.06;
            late_gait.swing_height_m = 0.03;
            late_gait.phase[0] = 0.80;
            late_gait.in_stance[0] = false;
            const BodyTwist late_cmd{Vec3{0.12, 0.0, 0.0}, Vec3{}};
            const LegTargets first = late_swing.update(planted, intent, late_gait, safety, late_cmd);
            const Vec3 plant = first.feet[0].pos_body_m;
            const double v_xy = std::hypot(first.feet[0].vel_body_mps.x, first.feet[0].vel_body_mps.y);
            const double stroke_l_m = v_xy * 0.5 / 1.0;
            double max_from_first = 0.0;
            bool saw_clamp_hit = false;
            for (int frame = 1; frame < 160; ++frame) {
                intent.timestamp_us.value += 5'000;
                const LegTargets now = late_swing.update(planted, intent, late_gait, safety, late_cmd);
                saw_clamp_hit = saw_clamp_hit || late_swing.lastStrokeClampHit()[0];
                max_from_first = std::max(
                    max_from_first,
                    std::hypot(now.feet[0].pos_body_m.x - plant.x, now.feet[0].pos_body_m.y - plant.y));
            }
            clock_ok = expect(!saw_clamp_hit,
                              "late-swing contact must not re-plant into the stance stroke clamp")
                && clock_ok;
            clock_ok = expect(first.feet[0].pos_body_m.z > -0.10,
                              "late-swing contact should command a raised swing foot")
                && clock_ok;
            (void)stroke_l_m;
            (void)max_from_first;
        }

        {
            BodyController wrap_budget{};
            MotionIntent intent{};
            intent.requested_mode = RobotMode::WALK;
            intent.twist.body_trans_m.z = 0.12;
            intent.timestamp_us = TimePointUs{1'000'000};
            RobotState planted{};
            planted.foot_contacts[0] = true;
            planted.foot_contact_fusion[0].phase = ContactPhase::ConfirmedStance;
            GaitState wrap_gait{};
            wrap_gait.duty_factor = 0.5;
            wrap_gait.stride_phase_rate_hz = FrequencyHz{1.0};
            wrap_gait.step_length_m = 0.06;
            wrap_gait.swing_height_m = 0.03;
            wrap_gait.phase[0] = 0.80;
            wrap_gait.in_stance[0] = false;
            const BodyTwist wrap_cmd{Vec3{0.12, 0.0, 0.0}, Vec3{}};
            const LegTargets extra = wrap_budget.update(planted, intent, wrap_gait, safety, wrap_cmd);
            const Vec3 extra_pos = extra.feet[0].pos_body_m;
            for (int frame = 1; frame < 10; ++frame) {
                intent.timestamp_us.value += 5'000;
                (void)wrap_budget.update(planted, intent, wrap_gait, safety, wrap_cmd);
            }
            wrap_gait.phase[0] = 0.10;
            wrap_gait.in_stance[0] = true;
            intent.timestamp_us.value += 5'000;
            const LegTargets planned0 = wrap_budget.update(planted, intent, wrap_gait, safety, wrap_cmd);
            const Vec3 planned_plant = planned0.feet[0].pos_body_m;
            int workspace_xy_hits = 0;
            double max_from_planned = 0.0;
            const double wrap_l_m =
                std::hypot(planned0.feet[0].vel_body_mps.x, planned0.feet[0].vel_body_mps.y) * 0.5 / 1.0;
            for (int frame = 1; frame < 80; ++frame) {
                intent.timestamp_us.value += 5'000;
                wrap_gait.phase[0] = 0.10 + static_cast<double>(frame) * dt_s;
                const LegTargets now = wrap_budget.update(planted, intent, wrap_gait, safety, wrap_cmd);
                workspace_xy_hits += wrap_budget.lastWorkspaceXyHit()[0] ? 1 : 0;
                max_from_planned = std::max(
                    max_from_planned,
                    std::hypot(now.feet[0].pos_body_m.x - planned_plant.x,
                               now.feet[0].pos_body_m.y - planned_plant.y));
            }
            (void)extra_pos;
            clock_ok = expect(max_from_planned <= wrap_l_m + 0.002,
                              "replanted planned stance after late swing must keep the new stroke budget")
                && clock_ok;
            clock_ok = expect(max_from_planned > 0.01,
                              "replanted planned stance after late swing must still stroke")
                && clock_ok;
            clock_ok = expect(workspace_xy_hits == 0,
                              "planned stance after late swing must not skate the annulus")
                && clock_ok;
        }
    }

    return clock_ok ? EXIT_SUCCESS : EXIT_FAILURE;
}
