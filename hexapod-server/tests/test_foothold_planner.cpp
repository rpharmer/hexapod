#include "foot_planners.hpp"
#include "foothold_planner.hpp"
#include "stance_progress_metrics.hpp"

#include <array>
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

bool nearlyEq(double a, double b, double eps = 1e-9) {
    return std::abs(a - b) <= eps;
}

bool nearlyEqVec(const Vec3& a, const Vec3& b, double eps = 1e-9) {
    return nearlyEq(a.x, b.x, eps) && nearlyEq(a.y, b.y, eps) && nearlyEq(a.z, b.z, eps);
}

bool planarCycleFollowsCommand(const BodyTwist& body, const char* label) {
    const Vec3 anchor{0.18, 0.02, -0.11};
    StanceFootInputs stance{};
    stance.anchor = anchor;
    stance.v_foot_body = supportFootVelocityAt(anchor, body);
    stance.phase = 0.5;
    stance.f_hz = 1.0;

    Vec3 stance_end{};
    Vec3 stance_velocity{};
    planStanceFoot(stance, stance_end, stance_velocity);
    const Vec3 stance_delta = stance_end - anchor;
    const double stance_along_command =
        stance_delta.x * body.linear_mps.x + stance_delta.y * body.linear_mps.y;
    if (!(stance_along_command < -1e-9)) {
        std::cerr << "FAIL: " << label << " stance foot should sweep opposite the body command\n";
        return false;
    }

    SwingFootInputs swing{};
    swing.anchor = anchor;
    swing.stance_end = stance_end;
    swing.v_liftoff_body = stance_velocity;
    swing.tau01 = 1.0;
    swing.swing_span = 0.5;
    swing.f_hz = 1.0;
    swing.step_length_m = 0.06;
    swing.swing_height_m = 0.03;
    swing.static_stability_margin_m = 0.03;
    swing.swing_time_ease_01 = 1.0;

    RobotState no_capture_est{};
    Vec3 touchdown{};
    Vec3 touchdown_velocity{};
    planSwingFoot(no_capture_est, body, swing, touchdown, touchdown_velocity);
    const Vec3 swing_delta = touchdown - stance_end;
    const double swing_along_command =
        swing_delta.x * body.linear_mps.x + swing_delta.y * body.linear_mps.y;
    const double planar_cross =
        swing_delta.x * body.linear_mps.y - swing_delta.y * body.linear_mps.x;
    if (!(swing_along_command > 1e-9)) {
        std::cerr << "FAIL: " << label << " swing foot should recover in the body-command direction\n";
        return false;
    }
    if (!(std::abs(planar_cross) < 1e-9)) {
        std::cerr << "FAIL: " << label << " swing recovery should not leak into the cross axis\n";
        return false;
    }
    return true;
}

} // namespace

int main() {
    BodyTwist body{};
    body.linear_mps = Vec3{0.1, 0.0, 0.0};
    body.angular_radps = Vec3{0.0, 0.0, 0.0};
    const Vec3 foot{0.0, 0.1, -0.05};
    const Vec3 d = twistIntegratedFootholdDeltaXY(body, foot, 0.2);
    if (!expect(nearlyEq(d.z, 0.0) && d.x < 0.0, "twist-integrated foothold should oppose forward motion in x")) {
        return EXIT_FAILURE;
    }

    const BodyTwist forward{Vec3{0.08, 0.0, 0.0}, Vec3{}};
    const BodyTwist backward{Vec3{-0.08, 0.0, 0.0}, Vec3{}};
    const BodyTwist strafe_left{Vec3{0.0, 0.08, 0.0}, Vec3{}};
    const BodyTwist strafe_right{Vec3{0.0, -0.08, 0.0}, Vec3{}};
    if (!planarCycleFollowsCommand(forward, "forward") ||
        !planarCycleFollowsCommand(backward, "backward") ||
        !planarCycleFollowsCommand(strafe_left, "strafe-left") ||
        !planarCycleFollowsCommand(strafe_right, "strafe-right")) {
        return EXIT_FAILURE;
    }

    // φ/f is a world-fixed support sweep only on the planned-stance interval. Sample the
    // closed form so a later wrap or late-swing contact cannot hide behind the mean.
    {
        const Vec3 anchor{0.18, 0.02, -0.11};
        const double duty = 0.5;
        const double f_hz = 1.0;
        const Vec3 v_foot = supportFootVelocityAt(anchor, forward);
        Vec3 previous = anchor;
        for (int sample = 0; sample <= 20; ++sample) {
            const double phi = duty * static_cast<double>(sample) / 20.0;
            StanceFootInputs st{};
            st.anchor = anchor;
            st.v_foot_body = v_foot;
            st.phase = phi;
            st.f_hz = f_hz;
            Vec3 pos{};
            Vec3 vel{};
            planStanceFoot(st, pos, vel);
            const Vec3 delta = pos - previous;
            const double along_command = delta.x * forward.linear_mps.x + delta.y * forward.linear_mps.y;
            if (sample > 0 && along_command > 1e-12) {
                std::cerr << "FAIL: planned-stance φ=" << phi
                          << " must keep sweeping opposite the body command\n";
                return EXIT_FAILURE;
            }
            previous = pos;
        }
        StanceFootInputs stance_end_in{};
        stance_end_in.anchor = anchor;
        stance_end_in.v_foot_body = v_foot;
        stance_end_in.phase = duty;
        stance_end_in.f_hz = f_hz;
        Vec3 stance_end{};
        Vec3 stance_end_vel{};
        planStanceFoot(stance_end_in, stance_end, stance_end_vel);
        const Vec3 integral = stance_end - anchor;
        const Vec3 expected = v_foot * (duty / f_hz);
        if (!expect(nearlyEqVec(integral, expected, 1e-9),
                    "planned-stance integral must equal v_foot * duty / f")) {
            return EXIT_FAILURE;
        }
    }

    {
        constexpr double duty = 0.5;
        if (!expect(isOnsetPlannedStance(true, true, 0.01, duty),
                    "φ=0.01 at duty 0.5 is onset")
            || !expect(!isMidPlannedStance(true, 0.01, duty),
                       "φ=0.01 at duty 0.5 is not mid-stance")
            || !expect(isMidPlannedStance(true, 0.25, duty),
                       "φ=0.25 at duty 0.5 is mid-stance")
            || !expect(!isOnsetPlannedStance(true, true, 0.25, duty),
                       "continuing planned φ=0.25 is not onset")
            || !expect(isOnsetPlannedStance(true, false, 0.25, duty),
                       "first planned frame is onset even at mid φ")
            || !expect(!isMidPlannedStance(true, 0.49, duty),
                       "φ=0.49 at duty 0.5 is near-liftoff, not mid-stance")
            || !expect(!isMidPlannedStance(false, 0.25, duty),
                       "unplanned contact is not mid-stance")) {
            return EXIT_FAILURE;
        }
        const std::array<bool, 6> tripod{true, true, true, false, false, false};
        const std::array<bool, 6> overlap{true, true, true, true, false, false};
        if (!expect(isTripodStanceFrame(plannedStanceCount(tripod)),
                    "three planned stance feet is a tripod frame")
            || !expect(isOverlapStanceFrame(plannedStanceCount(overlap)),
                       "four planned stance feet is an overlap frame")
            || !expect(isCleanTripodFrame(tripod, tripod, 0),
                       "matched 3-contact planned set with no L park is clean")
            || !expect(!isCleanTripodFrame(tripod, overlap, 0),
                       "an extra contact is not a clean tripod")
            || !expect(!isCleanTripodFrame(tripod, tripod, 1),
                       "L-parked contacted stance is not a clean tripod")
            || !expect(!isHighDuty(0.5) && isHighDuty(0.71),
                       "walk-entry duty > 0.70 is high-duty")) {
            return EXIT_FAILURE;
        }
    }

    const Vec3 c = clampFootholdExtraXY(Vec3{0.3, 0.4, 0.0}, 0.5);
    const double h = std::hypot(c.x, c.y);
    if (!expect(h <= 0.5 + 1e-9, "clamp hypot")) {
        return EXIT_FAILURE;
    }

    const Vec3 b = stabilityFootholdBiasXY(0.0, Vec3{0.1, 0.0, 0.0});
    if (!expect(b.x < 0.0, "low margin should bias inward (negative x for +x foot)")) {
        return EXIT_FAILURE;
    }

    RobotState est{};
    est.valid = true;
    est.has_body_twist_state = true;
    est.body_twist_state.body_trans_mps = VelocityMps3{0.0, 0.0, 0.0};
    est.body_twist_state.twist_vel_radps = AngularVelocityRadPerSec3{0.0, 0.0, 0.0};

    SwingFootInputs inputs{};
    inputs.anchor = Vec3{0.18, 0.00, -0.11};
    inputs.stance_end = Vec3{0.12, 0.00, -0.11};
    inputs.v_liftoff_body = Vec3{0.0, 0.0, 0.0};
    inputs.tau01 = 0.50;
    inputs.swing_span = 0.50;
    inputs.f_hz = 1.0;
    inputs.step_length_m = 0.06;
    inputs.swing_height_m = 0.03;
    inputs.stance_lookahead_s = 0.24;
    inputs.swing_time_ease_01 = 1.0;
    inputs.static_stability_margin_m = 0.03;

    const BodyTwist nominal_body{Vec3{0.12, 0.0, 0.0}, Vec3{0.0, 0.0, 0.18}};
    const SwingFootPlanDecomposition steady = computeSwingFootPlacement(est, nominal_body, inputs);
    if (!expect(nearlyEqVec(steady.nominal_body, inputs.anchor),
                "nominal swing touchdown should return to the next stance anchor")) {
        return EXIT_FAILURE;
    }
    if (!expect(nearlyEqVec(steady.nominal_body, steady.final_body),
                "with no measured drift and healthy margin, swing foothold should stay nominal")) {
        return EXIT_FAILURE;
    }
    if (!expect(nearlyEqVec(steady.capture_body, Vec3{0.0, 0.0, 0.0}),
                "with no drift and healthy margin, capture correction should be zero")) {
        return EXIT_FAILURE;
    }

    inputs.static_stability_margin_m = 0.0;
    const SwingFootPlanDecomposition low_margin = computeSwingFootPlacement(est, nominal_body, inputs);
    if (!expect(nearlyEqVec(low_margin.nominal_body, steady.nominal_body),
                "nominal foothold should stay intent-driven regardless of stability margin")) {
        return EXIT_FAILURE;
    }
    if (!expect(low_margin.capture_body.x < steady.capture_body.x,
                "lower stability margin should increase inward capture")) {
        return EXIT_FAILURE;
    }

    est.body_twist_state.body_trans_mps = VelocityMps3{0.28, 0.0, 0.0};
    const SwingFootPlanDecomposition drifting = computeSwingFootPlacement(est, nominal_body, inputs);
    if (!expect(nearlyEqVec(drifting.nominal_body, steady.nominal_body),
                "nominal foothold should stay intent-driven regardless of measured drift")) {
        return EXIT_FAILURE;
    }
    if (!expect(drifting.capture_body.x < low_margin.capture_body.x,
                "larger forward drift should produce a larger backward capture correction")) {
        return EXIT_FAILURE;
    }

    est.body_twist_state.body_trans_mps = VelocityMps3{0.80, 0.0, 0.0};
    const SwingFootPlanDecomposition bounded = computeSwingFootPlacement(est, nominal_body, inputs);
    const double bounded_mag = std::hypot(bounded.capture_body.x, bounded.capture_body.y);
    if (!expect(bounded_mag <= bounded.capture_limit_m + 1e-9,
                "capture correction should always remain bounded")) {
        return EXIT_FAILURE;
    }
    if (!expect(nearlyEqVec(bounded.final_body, bounded.nominal_body + bounded.capture_body),
                "final foothold should equal nominal plus bounded capture")) {
        return EXIT_FAILURE;
    }

    // With no capture correction, swing touchdown and the following phase-zero stance must
    // meet in both position and velocity. This guards against target snaps that turn intended
    // propulsion into foot slip in the physics simulation.
    RobotState no_capture_est{};
    SwingFootInputs continuity = inputs;
    continuity.anchor = Vec3{0.18, 0.02, -0.11};
    continuity.stance_end = Vec3{0.14, 0.02, -0.11};
    continuity.tau01 = 1.0;
    continuity.static_stability_margin_m = 0.03;
    continuity.swing_time_ease_01 = 0.85;
    const BodyTwist slow_forward{Vec3{0.04, 0.0, 0.0}, Vec3{}};
    Vec3 swing_touchdown{};
    Vec3 swing_touchdown_velocity{};
    planSwingFoot(no_capture_est,
                  slow_forward,
                  continuity,
                  swing_touchdown,
                  swing_touchdown_velocity);

    StanceFootInputs next_stance{};
    next_stance.anchor = continuity.anchor;
    next_stance.v_foot_body = supportFootVelocityAt(continuity.anchor, slow_forward);
    next_stance.phase = 0.0;
    next_stance.f_hz = continuity.f_hz;
    Vec3 stance_start{};
    Vec3 stance_start_velocity{};
    planStanceFoot(next_stance, stance_start, stance_start_velocity);
    if (!expect(nearlyEqVec(swing_touchdown, stance_start, 1e-8),
                "swing touchdown position should be continuous with the next stance")) {
        return EXIT_FAILURE;
    }
    if (!expect(nearlyEqVec(swing_touchdown_velocity, stance_start_velocity, 1e-8),
                "swing touchdown velocity should be continuous with the next stance")) {
        return EXIT_FAILURE;
    }

    // The low-speed velocity scale is below one. It must not shrink the gait's physical
    // clearance floor after gait-parameter validation has already accepted it.
    SwingFootInputs clearance = continuity;
    clearance.tau01 = 0.5;
    clearance.swing_height_m = 0.021;
    Vec3 apex{};
    Vec3 apex_velocity{};
    planSwingFoot(no_capture_est, slow_forward, clearance, apex, apex_velocity);
    if (!expect(apex.z >= clearance.anchor.z + clearance.swing_height_m - 1e-9,
                "low-speed swing should preserve the configured physical clearance floor")) {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
