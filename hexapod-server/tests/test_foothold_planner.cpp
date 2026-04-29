#include "foot_planners.hpp"
#include "foothold_planner.hpp"

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
    inputs.stance_end = Vec3{0.12, 0.00, -0.11};
    inputs.v_liftoff_body = Vec3{0.0, 0.0, 0.0};
    inputs.tau01 = 0.50;
    inputs.swing_span = 0.50;
    inputs.f_hz = 1.0;
    inputs.step_length_m = 0.06;
    inputs.swing_height_m = 0.03;
    inputs.stride_ux = 1.0;
    inputs.stride_uy = 0.0;
    inputs.stance_lookahead_s = 0.24;
    inputs.swing_time_ease_01 = 1.0;
    inputs.static_stability_margin_m = 0.03;

    const BodyTwist nominal_body{Vec3{0.12, 0.0, 0.0}, Vec3{0.0, 0.0, 0.18}};
    const SwingFootPlanDecomposition steady = computeSwingFootPlacement(est, nominal_body, inputs);
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

    return EXIT_SUCCESS;
}
