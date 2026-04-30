#include "body_controller.hpp"
#include "geometry_config.hpp"
#include "leg_ik.hpp"
#include "leg_fk.hpp"
#include "motion_intent_utils.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {

bool expect(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
    }
    return condition;
}

double maxAbsComponent(const Vec3& v) {
    return std::max({std::abs(v.x), std::abs(v.y), std::abs(v.z)});
}

JointTargets buildStandTargets() {
    BodyController body_controller{};
    LegIK leg_ik(defaultHexapodGeometry());

    RobotState est{};
    SafetyState safety{};
    safety.inhibit_motion = false;
    safety.torque_cut = false;
    safety.leg_enabled.fill(true);

    MotionIntent stand = makeMotionIntent(RobotMode::STAND, GaitType::TRIPOD, 0.14);
    GaitState gait{};
    const BodyTwist cmd_twist = rawLocomotionTwistFromIntent(stand, planarMotionCommand(stand));
    const LegTargets foot_targets = body_controller.update(est, stand, gait, safety, cmd_twist, nullptr);
    return leg_ik.solve(est, foot_targets, safety);
}

const std::array<Vec3, kNumLegs> kExpectedFootBodyFrame{{
    {-0.0719915, 0.0182234, 0.0701464},
    {-0.197992, -0.185223, -0.0841464},
    {0.0815, 0.169028, 0.0701464},
    {-0.0815, -0.169028, -0.0841464},
    {0.197992, 0.185223, 0.0701464},
    {0.0719915, -0.0182234, -0.0841464},
}};

} // namespace

int main() {
    const HexapodGeometry geometry = defaultHexapodGeometry();
    const JointTargets stand_targets = buildStandTargets();
    LegFK fk{};

    bool ok = true;
    double max_foot_error_m = 0.0;

    for (int leg = 0; leg < kNumLegs; ++leg) {
        const Vec3 actual_foot_body =
            fk.footInBodyFrame(stand_targets.leg_states[static_cast<std::size_t>(leg)],
                               geometry.legGeometry[static_cast<std::size_t>(leg)]).pos_body_m;
        const Vec3 expected_foot_body = kExpectedFootBodyFrame[static_cast<std::size_t>(leg)];
        const Vec3 err = actual_foot_body - expected_foot_body;

        constexpr double kFootEps = 1.0e-4;
        max_foot_error_m = std::max(max_foot_error_m, maxAbsComponent(err));

        if (!expect(maxAbsComponent(err) <= kFootEps,
                    "server nominal stand footprint should stay fixed")) {
            std::cerr << "leg=" << leg
                      << " actual_foot=(" << actual_foot_body.x << "," << actual_foot_body.y << ","
                      << actual_foot_body.z << ")"
                      << " expected_foot=(" << expected_foot_body.x << "," << expected_foot_body.y << ","
                      << expected_foot_body.z << ")"
                      << " err=(" << err.x << "," << err.y << "," << err.z << ")\n";
            ok = false;
        }
    }

    if (!ok) {
        std::cout << "test_physics_sim_server_initial_layout detected a footprint mismatch; see per-leg errors above\n";
    }
    std::cout << "test_physics_sim_server_initial_layout ok max_foot_error_m=" << max_foot_error_m << '\n';
    return ok ? 0 : 1;
}
