#include "autonomy/modules/global_planner.hpp"
#include "autonomy/modules/local_planner.hpp"
#include "autonomy/modules/locomotion_interface.hpp"
#include "autonomy/modules/traversability_analyzer.hpp"
#include "autonomy/motion_arbiter.hpp"

#include "autonomy_planner_regression_fixtures.hpp"

#include <cstdlib>
#include <iostream>

namespace {

bool expect(bool condition, const std::string& message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

autonomy::NavigationUpdate makeNavUpdate() {
    return autonomy::NavigationUpdate{
        .has_intent = true,
        .intent = autonomy::NavigationIntent{
            .mission_id = "regression-fixture-mission",
            .waypoint_index = 0,
            .target = autonomy::Waypoint{.frame_id = "map", .x_m = 3.0, .y_m = 1.5, .yaw_rad = 0.2},
        },
        .status = autonomy::NavigationStatus::Active,
        .reason = {},
    };
}

bool testTerrainFixturesCoverNominalThresholdAndAdversarial() {
    autonomy::TraversabilityAnalyzerModuleShell traversability;
    autonomy::GlobalPlannerModuleShell global_planner;
    const auto fixtures = autonomy_regression_fixtures::plannerTerrainFixtures();

    for (std::size_t index = 0; index < fixtures.size(); ++index) {
        const auto& fixture = fixtures[index];
        const auto report = traversability.analyze(fixture.world, 100 + static_cast<uint64_t>(index));
        const auto global_plan = global_planner.plan(makeNavUpdate(), report);

        if (!expect(report.traversable == fixture.expect_traversable,
                    fixture.name + ": traversability expectation mismatch")) {
            return false;
        }
        if (!expect(report.traversable || !global_plan.has_plan,
                    fixture.name + ": non-traversable terrain must not produce a global plan")) {
            return false;
        }
    }

    return true;
}

bool testDegradedContractStillPreventsUnsafeDispatch() {
    autonomy::GlobalPlannerModuleShell global_planner;
    autonomy::LocalPlannerModuleShell local_planner;
    autonomy::LocomotionInterfaceModuleShell locomotion;

    const auto allow_motion = autonomy::MotionDecision{
        .source = autonomy::MotionSource::Nav,
        .allow_motion = true,
        .reason = {},
    };

    const auto degraded_report = autonomy::TraversabilityReport{
        .traversable = true,
        .cost = 0.78,
        .risk = 0.55,
        .confidence = 0.45,
        .reason = "traversable",
        .timestamp_ms = 200,
    };
    const auto near_threshold_global = global_planner.plan(makeNavUpdate(), degraded_report);
    const auto near_threshold_local = local_planner.plan(near_threshold_global, false, 220);
    const auto near_threshold_dispatch = locomotion.dispatch(allow_motion, near_threshold_local);

    if (!expect(near_threshold_global.status == autonomy::PlannerStatus::Degraded,
                "near-threshold fixture should stay in degraded mode")) {
        return false;
    }
    if (!expect(near_threshold_dispatch.sent,
                "degraded mode should still dispatch when a feasible local target exists")) {
        return false;
    }

    autonomy::TraversabilityAnalyzerModuleShell traversability;
    const auto fixtures = autonomy_regression_fixtures::plannerTerrainFixtures();
    const auto adversarial = fixtures[2];
    const auto adversarial_report = traversability.analyze(adversarial.world, 300);
    const auto adversarial_global = global_planner.plan(makeNavUpdate(), adversarial_report);
    const auto adversarial_local = local_planner.plan(adversarial_global, false, 320);
    const auto adversarial_dispatch = locomotion.dispatch(allow_motion, adversarial_local);

    return expect(!adversarial_global.has_plan, "adversarial fixture should have no global plan") &&
           expect(!adversarial_local.has_command, "adversarial fixture should have no local command") &&
           expect(!adversarial_dispatch.sent, "adversarial fixture must suppress unsafe dispatch") &&
           expect(adversarial_dispatch.status == autonomy::LocomotionCommand::DispatchStatus::Suppressed,
                  "adversarial fixture should report suppressed dispatch status");
}

} // namespace

int main() {
    if (!testTerrainFixturesCoverNominalThresholdAndAdversarial() ||
        !testDegradedContractStillPreventsUnsafeDispatch()) {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
