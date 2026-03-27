#include "autonomy/modules/module_shells.hpp"

#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace {

bool expect(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

RobotState makeEstimatorState(uint64_t timestamp_us,
                              double x_m,
                              double y_m,
                              double yaw_rad,
                              bool valid = true,
                              bool has_body_pose = true) {
    RobotState est{};
    est.valid = valid;
    est.has_body_pose_state = has_body_pose;
    est.timestamp_us = TimePointUs{timestamp_us};
    est.body_pose_state.body_trans_m.x = x_m;
    est.body_pose_state.body_trans_m.y = y_m;
    est.body_pose_state.orientation_rad.z = yaw_rad;
    return est;
}

bool testAllShellsLifecycleAndIdentity() {
    std::vector<std::pair<std::string, std::unique_ptr<autonomy::AutonomyModuleStub>>> modules;
    modules.emplace_back("navigation_manager", std::make_unique<autonomy::NavigationManagerModuleShell>());
    modules.emplace_back("recovery_manager", std::make_unique<autonomy::RecoveryManagerModuleShell>());
    modules.emplace_back("motion_arbiter", std::make_unique<autonomy::MotionArbiterModuleShell>());
    modules.emplace_back("localization", std::make_unique<autonomy::LocalizationModuleShell>());
    modules.emplace_back("world_model", std::make_unique<autonomy::WorldModelModuleShell>());
    modules.emplace_back("traversability_analyzer", std::make_unique<autonomy::TraversabilityAnalyzerModuleShell>());
    modules.emplace_back("locomotion_interface", std::make_unique<autonomy::LocomotionInterfaceModuleShell>());
    modules.emplace_back("global_planner", std::make_unique<autonomy::GlobalPlannerModuleShell>());
    modules.emplace_back("local_planner", std::make_unique<autonomy::LocalPlannerModuleShell>());
    modules.emplace_back("progress_monitor", std::make_unique<autonomy::ProgressMonitorModuleShell>());

    for (auto& [name, module] : modules) {
        if (!expect(module->health().module_name == name, "module name should match shell identity")) {
            return false;
        }
        if (!expect(module->init(), "init should succeed")) {
            return false;
        }
        if (!expect(module->start(), "start should succeed")) {
            return false;
        }
        if (!expect(module->step(1000), "step should succeed")) {
            return false;
        }
        if (!expect(module->health().heartbeat_timestamp_ms == 1000, "step should set heartbeat")) {
            return false;
        }
        module->stop();
        if (!expect(module->state() == autonomy::ModuleRunState::Stopped, "stop should set stopped state")) {
            return false;
        }
    }

    return true;
}

bool testLocalizationAdapterNominalEstimatorPropagation() {
    autonomy::LocalizationModuleShell localization;
    const RobotState estimator_state = makeEstimatorState(/*timestamp_us=*/10'000, /*x_m=*/1.25, /*y_m=*/-0.5, /*yaw_rad=*/0.75);
    const auto observation = autonomy::localizationObservationFromEstimator(estimator_state, "map");

    const auto estimate = localization.update(observation, /*now_ms=*/20);
    return expect(estimate.valid, "nominal estimator observation should produce a valid localization estimate") &&
           expect(estimate.frame_id == "map", "nominal estimate should keep the map frame") &&
           expect(estimate.timestamp_ms == 10, "nominal estimate should propagate estimator timestamp") &&
           expect(estimate.x_m == 1.25 && estimate.y_m == -0.5 && estimate.yaw_rad == 0.75,
                  "nominal estimate should propagate estimator pose");
}

bool testLocalizationRejectsStaleObservation() {
    autonomy::LocalizationModuleShell localization("map", /*stale_threshold_ms=*/50);
    const auto stale_observation = autonomy::localizationObservationFromOdometry(
        /*x_m=*/0.2,
        /*y_m=*/0.3,
        /*yaw_rad=*/0.4,
        /*timestamp_ms=*/100,
        "map");

    const auto estimate = localization.update(stale_observation, /*now_ms=*/200);
    return expect(!estimate.valid, "stale localization observation should be rejected") &&
           expect(estimate.frame_id == "map", "rejected stale observation should preserve expected frame");
}

bool testLocalizationRejectsFrameMismatch() {
    autonomy::LocalizationModuleShell localization("map", /*stale_threshold_ms=*/100);
    const auto wrong_frame_observation = autonomy::localizationObservationFromOdometry(
        /*x_m=*/1.0,
        /*y_m=*/2.0,
        /*yaw_rad=*/0.2,
        /*timestamp_ms=*/120,
        "odom");

    const auto estimate = localization.update(wrong_frame_observation, /*now_ms=*/150);
    return expect(!estimate.valid, "frame mismatch should invalidate localization observation") &&
           expect(estimate.frame_id == "map", "frame mismatch should keep localization estimate in expected frame");
}

bool testAllShellsFunctionalWiring() {
    autonomy::NavigationManagerModuleShell navigation;
    autonomy::LocalizationModuleShell localization;
    autonomy::WorldModelModuleShell world_model;
    autonomy::TraversabilityAnalyzerModuleShell traversability;
    autonomy::GlobalPlannerModuleShell global_planner;
    autonomy::LocalPlannerModuleShell local_planner;
    autonomy::ProgressMonitorModuleShell progress_monitor(50);
    autonomy::RecoveryManagerModuleShell recovery(1);
    autonomy::MotionArbiterModuleShell motion_arbiter;
    autonomy::LocomotionInterfaceModuleShell locomotion;

    const autonomy::WaypointMission mission{
        .mission_id = "module-shells-mission",
        .waypoints = {autonomy::Waypoint{.frame_id = "map", .x_m = 1.0, .y_m = 2.0, .yaw_rad = 0.25}},
    };

    const auto nav_update = navigation.computeIntent(mission, 0, false);
    if (!expect(nav_update.has_intent, "navigation shell should emit intent")) {
        return false;
    }

    const RobotState estimator_state = makeEstimatorState(/*timestamp_us=*/10'000, /*x_m=*/1.0, /*y_m=*/2.0, /*yaw_rad=*/0.25);
    const auto loc = localization.update(autonomy::localizationObservationFromEstimator(estimator_state), 20);
    if (!expect(loc.valid, "localization shell should synthesize valid estimate from estimator state")) {
        return false;
    }

    const auto world = world_model.update(loc, false, 10);
    const auto traverse = traversability.analyze(world, 10);
    const auto global_plan = global_planner.plan(nav_update, traverse);
    const auto local_plan = local_planner.plan(global_plan, false);
    if (!expect(local_plan.has_command, "local planner shell should emit command from global plan")) {
        return false;
    }

    const auto progress_a = progress_monitor.evaluate(autonomy::ProgressSample{.timestamp_ms = 0, .completed_waypoints = 0});
    if (!expect(!progress_a.no_progress, "first progress sample should not trigger no-progress")) {
        return false;
    }
    const auto progress_b = progress_monitor.evaluate(autonomy::ProgressSample{.timestamp_ms = 100, .completed_waypoints = 0});
    if (!expect(progress_b.no_progress, "stagnant progress should trigger no-progress")) {
        return false;
    }
    const auto recovery_decision = recovery.onNoProgress(progress_b.no_progress);
    if (!expect(recovery_decision.recovery_active, "recovery shell should activate recovery when triggered")) {
        return false;
    }

    const auto motion = motion_arbiter.arbitrate(false, false, recovery_decision.recovery_active, nav_update);
    if (!expect(!motion.allow_motion, "active recovery should gate motion")) {
        return false;
    }
    const auto locomotion_command = locomotion.dispatch(motion, local_plan);
    return expect(!locomotion_command.sent, "locomotion shell should not dispatch when motion is gated");
}

} // namespace

int main() {
    if (!testAllShellsLifecycleAndIdentity() ||
        !testLocalizationAdapterNominalEstimatorPropagation() ||
        !testLocalizationRejectsStaleObservation() ||
        !testLocalizationRejectsFrameMismatch() ||
        !testAllShellsFunctionalWiring()) {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
