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
        if (!expect(module->health().heartbeat_timestamp_ms.value == 1000, "step should set heartbeat")) {
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
           expect(estimate.timestamp_ms.value == 10, "nominal estimate should propagate estimator timestamp") &&
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
    const auto world = world_model.update(
        loc,
        autonomy::MapSliceInput{
            .has_occupancy = true,
            .has_elevation = true,
            .has_risk_confidence = true,
            .occupancy = 0.2,
            .elevation_m = 0.05,
            .risk_confidence = 0.9,
        },
        false,
        10);
    const auto traverse = traversability.analyze(world, 10);
    const auto global_plan = global_planner.plan(nav_update, traverse);
    const auto local_plan = local_planner.plan(global_plan, false, 10);
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
    return expect(!locomotion_command.sent, "locomotion shell should not dispatch when motion is gated") &&
           expect(locomotion_command.status == autonomy::LocomotionCommand::DispatchStatus::Suppressed,
                  "suppressed locomotion dispatch should report suppressed status");
}

bool testLocomotionShellDispatchSinkOutcomes() {
    bool success_sink_called = false;
    autonomy::LocomotionInterfaceModuleShell success_locomotion(
        [&](const autonomy::Waypoint&, std::string*) {
            success_sink_called = true;
            return true;
        });
    const autonomy::MotionDecision allow_motion{
        .source = autonomy::MotionSource::Nav,
        .allow_motion = true,
        .reason = {},
    };
    const autonomy::LocalPlan local_plan{
        .has_command = true,
        .target = autonomy::Waypoint{.frame_id = "map", .x_m = 0.5, .y_m = -0.5, .yaw_rad = 0.1},
    };
    const auto success = success_locomotion.dispatch(allow_motion, local_plan);
    if (!expect(success_sink_called, "success sink should be invoked")) {
        return false;
    }
    if (!expect(success.status == autonomy::LocomotionCommand::DispatchStatus::Dispatched,
                "successful sink write should report dispatched status")) {
        return false;
    }

    bool failure_sink_called = false;
    autonomy::LocomotionInterfaceModuleShell failed_locomotion(
        [&](const autonomy::Waypoint&, std::string* failure_reason) {
            failure_sink_called = true;
            if (failure_reason) {
                *failure_reason = "simulated hardware failure";
            }
            return false;
        });
    const auto failed = failed_locomotion.dispatch(allow_motion, local_plan);
    return expect(failure_sink_called, "failure sink should be invoked") &&
           expect(failed.status == autonomy::LocomotionCommand::DispatchStatus::DispatchFailed,
                  "failing sink write should report dispatch failure status") &&
           expect(!failed.write_ok, "failing sink write should set write_ok=false");
}



bool testTerrainRiskInputsAffectPlanning() {
    autonomy::NavigationManagerModuleShell navigation;
    autonomy::LocalizationModuleShell localization;
    autonomy::WorldModelModuleShell world_model;
    autonomy::TraversabilityAnalyzerModuleShell traversability;
    autonomy::GlobalPlannerModuleShell global_planner;

    const autonomy::WaypointMission mission{
        .mission_id = "terrain-risk-mission",
        .waypoints = {autonomy::Waypoint{.frame_id = "map", .x_m = 2.0, .y_m = 1.0, .yaw_rad = 0.0}},
    };

    const auto nav_update = navigation.computeIntent(mission, 0, false);
    const auto loc = localization.update(
        autonomy::localizationObservationFromOdometry(0.0, 0.0, 0.0, 100, "map"),
        100);

    const auto nominal_world = world_model.update(
        loc,
        autonomy::MapSliceInput{
            .has_occupancy = true,
            .has_elevation = true,
            .has_risk_confidence = true,
            .occupancy = 0.1,
            .elevation_m = 0.02,
            .risk_confidence = 0.95,
        },
        false,
        100);
    const auto nominal_traversability = traversability.analyze(nominal_world, 100);
    const auto nominal_plan = global_planner.plan(nav_update, nominal_traversability);

    const auto risky_world = world_model.update(
        loc,
        autonomy::MapSliceInput{
            .has_occupancy = true,
            .has_elevation = true,
            .has_risk_confidence = true,
            .occupancy = 0.95,
            .elevation_m = 0.9,
            .risk_confidence = 0.1,
        },
        false,
        110);
    const auto risky_traversability = traversability.analyze(risky_world, 110);
    const auto risky_plan = global_planner.plan(nav_update, risky_traversability);

    return expect(nominal_plan.has_plan, "nominal terrain should remain plannable") &&
           expect(!risky_traversability.traversable, "high risk with low confidence should be non-traversable") &&
           expect(!risky_plan.has_plan, "risky terrain should suppress global plan");
}

bool testPlannerReachableBlockedAndDegradedFlows() {
    autonomy::GlobalPlannerModuleShell global_planner;
    autonomy::LocalPlannerModuleShell local_planner;

    const autonomy::NavigationUpdate nav_update{
        .has_intent = true,
        .intent = autonomy::NavigationIntent{
            .mission_id = "plan-test",
            .waypoint_index = 0,
            .target = autonomy::Waypoint{.frame_id = "map", .x_m = 4.0, .y_m = 2.0, .yaw_rad = 0.1},
        },
        .status = autonomy::NavigationStatus::Active,
        .reason = {},
    };

    const auto reachable = global_planner.plan(nav_update, autonomy::TraversabilityReport{
                                                               .traversable = true,
                                                               .cost = 0.2,
                                                               .timestamp_ms = 100,
                                                           });
    if (!expect(reachable.has_plan, "reachable traversability should produce a global plan")) {
        return false;
    }
    if (!expect(!reachable.route.empty(), "reachable plan should generate a route")) {
        return false;
    }
    if (!expect(reachable.status == autonomy::PlannerStatus::Ready, "reachable plan should be marked ready")) {
        return false;
    }

    const auto local_reachable = local_planner.plan(reachable, false, 200);
    if (!expect(local_reachable.has_command, "reachable global plan should produce local command")) {
        return false;
    }
    if (!expect(local_reachable.status == autonomy::PlannerStatus::Ready, "reachable local plan should be ready")) {
        return false;
    }
    if (!expect(local_reachable.target.x_m != reachable.route.front().x_m ||
                    local_reachable.target.y_m != reachable.route.front().y_m,
                "local short horizon target should not always be the first waypoint")) {
        return false;
    }

    const auto blocked_global = global_planner.plan(nav_update, autonomy::TraversabilityReport{
                                                                    .traversable = false,
                                                                    .cost = 1.0,
                                                                    .timestamp_ms = 250,
                                                                });
    if (!expect(!blocked_global.has_plan, "non-traversable space should yield no global plan")) {
        return false;
    }
    if (!expect(blocked_global.status == autonomy::PlannerStatus::UnsafePlan,
                "non-traversable plan should be unsafe")) {
        return false;
    }

    const auto blocked_local = local_planner.plan(blocked_global, true, 260);
    if (!expect(!blocked_local.has_command, "blocked local flow should suppress commands")) {
        return false;
    }
    if (!expect(blocked_local.status == autonomy::PlannerStatus::UnsafePlan,
                "blocked local flow should be marked unsafe")) {
        return false;
    }

    const auto degraded_global = global_planner.plan(nav_update, autonomy::TraversabilityReport{
                                                                     .traversable = true,
                                                                     .cost = 0.8,
                                                                     .timestamp_ms = 400,
                                                                 });
    if (!expect(degraded_global.has_plan, "high cost but traversable path should still produce plan")) {
        return false;
    }
    if (!expect(degraded_global.status == autonomy::PlannerStatus::Degraded,
                "high cost route should be marked degraded")) {
        return false;
    }

    const auto degraded_local = local_planner.plan(degraded_global, false, 401);
    if (!expect(degraded_local.has_command, "degraded global plan should still produce local command")) {
        return false;
    }
    if (!expect(degraded_local.status == autonomy::PlannerStatus::Degraded,
                "degraded global plan should propagate degraded local status")) {
        return false;
    }

    const auto stale_local = local_planner.plan(degraded_global, false, 1500);
    if (!expect(stale_local.fallback_active, "stale planning should activate fallback handling")) {
        return false;
    }
    if (!expect(stale_local.status == autonomy::PlannerStatus::StalePlan,
                "stale planning should mark local plan stale")) {
        return false;
    }

    const auto replanned_after_unblock = global_planner.plan(nav_update, autonomy::TraversabilityReport{
                                                                          .traversable = true,
                                                                          .cost = 0.15,
                                                                          .risk = 0.15,
                                                                          .timestamp_ms = 1600,
                                                                      });
    if (!expect(replanned_after_unblock.has_plan, "unblocked traversability should trigger replanning")) {
        return false;
    }
    if (!expect(replanned_after_unblock.status == autonomy::PlannerStatus::Ready,
                "replanned route after unblock should return to ready status")) {
        return false;
    }
    if (!expect(replanned_after_unblock.reason == "route-ready",
                "replanned route should keep stable ready telemetry reason")) {
        return false;
    }
    return true;
}

bool testPlannerNoIntentUnsafeAndStaleNoFallback() {
    autonomy::GlobalPlannerModuleShell global_planner;
    autonomy::LocalPlannerModuleShell local_planner;

    const auto no_intent = global_planner.plan(autonomy::NavigationUpdate{},
                                               autonomy::TraversabilityReport{
                                                   .traversable = true,
                                                   .cost = 0.2,
                                                   .timestamp_ms = 50,
                                               });
    if (!expect(!no_intent.has_plan, "missing navigation intent should not produce global plan")) {
        return false;
    }
    if (!expect(no_intent.status == autonomy::PlannerStatus::NoPlan,
                "missing navigation intent should surface no-plan status")) {
        return false;
    }
    if (!expect(no_intent.reason == "no-navigation-intent",
                "missing navigation intent should preserve no-intent reason")) {
        return false;
    }

    const auto local_no_intent = local_planner.plan(no_intent, false, 100);
    if (!expect(local_no_intent.status == autonomy::PlannerStatus::NoPlan,
                "no-intent global plan should map to no-plan local status")) {
        return false;
    }
    if (!expect(local_no_intent.reason == "no-navigation-intent",
                "no-intent local plan should preserve telemetry reason")) {
        return false;
    }

    const autonomy::NavigationUpdate nav_update{
        .has_intent = true,
        .intent = autonomy::NavigationIntent{
            .mission_id = "unsafe-terrain",
            .waypoint_index = 0,
            .target = autonomy::Waypoint{.frame_id = "map", .x_m = 2.0, .y_m = 1.0, .yaw_rad = 0.0},
        },
        .status = autonomy::NavigationStatus::Active,
        .reason = {},
    };
    const auto unsafe_plan = global_planner.plan(nav_update, autonomy::TraversabilityReport{
                                                                 .traversable = true,
                                                                 .cost = 0.99,
                                                                 .risk = 0.9,
                                                                 .timestamp_ms = 120,
                                                             });
    if (!expect(!unsafe_plan.has_plan, "unsafe terrain constraints should suppress global plan")) {
        return false;
    }
    if (!expect(unsafe_plan.status == autonomy::PlannerStatus::UnsafePlan,
                "unsafe terrain constraints should report unsafe status")) {
        return false;
    }
    if (!expect(unsafe_plan.reason == "unsafe-traversability",
                "unsafe terrain constraints should preserve telemetry reason")) {
        return false;
    }

    const autonomy::GlobalPlan stale_plan{
        .has_plan = true,
        .target = autonomy::Waypoint{.frame_id = "map", .x_m = 1.0, .y_m = 0.0, .yaw_rad = 0.0},
        .route = {autonomy::Waypoint{.frame_id = "map", .x_m = 0.2, .y_m = 0.0, .yaw_rad = 0.0}},
        .cost = 0.5,
        .status = autonomy::PlannerStatus::Ready,
        .reason = "route-ready",
        .source_timestamp_ms = 200,
    };
    const auto stale_without_fallback = autonomy::LocalPlannerModuleShell{}.plan(stale_plan, false, 2000);
    if (!expect(!stale_without_fallback.has_command, "stale plan without prior command should not emit target")) {
        return false;
    }
    if (!expect(stale_without_fallback.status == autonomy::PlannerStatus::StalePlan,
                "stale plan should report stale status")) {
        return false;
    }
    if (!expect(stale_without_fallback.reason == "stale-global-plan-no-fallback",
                "stale plan without prior command should keep stable fallback reason")) {
        return false;
    }
    return true;
}

} // namespace

int main() {
    if (!testAllShellsLifecycleAndIdentity() ||
        !testLocalizationAdapterNominalEstimatorPropagation() ||
        !testLocalizationRejectsStaleObservation() ||
        !testLocalizationRejectsFrameMismatch() ||
        !testAllShellsFunctionalWiring() ||
        !testLocomotionShellDispatchSinkOutcomes() ||
        !testTerrainRiskInputsAffectPlanning() ||
        !testPlannerReachableBlockedAndDegradedFlows() ||
        !testPlannerNoIntentUnsafeAndStaleNoFallback()) {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
