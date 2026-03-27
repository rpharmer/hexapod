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
    const auto loc = localization.update(nav_update, 10);
    if (!expect(loc.valid, "localization shell should synthesize valid estimate from nav intent")) {
        return false;
    }
    const auto world = world_model.update(loc, false, 10);
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
    return expect(!locomotion_command.sent, "locomotion shell should not dispatch when motion is gated");
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
    return true;
}

} // namespace

int main() {
    if (!testAllShellsLifecycleAndIdentity() ||
        !testAllShellsFunctionalWiring() ||
        !testPlannerReachableBlockedAndDegradedFlows()) {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
