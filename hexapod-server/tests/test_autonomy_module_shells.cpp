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
    const auto local_plan = local_planner.plan(global_plan);
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
    const auto loc = localization.update(nav_update, 100);

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

} // namespace

int main() {
    if (!testAllShellsLifecycleAndIdentity() ||
        !testAllShellsFunctionalWiring() ||
        !testTerrainRiskInputsAffectPlanning()) {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
