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

} // namespace

int main() {
    if (!testAllShellsLifecycleAndIdentity() ||
        !testAllShellsFunctionalWiring() ||
        !testLocomotionShellDispatchSinkOutcomes()) {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
