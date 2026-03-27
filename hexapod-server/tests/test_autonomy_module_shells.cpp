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

} // namespace

int main() {
    if (!testAllShellsLifecycleAndIdentity()) {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
