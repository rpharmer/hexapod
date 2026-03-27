#include "autonomy/module_stubs.hpp"

namespace autonomy {

AutonomyModuleStub::AutonomyModuleStub(std::string module_name)
    : module_name_(std::move(module_name)) {
    health_.module_name = module_name_;
}

bool AutonomyModuleStub::init() {
    if (state_ != ModuleRunState::Stopped) {
        return false;
    }
    state_ = ModuleRunState::Initialized;
    health_.state = state_;
    health_.detail = "initialized";
    return true;
}

bool AutonomyModuleStub::start() {
    if (state_ != ModuleRunState::Initialized) {
        return false;
    }
    state_ = ModuleRunState::Running;
    health_.state = state_;
    health_.detail = "running";
    return true;
}

bool AutonomyModuleStub::step(uint64_t now_ms) {
    if (state_ != ModuleRunState::Running) {
        health_.healthy = false;
        health_.detail = "step called while not running";
        return false;
    }
    health_.healthy = true;
    health_.heartbeat_timestamp_ms = now_ms;
    return true;
}

void AutonomyModuleStub::stop() {
    state_ = ModuleRunState::Stopped;
    health_.state = state_;
    health_.detail = "stopped";
}

ModuleHealth AutonomyModuleStub::health() const {
    return health_;
}

ModuleRunState AutonomyModuleStub::state() const {
    return state_;
}

MissionExecutiveModule::MissionExecutiveModule()
    : AutonomyModuleStub("mission_executive") {}

NavigationManagerModule::NavigationManagerModule()
    : AutonomyModuleStub("navigation_manager") {}

RecoveryManagerModule::RecoveryManagerModule()
    : AutonomyModuleStub("recovery_manager") {}

MotionArbiterModule::MotionArbiterModule()
    : AutonomyModuleStub("motion_arbiter") {}

LocalizationModule::LocalizationModule()
    : AutonomyModuleStub("localization") {}

WorldModelModule::WorldModelModule()
    : AutonomyModuleStub("world_model") {}

TraversabilityAnalyzerModule::TraversabilityAnalyzerModule()
    : AutonomyModuleStub("traversability_analyzer") {}

LocomotionInterfaceModule::LocomotionInterfaceModule()
    : AutonomyModuleStub("locomotion_interface") {}

ProgressMonitorModule::ProgressMonitorModule()
    : AutonomyModuleStub("progress_monitor") {}

MissionScriptingModule::MissionScriptingModule()
    : AutonomyModuleStub("mission_scripting") {}

} // namespace autonomy
