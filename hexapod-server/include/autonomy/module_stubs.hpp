#pragma once

#include <cstdint>
#include <string>

namespace autonomy {

enum class ModuleRunState {
    Stopped,
    Initialized,
    Running,
};

struct ModuleHealth {
    std::string module_name{};
    ModuleRunState state{ModuleRunState::Stopped};
    bool healthy{true};
    uint64_t heartbeat_timestamp_ms{0};
    std::string detail{"ok"};
};

class AutonomyModuleStub {
public:
    explicit AutonomyModuleStub(std::string module_name);

    virtual ~AutonomyModuleStub() = default;

    virtual bool init();
    virtual bool start();
    virtual bool step(uint64_t now_ms);
    virtual void stop();

    [[nodiscard]] ModuleHealth health() const;
    [[nodiscard]] ModuleRunState state() const;

protected:
    std::string module_name_;
    ModuleRunState state_{ModuleRunState::Stopped};
    ModuleHealth health_{};
};

class MissionExecutiveModule : public AutonomyModuleStub {
public:
    MissionExecutiveModule();
};

class NavigationManagerModule : public AutonomyModuleStub {
public:
    NavigationManagerModule();
};

class RecoveryManagerModule : public AutonomyModuleStub {
public:
    RecoveryManagerModule();
};

class MotionArbiterModule : public AutonomyModuleStub {
public:
    MotionArbiterModule();
};

class LocalizationModule : public AutonomyModuleStub {
public:
    LocalizationModule();
};

class WorldModelModule : public AutonomyModuleStub {
public:
    WorldModelModule();
};

class TraversabilityAnalyzerModule : public AutonomyModuleStub {
public:
    TraversabilityAnalyzerModule();
};

class LocomotionInterfaceModule : public AutonomyModuleStub {
public:
    LocomotionInterfaceModule();
};

class ProgressMonitorModule : public AutonomyModuleStub {
public:
    ProgressMonitorModule();
};

class MissionScriptingModule : public AutonomyModuleStub {
public:
    MissionScriptingModule();
};

} // namespace autonomy
