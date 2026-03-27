#pragma once

#include "autonomy/module_stubs.hpp"
#include "autonomy/common_types.hpp"
#include "autonomy/modules/module_data.hpp"
#include "autonomy/motion_arbiter.hpp"

#include <functional>
#include <string>

namespace autonomy {

class LocomotionInterfaceModuleShell : public AutonomyModuleStub {
public:
    using CommandSink = std::function<bool(const Waypoint& target, std::string* failure_reason)>;

    explicit LocomotionInterfaceModuleShell(CommandSink command_sink = {});

    LocomotionCommand dispatch(const MotionDecision& motion_decision,
                               const LocalPlan& local_plan,
                               ContractEnvelope envelope = {});
    [[nodiscard]] LocomotionCommand lastCommand() const;

private:
    CommandSink command_sink_;
    LocomotionCommand last_command_{};
};

} // namespace autonomy
