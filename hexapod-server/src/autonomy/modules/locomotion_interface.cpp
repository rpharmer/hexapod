#include "autonomy/modules/locomotion_interface.hpp"

#include <utility>

namespace autonomy {

LocomotionInterfaceModuleShell::LocomotionInterfaceModuleShell(CommandSink command_sink)
    : AutonomyModuleStub("locomotion_interface"),
      command_sink_(std::move(command_sink)) {
    if (!command_sink_) {
        command_sink_ = [](const Waypoint&, std::string*) { return true; };
    }
}

LocomotionCommand LocomotionInterfaceModuleShell::dispatch(const MotionDecision& motion_decision,
                                                           const LocalPlan& local_plan) {
    if (motion_decision.allow_motion && local_plan.has_command) {
        std::string failure_reason{};
        const bool write_ok = command_sink_(local_plan.target, &failure_reason);
        last_command_ = LocomotionCommand{
            .status = write_ok ? LocomotionCommand::DispatchStatus::Dispatched
                               : LocomotionCommand::DispatchStatus::DispatchFailed,
            .sent = write_ok,
            .write_ok = write_ok,
            .target = local_plan.target,
            .reason = write_ok ? std::string{} : failure_reason,
        };
        return last_command_;
    }

    last_command_ = LocomotionCommand{
        .status = LocomotionCommand::DispatchStatus::Suppressed,
        .sent = false,
        .write_ok = false,
        .target = {},
        .reason = motion_decision.reason,
    };
    return last_command_;
}

LocomotionCommand LocomotionInterfaceModuleShell::lastCommand() const {
    return last_command_;
}

} // namespace autonomy
