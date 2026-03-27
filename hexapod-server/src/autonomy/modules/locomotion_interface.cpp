#include "autonomy/modules/locomotion_interface.hpp"
#include "autonomy/modules/shell_utils.hpp"

#include "utils/logger.hpp"

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
                                                           const LocalPlan& local_plan,
                                                           ContractEnvelope envelope) {
    (void)envelope;
    const auto logger = logging::GetDefaultLogger();

    if (motion_decision.allow_motion && local_plan.has_command) {
        std::string failure_reason{};
        const bool write_ok = command_sink_(local_plan.target, &failure_reason);
        const auto command = LocomotionCommand{
            .status = write_ok ? LocomotionCommand::DispatchStatus::Dispatched
                               : LocomotionCommand::DispatchStatus::DispatchFailed,
            .sent = write_ok,
            .write_ok = write_ok,
            .target = local_plan.target,
            .reason = write_ok ? std::string{} : failure_reason,
        };
        if (!write_ok && logger) {
            LOG_WARN(logger,
                     "[autonomy] locomotion dispatch failed; reason=",
                     failure_reason.empty() ? std::string{"unspecified"} : failure_reason);
        }
        return modules::storeLast(last_command_, command);
    }

    const auto command = LocomotionCommand{
        .status = LocomotionCommand::DispatchStatus::Suppressed,
        .sent = false,
        .write_ok = false,
        .target = {},
        .reason = motion_decision.allow_motion ? local_plan.reason : motion_decision.reason,
    };
    if (logger && !motion_decision.allow_motion) {
        LOG_DEBUG(logger, "[autonomy] locomotion command suppressed by motion arbiter: ", motion_decision.reason);
    }
    return modules::storeLast(last_command_, command);
}

LocomotionCommand LocomotionInterfaceModuleShell::lastCommand() const {
    return last_command_;
}

} // namespace autonomy
