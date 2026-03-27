#include "autonomy/modules/locomotion_interface.hpp"

namespace autonomy {

LocomotionInterfaceModuleShell::LocomotionInterfaceModuleShell()
    : AutonomyModuleStub("locomotion_interface") {}

LocomotionCommand LocomotionInterfaceModuleShell::dispatch(const MotionDecision& motion_decision,
                                                           const LocalPlan& local_plan,
                                                           ContractEnvelope envelope) {
    (void)envelope;
    if (motion_decision.allow_motion && local_plan.has_command) {
        last_command_ = LocomotionCommand{
            .sent = true,
            .target = local_plan.target,
            .reason = {},
        };
        return last_command_;
    }

    last_command_ = LocomotionCommand{
        .sent = false,
        .target = {},
        .reason = motion_decision.reason,
    };
    return last_command_;
}

LocomotionCommand LocomotionInterfaceModuleShell::lastCommand() const {
    return last_command_;
}

} // namespace autonomy
