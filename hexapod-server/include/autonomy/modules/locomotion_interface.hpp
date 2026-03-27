#pragma once

#include "autonomy/module_stubs.hpp"
#include "autonomy/common_types.hpp"
#include "autonomy/modules/module_data.hpp"
#include "autonomy/motion_arbiter.hpp"

namespace autonomy {

class LocomotionInterfaceModuleShell : public AutonomyModuleStub {
public:
    LocomotionInterfaceModuleShell();

    LocomotionCommand dispatch(const MotionDecision& motion_decision,
                               const LocalPlan& local_plan,
                               ContractEnvelope envelope = {});
    [[nodiscard]] LocomotionCommand lastCommand() const;

private:
    LocomotionCommand last_command_{};
};

} // namespace autonomy
