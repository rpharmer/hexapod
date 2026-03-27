#include "autonomy/modules/motion_arbiter.hpp"
#include "autonomy/modules/shell_utils.hpp"

namespace autonomy {

MotionArbiterModuleShell::MotionArbiterModuleShell()
    : AutonomyModuleStub("motion_arbiter") {}

MotionDecision MotionArbiterModuleShell::arbitrate(bool estop,
                                                   bool hold,
                                                   bool recovery_active,
                                                   const NavigationUpdate& nav_update,
                                                   ContractEnvelope envelope) {
    (void)envelope;
    return modules::storeLast(last_decision_,
                              motion_arbiter_.select(estop, hold, recovery_active, nav_update));
}

MotionDecision MotionArbiterModuleShell::lastDecision() const {
    return last_decision_;
}

} // namespace autonomy
