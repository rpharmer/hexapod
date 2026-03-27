#include "autonomy/modules/motion_arbiter.hpp"

namespace autonomy {

MotionArbiterModuleShell::MotionArbiterModuleShell()
    : AutonomyModuleStub("motion_arbiter") {}

MotionDecision MotionArbiterModuleShell::arbitrate(bool estop,
                                                   bool hold,
                                                   bool recovery_active,
                                                   const NavigationUpdate& nav_update,
                                                   ContractEnvelope envelope) {
    (void)envelope;
    last_decision_ = motion_arbiter_.select(estop, hold, recovery_active, nav_update);
    return last_decision_;
}

MotionDecision MotionArbiterModuleShell::lastDecision() const {
    return last_decision_;
}

} // namespace autonomy
