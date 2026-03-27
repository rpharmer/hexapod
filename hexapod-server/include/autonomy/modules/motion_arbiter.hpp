#pragma once

#include "autonomy/module_stubs.hpp"
#include "autonomy/motion_arbiter.hpp"

namespace autonomy {

class MotionArbiterModuleShell : public AutonomyModuleStub {
public:
    MotionArbiterModuleShell();

    MotionDecision arbitrate(bool estop,
                             bool hold,
                             bool recovery_active,
                             const NavigationUpdate& nav_update);
    [[nodiscard]] MotionDecision lastDecision() const;

private:
    MotionArbiter motion_arbiter_{};
    MotionDecision last_decision_{};
};

} // namespace autonomy
