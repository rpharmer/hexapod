#include "autonomy/modules/recovery_manager.hpp"

namespace autonomy {

RecoveryManagerModuleShell::RecoveryManagerModuleShell(uint64_t retry_budget)
    : AutonomyModuleStub("recovery_manager"),
      recovery_manager_(retry_budget) {}

void RecoveryManagerModuleShell::reset() {
    recovery_manager_.reset();
    last_decision_ = {};
}

RecoveryDecision RecoveryManagerModuleShell::onNoProgress(bool triggered) {
    last_decision_ = triggered ? recovery_manager_.onNoProgress() : RecoveryDecision{};
    return last_decision_;
}

RecoveryDecision RecoveryManagerModuleShell::lastDecision() const {
    return last_decision_;
}

} // namespace autonomy
