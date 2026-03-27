#include "autonomy/modules/recovery_manager.hpp"
#include "autonomy/modules/shell_utils.hpp"

namespace autonomy {

RecoveryManagerModuleShell::RecoveryManagerModuleShell(uint64_t retry_budget)
    : AutonomyModuleStub("recovery_manager"),
      recovery_manager_(retry_budget) {}

void RecoveryManagerModuleShell::reset() {
    recovery_manager_.reset();
    last_decision_ = {};
}

RecoveryDecision RecoveryManagerModuleShell::onNoProgress(bool triggered,
                                                          ContractEnvelope envelope) {
    (void)envelope;
    return modules::storeLast(last_decision_,
                              triggered ? recovery_manager_.onNoProgress() : RecoveryDecision{});
}

RecoveryDecision RecoveryManagerModuleShell::lastDecision() const {
    return last_decision_;
}

} // namespace autonomy
