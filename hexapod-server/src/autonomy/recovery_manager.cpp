#include "autonomy/recovery_manager.hpp"

namespace autonomy {

RecoveryManager::RecoveryManager(uint64_t retry_budget)
    : retry_budget_(retry_budget) {}

void RecoveryManager::reset() {
    attempts_ = 0;
}

RecoveryDecision RecoveryManager::onNoProgress() {
    ++attempts_;

    if (attempts_ == 1) {
        return RecoveryDecision{
            .action = RecoveryAction::Hold,
            .recovery_active = true,
            .mission_should_abort = false,
            .reason = "hold before retry",
        };
    }

    if (attempts_ <= retry_budget_) {
        return RecoveryDecision{
            .action = RecoveryAction::Retry,
            .recovery_active = true,
            .mission_should_abort = false,
            .reason = "retry trajectory",
        };
    }

    if (attempts_ == retry_budget_ + 1) {
        return RecoveryDecision{
            .action = RecoveryAction::Replan,
            .recovery_active = true,
            .mission_should_abort = false,
            .reason = "replan route",
        };
    }

    return RecoveryDecision{
        .action = RecoveryAction::Abort,
        .recovery_active = false,
        .mission_should_abort = true,
        .reason = "retry budget exhausted",
    };
}

} // namespace autonomy
