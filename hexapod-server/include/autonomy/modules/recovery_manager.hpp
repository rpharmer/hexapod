#pragma once

#include "autonomy/module_stubs.hpp"
#include "autonomy/recovery_manager.hpp"

namespace autonomy {

class RecoveryManagerModuleShell : public AutonomyModuleStub {
public:
    explicit RecoveryManagerModuleShell(uint64_t retry_budget = 2);

    void reset();
    RecoveryDecision onNoProgress(bool triggered);
    [[nodiscard]] RecoveryDecision lastDecision() const;

private:
    RecoveryManager recovery_manager_;
    RecoveryDecision last_decision_{};
};

} // namespace autonomy
