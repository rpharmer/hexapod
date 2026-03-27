#pragma once

#include "autonomy/module_stubs.hpp"
#include "autonomy/common_types.hpp"
#include "autonomy/progress_monitor.hpp"

namespace autonomy {

class ProgressMonitorModuleShell : public AutonomyModuleStub {
public:
    explicit ProgressMonitorModuleShell(uint64_t no_progress_timeout_ms = 1000);

    void reset();
    ProgressEvaluation evaluate(const ProgressSample& sample,
                                ContractEnvelope envelope = {});
    [[nodiscard]] ProgressEvaluation lastEvaluation() const;

private:
    ProgressMonitor progress_monitor_;
    ProgressEvaluation last_evaluation_{};
};

} // namespace autonomy
