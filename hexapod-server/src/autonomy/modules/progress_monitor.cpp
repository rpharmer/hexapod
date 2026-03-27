#include "autonomy/modules/progress_monitor.hpp"

namespace autonomy {

ProgressMonitorModuleShell::ProgressMonitorModuleShell(uint64_t no_progress_timeout_ms)
    : AutonomyModuleStub("progress_monitor"),
      progress_monitor_(no_progress_timeout_ms) {}

void ProgressMonitorModuleShell::reset() {
    progress_monitor_.reset();
    last_evaluation_ = {};
}

ProgressEvaluation ProgressMonitorModuleShell::evaluate(const ProgressSample& sample) {
    last_evaluation_ = progress_monitor_.evaluate(sample);
    return last_evaluation_;
}

ProgressEvaluation ProgressMonitorModuleShell::lastEvaluation() const {
    return last_evaluation_;
}

} // namespace autonomy
