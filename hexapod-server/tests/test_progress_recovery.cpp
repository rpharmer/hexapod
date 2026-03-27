#include "autonomy/progress_monitor.hpp"
#include "autonomy/recovery_manager.hpp"

#include <cstdlib>
#include <iostream>

namespace {

bool expect(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

bool testNoProgressTriggersEscalation() {
    autonomy::ProgressMonitor monitor(100);
    autonomy::RecoveryManager recovery(2);

    const auto baseline = monitor.evaluate(autonomy::ProgressSample{.timestamp_ms = 0, .completed_waypoints = 0});
    if (!expect(!baseline.no_progress, "first sample should not trigger no progress")) {
        return false;
    }

    const auto stale = monitor.evaluate(autonomy::ProgressSample{.timestamp_ms = 150, .completed_waypoints = 0});
    if (!expect(stale.no_progress, "stale progress should trigger no-progress")) {
        return false;
    }

    const auto step1 = recovery.onNoProgress();
    const auto step2 = recovery.onNoProgress();
    const auto step3 = recovery.onNoProgress();
    const auto step4 = recovery.onNoProgress();

    return expect(step1.action == autonomy::RecoveryAction::Hold, "first recovery step should hold") &&
           expect(step2.action == autonomy::RecoveryAction::Retry, "second recovery step should retry") &&
           expect(step3.action == autonomy::RecoveryAction::Replan, "third recovery step should replan") &&
           expect(step4.action == autonomy::RecoveryAction::Abort && step4.mission_should_abort,
                  "final recovery step should abort mission");
}

} // namespace

int main() {
    if (!testNoProgressTriggersEscalation()) {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
