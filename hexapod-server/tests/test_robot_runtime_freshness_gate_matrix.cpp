#include "estimator.hpp"
#include "robot_runtime.hpp"
#include "sim_hardware_bridge.hpp"

#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace {

bool expect(bool condition, const std::string& message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

class ScriptedEstimator final : public IEstimator {
public:
    void enqueue(const RobotState& sample) { queue_.push_back(sample); }

    RobotState update(const RobotState& raw) override {
        if (!queue_.empty()) {
            last_ = queue_.front();
            queue_.erase(queue_.begin());
            return last_;
        }

        if (last_.sample_id == 0) {
            last_.sample_id = raw.sample_id;
            last_.timestamp_us = raw.timestamp_us;
        }
        return last_;
    }

private:
    std::vector<RobotState> queue_{};
    RobotState last_{};
};

control_config::ControlConfig strictFreshnessConfig() {
    control_config::ControlConfig cfg{};
    cfg.freshness.estimator.max_allowed_age_us = DurationUs{1'000};
    cfg.freshness.intent.max_allowed_age_us = DurationUs{1'000};
    return cfg;
}

RobotState makeEstimatorSample(uint64_t sample_id, TimePointUs timestamp_us) {
    RobotState est{};
    est.sample_id = sample_id;
    est.timestamp_us = timestamp_us;
    return est;
}

MotionIntent makeIntentSample(uint64_t sample_id, TimePointUs timestamp_us) {
    MotionIntent intent{};
    intent.requested_mode = RobotMode::WALK;
    intent.sample_id = sample_id;
    intent.timestamp_us = timestamp_us;
    return intent;
}

void runControlStep(RobotRuntime& runtime) {
    runtime.busStep();
    runtime.estimatorStep();
    runtime.safetyStep();
    runtime.controlStep();
}

bool testEstimatorAndIntentFreshAllowsNominalControl() {
    auto bridge = std::make_unique<SimHardwareBridge>();
    auto estimator = std::make_unique<ScriptedEstimator>();
    auto* scripted = estimator.get();
    RobotRuntime runtime(std::move(bridge), std::move(estimator), nullptr, strictFreshnessConfig());

    if (!expect(runtime.init(), "runtime init should succeed")) {
        return false;
    }

    const TimePointUs now = now_us();
    scripted->enqueue(makeEstimatorSample(10, now));
    runtime.setMotionIntent(makeIntentSample(10, now));
    runControlStep(runtime);

    const ControlStatus status = runtime.getStatus();
    return expect(status.active_mode == RobotMode::WALK, "fresh samples should allow WALK mode");
}

bool testStaleEstimatorTriggersEstimatorInvalidFault() {
    auto bridge = std::make_unique<SimHardwareBridge>();
    auto estimator = std::make_unique<ScriptedEstimator>();
    auto* scripted = estimator.get();
    RobotRuntime runtime(std::move(bridge), std::move(estimator), nullptr, strictFreshnessConfig());

    if (!expect(runtime.init(), "runtime init should succeed")) {
        return false;
    }

    const TimePointUs now = now_us();
    scripted->enqueue(makeEstimatorSample(20, now));
    runtime.setMotionIntent(makeIntentSample(20, now));
    runControlStep(runtime);

    scripted->enqueue(makeEstimatorSample(21, TimePointUs{1}));
    runtime.setMotionIntent(makeIntentSample(21, now_us()));
    runControlStep(runtime);

    const ControlStatus status = runtime.getStatus();
    return expect(status.active_mode == RobotMode::SAFE_IDLE,
                  "stale estimator should force SAFE_IDLE") &&
           expect(status.active_fault == FaultCode::ESTIMATOR_INVALID,
                  "stale estimator should set ESTIMATOR_INVALID");
}

bool testStaleIntentTriggersCommandTimeoutFault() {
    auto bridge = std::make_unique<SimHardwareBridge>();
    auto estimator = std::make_unique<ScriptedEstimator>();
    auto* scripted = estimator.get();
    RobotRuntime runtime(std::move(bridge), std::move(estimator), nullptr, strictFreshnessConfig());

    if (!expect(runtime.init(), "runtime init should succeed")) {
        return false;
    }

    const TimePointUs now = now_us();
    scripted->enqueue(makeEstimatorSample(30, now));
    runtime.setMotionIntent(makeIntentSample(30, now));
    runControlStep(runtime);

    scripted->enqueue(makeEstimatorSample(31, now_us()));
    runtime.setMotionIntentForTest(makeIntentSample(31, TimePointUs{1}));
    runControlStep(runtime);

    const ControlStatus status = runtime.getStatus();
    return expect(status.active_mode == RobotMode::SAFE_IDLE,
                  "stale intent should force SAFE_IDLE") &&
           expect(status.active_fault == FaultCode::COMMAND_TIMEOUT,
                  "stale intent should set COMMAND_TIMEOUT");
}

bool testStaleEstimatorAndIntentPreferEstimatorFault() {
    auto bridge = std::make_unique<SimHardwareBridge>();
    auto estimator = std::make_unique<ScriptedEstimator>();
    auto* scripted = estimator.get();
    RobotRuntime runtime(std::move(bridge), std::move(estimator), nullptr, strictFreshnessConfig());

    if (!expect(runtime.init(), "runtime init should succeed")) {
        return false;
    }

    const TimePointUs now = now_us();
    scripted->enqueue(makeEstimatorSample(40, now));
    runtime.setMotionIntent(makeIntentSample(40, now));
    runControlStep(runtime);

    scripted->enqueue(makeEstimatorSample(41, TimePointUs{1}));
    runtime.setMotionIntentForTest(makeIntentSample(41, TimePointUs{1}));
    runControlStep(runtime);

    const ControlStatus status = runtime.getStatus();
    return expect(status.active_mode == RobotMode::SAFE_IDLE,
                  "both stale streams should force SAFE_IDLE") &&
           expect(status.active_fault == FaultCode::ESTIMATOR_INVALID,
                  "ESTIMATOR_INVALID should take precedence when both streams are stale");
}

} // namespace

int main() {
    if (!testEstimatorAndIntentFreshAllowsNominalControl() ||
        !testStaleEstimatorTriggersEstimatorInvalidFault() ||
        !testStaleIntentTriggersCommandTimeoutFault() ||
        !testStaleEstimatorAndIntentPreferEstimatorFault()) {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
