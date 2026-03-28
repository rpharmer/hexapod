#pragma once

#include "freshness_policy.hpp"
#include "logger.hpp"
#include "types.hpp"

#include <atomic>
#include <cstdint>
#include <memory>

class RuntimeFreshnessGate {
public:
    enum class EvaluationMode {
        StrictControl,
        SafetyLenient,
    };

    struct Decision {
        bool allow_pipeline{false};
        ControlStatus status{};
        JointTargets joint_targets{};
    };

    explicit RuntimeFreshnessGate(FreshnessPolicy& freshness_policy);

    void reset();

    FreshnessPolicy::Evaluation evaluate(EvaluationMode mode,
                                         TimePointUs now,
                                         const RobotState& est,
                                         const MotionIntent& intent);

    void recordStrictMetrics(const FreshnessPolicy::Evaluation& freshness,
                             std::atomic<uint64_t>& stale_intent_count,
                             std::atomic<uint64_t>& stale_estimator_count) const;

    Decision computeControlDecision(const FreshnessPolicy::Evaluation& freshness,
                                    bool bus_ok,
                                    uint64_t loop_counter,
                                    const JointTargets& last_joint_targets,
                                    bool has_last_joint_targets,
                                    uint64_t consecutive_rejects) const;

    static void maybeLogReject(const std::shared_ptr<logging::AsyncLogger>& logger,
                               TimePointUs now,
                               const FreshnessPolicy::Evaluation& freshness,
                               const RobotState& est,
                               const MotionIntent& intent,
                               uint64_t loop_counter,
                               uint64_t consecutive_rejects);

private:
    FreshnessPolicy& freshness_policy_;
};
