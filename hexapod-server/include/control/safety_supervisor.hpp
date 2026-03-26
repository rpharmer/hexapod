#pragma once

#include "control_config.hpp"
#include "types.hpp"

#include <array>
#include <cstdint>
#include <functional>

#include "stability_tracker.hpp"

class SafetySupervisor {
public:
    struct FreshnessInputs {
        bool estimator_valid{true};
        bool intent_valid{true};
    };

    explicit SafetySupervisor(control_config::SafetyConfig config = {});

    SafetyState evaluate(const RobotState& raw,
                         const RobotState& est,
                         const MotionIntent& intent);

    SafetyState evaluate(const RobotState& raw,
                         const RobotState& est,
                         const MotionIntent& intent,
                         FreshnessInputs freshness);

private:
    static constexpr DurationUs kRecoveryHoldTimeUs{500000};
    static constexpr std::size_t kFaultCodeCount =
        static_cast<std::size_t>(FaultCode::COMMAND_TIMEOUT) + 1;

    struct FaultDecision {
        FaultCode code{FaultCode::NONE};
        bool torque_cut{false};
    };

    // Fault checks are expressed as an ordered rule model.
    //
    // Update policy in one place by editing the ordered rules in
    // evaluateFaultRules(): each rule defines a trigger condition,
    // resulting fault code, and torque-cut policy.
    //
    // Ordering is low->high precedence (matching faultPriority), and
    // each triggered rule can override the current decision only when its
    // fault has a higher priority than the current selection.
    struct FaultRule {
        std::function<bool()> is_triggered;
        FaultCode fault{FaultCode::NONE};
        bool torque_cut{false};
    };

    static int faultPriority(FaultCode code);
    static bool shouldReplaceFault(FaultCode current, FaultCode candidate);
    static std::size_t faultIndex(FaultCode code);
    bool canAttemptClear(const MotionIntent& intent, const FreshnessInputs& freshness) const;
    FaultDecision evaluateFaultRules(const RobotState& raw,
                                     const RobotState& est,
                                     const MotionIntent& intent,
                                     const FreshnessInputs& freshness,
                                     const StabilityAssessment& stability,
                                     int contact_count) const;
    FaultDecision evaluateCurrentFault(const RobotState& raw,
                                       const RobotState& est,
                                       const MotionIntent& intent,
                                       const FreshnessInputs& freshness,
                                       StabilityAssessment& stability) const;
    bool hasLargeJointPositionDiscontinuity(const RobotState& est) const;

    void trip(FaultCode code, bool torque_cut, TimePointUs timestamp_us);
    void clearActiveFault();

    control_config::SafetyConfig config_{};
    SafetyState state_{};
    TimePointUs recovery_started_at_us_{};
    std::array<uint32_t, kFaultCodeCount> trip_counts_{};
    std::array<TimePointUs, kFaultCodeCount> last_trip_timestamps_{};
    mutable bool has_previous_estimate_{false};
    mutable RobotState previous_estimate_{};
};
