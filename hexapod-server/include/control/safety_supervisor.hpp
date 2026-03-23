#pragma once

#include "control_config.hpp"
#include "types.hpp"

#include <array>
#include <cstdint>

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

    static int faultPriority(FaultCode code);
    static bool shouldReplaceFault(FaultCode current, FaultCode candidate);
    static std::size_t faultIndex(FaultCode code);
    bool canAttemptClear(const MotionIntent& intent, const FreshnessInputs& freshness) const;
    FaultDecision evaluateCurrentFault(const RobotState& raw,
                                       const RobotState& est,
                                       const MotionIntent& intent,
                                       const FreshnessInputs& freshness) const;

    void trip(FaultCode code, bool torque_cut, TimePointUs timestamp_us);
    void clearActiveFault();

    control_config::SafetyConfig config_{};
    SafetyState state_{};
    TimePointUs recovery_started_at_us_{};
    std::array<uint32_t, kFaultCodeCount> trip_counts_{};
    std::array<TimePointUs, kFaultCodeCount> last_trip_timestamps_{};
};
