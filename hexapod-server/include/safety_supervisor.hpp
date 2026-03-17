#pragma once

#include "types.hpp"

class SafetySupervisor {
public:
    SafetyState evaluate(const RawHardwareState& raw,
                         const EstimatedState& est,
                         const MotionIntent& intent);

private:
    static int faultPriority(FaultCode code);
    static bool shouldReplaceFault(FaultCode current, FaultCode candidate);

    void trip(SafetyState& s, FaultCode code, bool torque_cut);
};
