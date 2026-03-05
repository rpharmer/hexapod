#pragma once

#include "types.hpp"

class SafetySupervisor {
public:
    SafetyState evaluate(const RawHardwareState& raw,
                         const EstimatedState& est,
                         const MotionIntent& intent);

private:
    void trip(SafetyState& s, FaultCode code, bool torque_cut);
};