#pragma once

#include "types.hpp"

class IEstimator {
public:
    virtual ~IEstimator() = default;
    virtual EstimatedState update(const RawHardwareState& raw) = 0;
};

class SimpleEstimator final : public IEstimator {
public:
    EstimatedState update(const RawHardwareState& raw) override;
};