#pragma once

#include "types.hpp"

class IHardwareBridge {
public:
    virtual ~IHardwareBridge() = default;
    virtual bool init() = 0;
    virtual bool read(RawHardwareState& out) = 0;
    virtual bool write(const JointTargets& in) = 0;
};

class SimpleHardwareBridge final : public IHardwareBridge {
public:
    bool init() override;
    bool read(RawHardwareState& out) override;
    bool write(const JointTargets& in) override;

private:
    RawHardwareState state_{};
    JointTargets last_written_{};
    
    SerialCommsServer serialComs_{};
};