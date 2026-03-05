#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "types.hpp"
#include "serialCommsServer.hpp"

class IHardwareBridge {
public:
    virtual ~IHardwareBridge() = default;
    virtual bool init() = 0;
    virtual bool read(RawHardwareState& out) = 0;
    virtual bool write(const JointTargets& in) = 0;
};

class SimpleHardwareBridge final : public IHardwareBridge {
public:
    explicit SimpleHardwareBridge(std::string device = "/dev/ttyACM0",
                                  int baud_rate = 115200,
                                  int timeout_ms = 100);

    bool init() override;
    bool read(RawHardwareState& out) override;
    bool write(const JointTargets& in) override;

private:
    uint16_t next_sequence();
    bool wait_for_ack(uint16_t seq) const;

    std::vector<uint8_t> encode_joint_targets(const JointTargets& in) const;
    bool decode_full_hardware_state(const std::vector<uint8_t>& payload,
                                    RawHardwareState& out) const;

    bool do_handshake(const uint8_t requested_caps);
    bool send_heartbeat();
    bool send_calibrations(const std::vector<float>& calibs);
  
    std::string device_;
    int baud_rate_;
    int timeout_ms_;
    uint16_t seq_{0};
    bool initialized_{false};

    RawHardwareState state_{};
    JointTargets last_written_{};

    std::unique_ptr<SerialCommsServer> serialComs_{};
};
