#pragma once

#include <cstdint>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "types.hpp"
#include "serialCommsServer.hpp"

class IHardwareBridge {
public:
    virtual ~IHardwareBridge() = default;
    virtual bool init() = 0;
    virtual bool read(RawHardwareState& out) = 0;
    virtual bool write(const JointTargets& in) = 0;
};

class TransportSession;
class HandshakeClient;
class HardwareStateCodec;
class CommandClient;

class SimpleHardwareBridge final : public IHardwareBridge {
public:
    explicit SimpleHardwareBridge(std::string device = "/dev/ttyACM0",
                                  int baud_rate = 115200,
                                  int timeout_ms = 100,
                                  std::vector<float> calibrations = {});
    ~SimpleHardwareBridge() override;

    bool init() override;
    bool read(RawHardwareState& out) override;
    bool write(const JointTargets& in) override;

    bool set_angle_calibrations(const std::vector<float>& calibs);
    bool set_target_angle(uint8_t servo_id, float angle);
    bool set_power_relay(bool enabled);
    bool get_angle_calibrations(std::vector<float>& out_calibs);
    bool get_current(float& out_current);
    bool get_voltage(float& out_voltage);
    bool get_sensor(uint8_t sensor_id, float& out_voltage);
    bool send_diagnostic(const std::vector<uint8_t>& payload,
                         std::vector<uint8_t>& response_payload);
    bool set_servos_enabled(const std::array<bool, kNumJoints>& enabled);
    bool get_servos_enabled(std::array<bool, kNumJoints>& enabled);
    bool set_servos_to_mid();

private:
    bool ensure_link();
    bool send_calibrations(const std::vector<float>& calibs);
  
    std::string device_;
    int baud_rate_;
    int timeout_ms_;
    std::vector<float> calibrations_;
    uint16_t seq_{0};
    bool initialized_{false};

    RawHardwareState state_{};
    JointTargets last_written_{};

    std::unique_ptr<SerialCommsServer> serialComs_{};
    std::unique_ptr<TransportSession> transport_;
    std::unique_ptr<HandshakeClient> handshake_;
    std::unique_ptr<HardwareStateCodec> codec_;
    std::unique_ptr<CommandClient> command_client_;
};
