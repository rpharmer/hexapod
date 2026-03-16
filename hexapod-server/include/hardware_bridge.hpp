#pragma once

#include <cstdint>
#include <array>
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
                                  int timeout_ms = 100,
                                  std::vector<float> calibrations = {});

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
    uint16_t next_sequence();
    bool wait_for_ack(uint16_t seq) const;
    bool parse_ack_or_nack(const DecodedPacket& response,
                           uint16_t expected_seq,
                           std::vector<uint8_t>* ack_payload = nullptr) const;
    bool send_command_and_expect_ack(uint8_t cmd,
                                     const std::vector<uint8_t>& payload = {});
    bool send_command_and_expect_ack_payload(uint8_t cmd,
                                             const std::vector<uint8_t>& payload,
                                             std::vector<uint8_t>& ack_payload);

    std::vector<uint8_t> encode_joint_targets(const JointTargets& in) const;
    bool decode_full_hardware_state(const std::vector<uint8_t>& payload,
                                    RawHardwareState& out) const;

    bool do_handshake(const uint8_t requested_caps);
    bool send_heartbeat();
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
};
