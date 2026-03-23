#pragma once

#include <cstdint>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "types.hpp"
#include "serialCommsServer.hpp"
#include "logger.hpp"

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

class IPacketEndpoint {
public:
    virtual ~IPacketEndpoint() = default;
    virtual void send_packet(uint16_t seq, uint8_t cmd, const std::vector<uint8_t>& payload) = 0;
    virtual bool recv_packet(DecodedPacket& packet) = 0;
};

class SimpleHardwareBridge final : public IHardwareBridge {
public:
    explicit SimpleHardwareBridge(std::string device = "/dev/ttyACM0",
                                  int baud_rate = 115200,
                                  int timeout_ms = 100,
                                  std::vector<float> calibrations = {});
    explicit SimpleHardwareBridge(std::unique_ptr<IPacketEndpoint> endpoint,
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
    bool request_ack(uint8_t cmd,
                     const std::vector<uint8_t>& payload,
                     const char* command_name);

    template <typename T, typename Decoder>
    bool request_decoded(uint8_t cmd,
                         const std::vector<uint8_t>& payload,
                         Decoder&& decoder,
                         T& out,
                         const char* command_name) {
        std::vector<uint8_t> response_payload;
        if (!request_ack_payload(cmd, payload, response_payload, command_name)) {
            return false;
        }

        if (!decoder(response_payload, out)) {
            if (auto logger = logging::GetDefaultLogger()) {
                LOG_ERROR(logger,
                          "command ",
                          command_name,
                          " decode failed (payload_size=",
                          static_cast<unsigned>(response_payload.size()),
                          ")");
            }
            return false;
        }

        return true;
    }

    bool request_ack_payload(uint8_t cmd,
                             const std::vector<uint8_t>& payload,
                             std::vector<uint8_t>& out_payload,
                             const char* command_name);
    bool request_transaction(uint8_t cmd,
                             const std::vector<uint8_t>& payload,
                             std::vector<uint8_t>* out_payload,
                             const char* command_name);

    bool ensure_link();
    bool send_calibrations(const std::vector<float>& calibs);
    void synthesizeJointFeedback(RawHardwareState& out);
  
    std::string device_;
    int baud_rate_;
    int timeout_ms_;
    std::vector<float> calibrations_;
    uint16_t seq_{0};
    bool initialized_{false};

    RawHardwareState state_{};
    JointTargets last_written_{};
    bool software_feedback_enabled_{false};
    TimePointUs last_software_feedback_timestamp_{};

    std::unique_ptr<SerialCommsServer> serialComs_{};
    std::unique_ptr<IPacketEndpoint> packet_endpoint_{};
    std::unique_ptr<TransportSession> transport_;
    std::unique_ptr<HandshakeClient> handshake_;
    std::unique_ptr<HardwareStateCodec> codec_;
    std::unique_ptr<CommandClient> command_client_;
};
