#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "framing.hpp"
#include "types.hpp"

class BridgeCommandApi;
class BridgeLinkManager;
class JointFeedbackEstimator;
enum class CommandCode : uint8_t;

namespace logging {
class AsyncLogger;
}

class IHardwareBridge {
public:
    virtual ~IHardwareBridge() = default;
    virtual bool init() = 0;
    virtual bool read(RobotState& out) = 0;
    virtual bool write(const JointTargets& in) = 0;
    virtual std::optional<struct BridgeCommandResultMetadata> last_bridge_result() const;
};

// Typed command/bridge error taxonomy for migration away from bool-only APIs.
// Existing methods still return bool and now expose details through last_error().
enum class BridgeError : uint8_t {
    None = 0,
    NotReady,
    TransportFailure,
    ProtocolFailure,
    Timeout,
    Unsupported,
};

enum class BridgeFailurePhase : uint8_t {
    None = 0,
    Readiness,
    CapabilityNegotiation,
    CommandTransport,
    CommandResponse,
    CommandDecode,
    CommandExecution,
    Initialization,
};

enum class BridgeFailureDomain : uint8_t {
    None = 0,
    CapabilityProtocol,
    TransportLink,
    CommandProtocol,
};

struct BridgeCommandResultMetadata {
    BridgeError error{BridgeError::None};
    BridgeFailurePhase phase{BridgeFailurePhase::None};
    BridgeFailureDomain domain{BridgeFailureDomain::None};
    bool retryable{false};
    uint8_t command_code{0};
    uint8_t requested_capabilities{0};
    uint8_t negotiated_capabilities{0};
};

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
                                  std::vector<float> calibrations = {},
                                  std::shared_ptr<logging::AsyncLogger> logger = nullptr);
    explicit SimpleHardwareBridge(std::unique_ptr<IPacketEndpoint> endpoint,
                                  int timeout_ms = 100,
                                  std::vector<float> calibrations = {},
                                  std::shared_ptr<logging::AsyncLogger> logger = nullptr);
    ~SimpleHardwareBridge() override;

    bool init() override;
    bool read(RobotState& out) override;
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
    bool get_led_info(bool& present, uint8_t& count);
    bool set_led_colors(const std::array<uint8_t, kProtocolLedColorsPayloadBytes>& colors);
    BridgeError last_error() const;
    std::optional<BridgeCommandResultMetadata> last_bridge_result() const override;

private:
    bool run_ack_command(const char* command_name,
                         CommandCode command_code,
                         const std::vector<uint8_t>& payload,
                         bool require_estimator = false);
    template <typename DecodedPayload, typename Decoder>
    bool run_decoded_command(const char* command_name,
                             CommandCode command_code,
                             const std::vector<uint8_t>& request_payload,
                             Decoder&& decoder,
                             DecodedPayload& decoded,
                             bool require_estimator = false);

    bool complete_command(const char* command_name, BridgeError error, const char* reason = nullptr);
    BridgeError requireReady(const char* command_name, bool require_estimator = false);
    BridgeError withCommandApi(const char* command_name,
                               CommandCode command_code,
                               const std::function<BridgeError(BridgeCommandApi&)>& action,
                               bool require_estimator = false);
    BridgeError send_calibrations_result(const std::vector<float>& calibs);
    static const char* bridge_error_to_text(BridgeError error);
    static const char* bridge_phase_to_text(BridgeFailurePhase phase);
    static const char* bridge_domain_to_text(BridgeFailureDomain domain);
    void set_last_result(const BridgeCommandResultMetadata& metadata);

    std::string device_;
    int baud_rate_;
    int timeout_ms_;
    std::vector<float> calibrations_;
    bool initialized_{false};

    std::unique_ptr<IPacketEndpoint> packet_endpoint_{};
    std::shared_ptr<logging::AsyncLogger> logger_{};

    std::unique_ptr<BridgeLinkManager> link_manager_;
    std::unique_ptr<BridgeCommandApi> command_api_;
    std::unique_ptr<JointFeedbackEstimator> feedback_estimator_;
    BridgeError last_error_{BridgeError::None};
    BridgeCommandResultMetadata last_result_{};
};

inline std::optional<BridgeCommandResultMetadata> IHardwareBridge::last_bridge_result() const {
    return std::nullopt;
}
