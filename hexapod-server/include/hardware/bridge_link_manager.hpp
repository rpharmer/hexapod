#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "types.hpp"

class IPacketEndpoint;
class SerialCommsServer;
class TransportSession;
class HandshakeClient;
class HardwareStateCodec;
class CommandClient;

class BridgeLinkManager {
public:
    enum class EnsureLinkFailure : uint8_t {
        None = 0,
        CapabilityNegotiation,
        TransportLink,
    };

    struct EnsureLinkResult {
        bool ok{false};
        EnsureLinkFailure failure{EnsureLinkFailure::TransportLink};
        bool link_timed_out{false};
        bool heartbeat_due{false};
        uint8_t negotiated_capabilities{0};
    };

    ~BridgeLinkManager();
    BridgeLinkManager(std::string device,
                      int baud_rate,
                      int timeout_ms,
                      std::unique_ptr<IPacketEndpoint> endpoint = nullptr,
                      DurationUs link_timeout = DurationUs{500000},
                      DurationUs heartbeat_idle_interval = DurationUs{150000});

    bool init(uint8_t requested_capabilities);
    bool ensure_link(uint8_t requested_capabilities);
    EnsureLinkResult ensure_link_with_status(uint8_t requested_capabilities);

    bool has_capability(uint8_t capability) const;
    bool is_initialized() const;
    uint8_t negotiated_capabilities() const;

    HardwareStateCodec* codec() const;
    CommandClient* command_client() const;

private:
    std::string device_;
    int baud_rate_{0};
    int timeout_ms_{0};
    DurationUs link_timeout_{};
    DurationUs heartbeat_idle_interval_{};
    bool initialized_{false};

    std::unique_ptr<SerialCommsServer> serial_comms_{};
    std::unique_ptr<IPacketEndpoint> packet_endpoint_{};
    std::unique_ptr<TransportSession> transport_{};
    std::unique_ptr<HandshakeClient> handshake_{};
    std::unique_ptr<HardwareStateCodec> codec_{};
    std::unique_ptr<CommandClient> command_client_{};
};
