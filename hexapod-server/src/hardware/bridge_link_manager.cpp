#include "bridge_link_manager.hpp"

#include <memory>
#include <utility>

#include <CppLinuxSerial/SerialPort.hpp>

#include "command_client.hpp"
#include "handshake_client.hpp"
#include "hardware_bridge.hpp"
#include "hardware_state_codec.hpp"
#include "serialCommsServer.hpp"
#include "transport_session.hpp"

using namespace mn::CppLinuxSerial;

namespace {

class SerialPacketEndpoint final : public IPacketEndpoint {
public:
    explicit SerialPacketEndpoint(SerialCommsServer& serial)
        : serial_(serial) {}

    void send_packet(uint16_t seq, uint8_t cmd, const std::vector<uint8_t>& payload) override {
        serial_.send_packet(seq, cmd, payload);
    }

    bool recv_packet(DecodedPacket& packet) override {
        return serial_.recv_packet(packet);
    }

private:
    SerialCommsServer& serial_;
};

}  // namespace


BridgeLinkManager::~BridgeLinkManager() = default;
BridgeLinkManager::BridgeLinkManager(std::string device,
                                     int baud_rate,
                                     int timeout_ms,
                                     std::unique_ptr<IPacketEndpoint> endpoint,
                                     DurationUs link_timeout,
                                     DurationUs heartbeat_idle_interval)
    : device_(std::move(device)),
      baud_rate_(baud_rate),
      timeout_ms_(timeout_ms),
      link_timeout_(link_timeout),
      heartbeat_idle_interval_(heartbeat_idle_interval),
      packet_endpoint_(std::move(endpoint)) {}

bool BridgeLinkManager::init(uint8_t requested_capabilities) {
    if (!packet_endpoint_) {
        serial_comms_ = std::make_unique<SerialCommsServer>(device_,
                                                             SerialCommsServer::int_to_baud_rate(baud_rate_),
                                                             NumDataBits::EIGHT,
                                                             Parity::NONE,
                                                             NumStopBits::ONE);
        serial_comms_->SetTimeout(timeout_ms_);
        serial_comms_->Open();
        packet_endpoint_ = std::make_unique<SerialPacketEndpoint>(*serial_comms_);
    }

    if (!packet_endpoint_) {
        return false;
    }

    transport_ = std::make_unique<TransportSession>(*packet_endpoint_, link_timeout_, heartbeat_idle_interval_);
    codec_ = std::make_unique<HardwareStateCodec>();
    command_client_ = std::make_unique<CommandClient>(*transport_);
    handshake_ = std::make_unique<HandshakeClient>(*transport_, *command_client_);

    if (!handshake_->establish_link(requested_capabilities)) {
        return false;
    }

    if (!handshake_->send_heartbeat()) {
        return false;
    }

    initialized_ = true;
    return true;
}

bool BridgeLinkManager::ensure_link(uint8_t requested_capabilities) {
    if (!initialized_ || !transport_ || !handshake_) {
        return false;
    }

    const TimePointUs now = now_us();
    if (transport_->has_link_timed_out(now)) {
        return handshake_->establish_link(requested_capabilities);
    }

    if (transport_->heartbeat_due(now)) {
        return handshake_->send_heartbeat();
    }

    return true;
}

bool BridgeLinkManager::has_capability(uint8_t capability) const {
    return handshake_ && handshake_->has_capability(capability);
}

bool BridgeLinkManager::is_initialized() const {
    return initialized_;
}

HardwareStateCodec* BridgeLinkManager::codec() const {
    return codec_.get();
}

CommandClient* BridgeLinkManager::command_client() const {
    return command_client_.get();
}
