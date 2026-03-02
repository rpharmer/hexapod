#include "hardware_bridge.hpp"

#include <array>

#include <CppLinuxSerial/SerialPort.hpp>
#include "hexapod-common.hpp"

using namespace mn::CppLinuxSerial;

namespace {
constexpr std::size_t kWireJointCount = 18;
constexpr std::size_t kWireFootContactCount = 6;

} // namespace

SimpleHardwareBridge::SimpleHardwareBridge(std::string device, int baud_rate, int timeout_ms)
    : device_(std::move(device)), baud_rate_(baud_rate), timeout_ms_(timeout_ms)
{}

bool SimpleHardwareBridge::init()
{
    serialComs_ = std::make_unique<SerialCommsServer>(device_, SerialCommsServer::int_to_baud_rate(baud_rate_), NumDataBits::EIGHT,
                                                      Parity::NONE, NumStopBits::ONE);
    serialComs_->SetTimeout(timeout_ms_);
    serialComs_->Open();

    initialized_ = true;
    return true;
}

bool SimpleHardwareBridge::read(RawHardwareState& out)
{
    if (!initialized_ || !serialComs_)
    {
        return false;
    }

    const uint16_t seq = next_sequence();
    serialComs_->send_packet(seq, GET_FULL_HARDWARE_STATE, {});

    DecodedPacket response;
    if (!serialComs_->recv_packet(response) || response.seq != seq || response.cmd != ACK)
    {
        return false;
    }

    if (!decode_full_hardware_state(response.payload, out))
    {
        return false;
    }

    state_ = out;
    return true;
}

bool SimpleHardwareBridge::write(const JointTargets& in)
{
    if (!initialized_ || !serialComs_)
    {
        return false;
    }

    const uint16_t seq = next_sequence();
    serialComs_->send_packet(seq, SET_JOINT_TARGETS, encode_joint_targets(in));

    if (!wait_for_ack(seq))
    {
        return false;
    }

    last_written_ = in;
    return true;
}

uint16_t SimpleHardwareBridge::next_sequence()
{
    return seq_++;
}

bool SimpleHardwareBridge::wait_for_ack(uint16_t seq) const
{
    DecodedPacket response;
    return serialComs_->recv_packet(response) && response.seq == seq && response.cmd == ACK;
}

std::vector<uint8_t> SimpleHardwareBridge::encode_joint_targets(const JointTargets& in) const
{
    std::vector<uint8_t> payload;
    payload.reserve(kWireJointCount * sizeof(float));

    for (const auto& leg : in.leg_raw_states)
    {
        for (int j = 0; j < kJointsPerLeg; ++j)
        {
            const float pos_rad = static_cast<float>(leg.joint_raw_state[j].pos_rad);
            append_scalar(payload, pos_rad);
        }
    }

    return payload;
}

bool SimpleHardwareBridge::decode_full_hardware_state(const std::vector<uint8_t>& payload,
                                                      RawHardwareState& out) const
{
    std::size_t offset = 0;

    for (auto& leg : out.leg_states)
    {
        for (int j = 0; j < kJointsPerLeg; ++j)
        {
            float pos_rad = 0.0f;
            if (!read_scalar(payload, offset, pos_rad))
            {
                return false;
            }
            leg.joint_raw_state[j].pos_rad = pos_rad;
        }
    }

    for (std::size_t i = 0; i < kWireFootContactCount; ++i)
    {
        uint8_t contact = 0;
        if (!read_scalar(payload, offset, contact))
        {
            return false;
        }
        out.foot_contacts[i] = (contact != 0);
    }

    if (!read_scalar(payload, offset, out.voltage) ||
        !read_scalar(payload, offset, out.current))
    {
        return false;
    }

    if (offset != payload.size())
    {
        return false;
    }

    out.timestamp_us = now_us();
    return true;
}
