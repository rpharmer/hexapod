#include "pico/stdlib.h"
#include "hexapod-common.hpp"
#include "hexapod-client.hpp"
#include "firmware_context.hpp"

#include <array>
#include <cstddef>

namespace {

constexpr int64_t HOST_LIVENESS_TIMEOUT_US = 2000000;

void transitionToHostDisconnectedSafeState()
{
  firmware().servos.disable_all();
  gpio_put_masked(A0_GPIO_MASK, GPIO_LOW_MASK);
  firmware().state = HexapodState::WAITING_FOR_HOST;
}

enum class PayloadPolicyType : uint8_t
{
  Any,
  ExactBytes
};

struct PayloadPolicy
{
  PayloadPolicyType type;
  std::size_t expectedBytes;

  bool accepts(std::size_t payloadBytes) const
  {
    if(type == PayloadPolicyType::Any)
      return true;

    return payloadBytes == expectedBytes;
  }
};

using RoutedHandler = void (*)(uint16_t seq, const std::vector<uint8_t>& payload);

struct CommandRoute
{
  uint8_t cmd;
  PayloadPolicy payloadPolicy;
  RoutedHandler handler;
};

void handleGetAngleCalibRouted(uint16_t seq, const std::vector<uint8_t>&)
{
  handleGetAngleCalibCommand(seq);
}

void handleGetCurrentRouted(uint16_t seq, const std::vector<uint8_t>&)
{
  handleGetCurrentCommand(seq);
}

void handleGetVoltageRouted(uint16_t seq, const std::vector<uint8_t>&)
{
  handleGetVoltageCommand(seq);
}

void handleGetFullHardwareStateRouted(uint16_t seq, const std::vector<uint8_t>&)
{
  handleGetFullHardwareStateCommand(seq);
}

void handleGetServosEnabledRouted(uint16_t seq, const std::vector<uint8_t>&)
{
  handleGetServosEnabledCommand(seq);
}

void handleSetServosToMidRouted(uint16_t seq, const std::vector<uint8_t>&)
{
  handleSetServosToMidCommand(seq);
}

constexpr std::array<CommandRoute, 12> COMMAND_ROUTES{{
    {SET_POWER_RELAY, {PayloadPolicyType::ExactBytes, 1}, handleSetPowerRelayCommand},
    {SET_SERVOS_ENABLED, {PayloadPolicyType::ExactBytes, kProtocolServoEnablePayloadBytes}, handleSetServosEnabledCommand},
    {GET_SERVOS_ENABLED, {PayloadPolicyType::ExactBytes, 0}, handleGetServosEnabledRouted},
    {SET_SERVOS_TO_MID, {PayloadPolicyType::ExactBytes, 0}, handleSetServosToMidRouted},
    {GET_ANGLE_CALIBRATIONS, {PayloadPolicyType::ExactBytes, 0}, handleGetAngleCalibRouted},
    {GET_CURRENT, {PayloadPolicyType::ExactBytes, 0}, handleGetCurrentRouted},
    {GET_VOLTAGE, {PayloadPolicyType::ExactBytes, 0}, handleGetVoltageRouted},
    {GET_SENSOR, {PayloadPolicyType::ExactBytes, 1}, handleGetSensorCommand},
    {GET_FULL_HARDWARE_STATE, {PayloadPolicyType::ExactBytes, 0}, handleGetFullHardwareStateRouted},
    {SET_ANGLE_CALIBRATIONS, {PayloadPolicyType::ExactBytes, kProtocolCalibrationsPayloadBytes}, handleCalibCommand},
    {SET_TARGET_ANGLE, {PayloadPolicyType::ExactBytes, sizeof(uint8_t) + sizeof(float)}, handleSetAngleCommand},
    {SET_JOINT_TARGETS, {PayloadPolicyType::ExactBytes, kProtocolJointTargetsPayloadBytes}, handleSetJointTargetsCommand},
}};

const CommandRoute* findRoute(uint8_t cmd)
{
  for(const CommandRoute& route : COMMAND_ROUTES)
  {
    if(route.cmd == cmd)
      return &route;
  }

  return nullptr;
}

bool dispatchCommand(const DecodedPacket& packet)
{
  const CommandRoute* route = findRoute(packet.cmd);
  if(route == nullptr)
    return false;

  if(!route->payloadPolicy.accepts(packet.payload.size()))
  {
    firmware().serial.send_packet(packet.seq, NACK, {INVALID_PAYLOAD_LENGTH});
    return true;
  }

  route->handler(packet.seq, packet.payload);
  return true;
}

} // namespace

void runCommandLoop()
{
  firmware().state = HexapodState::WAITING_FOR_HOST;
  absolute_time_t lastHostActivity = get_absolute_time();

  while(1)
  {
    DecodedPacket packet;
    if(firmware().serial.recv_packet(packet))
    {
      lastHostActivity = get_absolute_time();

      if(firmware().state != HexapodState::ACTIVE)
      {
        if(packet.cmd == HELLO)
        {
          if(handleHandshake(packet.seq, packet.payload))
            firmware().state = HexapodState::ACTIVE;
        }
        else
        {
          continue;
        }
      }
      else if(packet.cmd == HELLO)
      {
        firmware().serial.send_packet(packet.seq, NACK, {ALREADY_PAIRED});
      }
      else if(packet.cmd == HEARTBEAT)
      {
        handleHeartbeatCommand(packet.seq);
      }
      else if(dispatchCommand(packet))
      {
      }
      else if(packet.cmd == KILL)
      {
        break;
      }
      else
      {
        firmware().serial.send_packet(packet.seq, NACK, {UNSUPPORTED_COMMAND});
      }

      continue;
    }

    if(firmware().state == HexapodState::ACTIVE &&
       absolute_time_diff_us(lastHostActivity, get_absolute_time()) > HOST_LIVENESS_TIMEOUT_US)
    {
      transitionToHostDisconnectedSafeState();
      lastHostActivity = get_absolute_time();
    }
  }
}
