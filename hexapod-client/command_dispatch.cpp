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
  FirmwareContext& ctx = firmware();
  ctx.servos.disable_all();
  gpio_put_masked(A0_GPIO_MASK, GPIO_LOW_MASK);
  ctx.state = HexapodState::WAITING_FOR_HOST;
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

using RoutedHandler = void (*)(FirmwareContext& ctx, uint16_t seq, const std::vector<uint8_t>& payload);

struct CommandRoute
{
  uint8_t cmd;
  PayloadPolicy payloadPolicy;
  RoutedHandler handler;
};

void handleGetAngleCalibRouted(FirmwareContext& ctx, uint16_t seq, const std::vector<uint8_t>&)
{
  handleGetAngleCalibCommand(ctx, seq);
}

void handleGetCurrentRouted(FirmwareContext& ctx, uint16_t seq, const std::vector<uint8_t>&)
{
  handleGetCurrentCommand(ctx, seq);
}

void handleGetVoltageRouted(FirmwareContext& ctx, uint16_t seq, const std::vector<uint8_t>&)
{
  handleGetVoltageCommand(ctx, seq);
}

void handleGetFullHardwareStateRouted(FirmwareContext& ctx, uint16_t seq, const std::vector<uint8_t>&)
{
  handleGetFullHardwareStateCommand(ctx, seq);
}

void handleGetServosEnabledRouted(FirmwareContext& ctx, uint16_t seq, const std::vector<uint8_t>&)
{
  handleGetServosEnabledCommand(ctx, seq);
}

void handleSetServosToMidRouted(FirmwareContext& ctx, uint16_t seq, const std::vector<uint8_t>&)
{
  handleSetServosToMidCommand(ctx, seq);
}

void handleSetPowerRelayRouted(FirmwareContext& ctx, uint16_t seq, const std::vector<uint8_t>& payload)
{
  handleSetPowerRelayCommand(ctx, seq, payload);
}

void handleSetServosEnabledRouted(FirmwareContext& ctx, uint16_t seq, const std::vector<uint8_t>& payload)
{
  handleSetServosEnabledCommand(ctx, seq, payload);
}

void handleGetSensorRouted(FirmwareContext& ctx, uint16_t seq, const std::vector<uint8_t>& payload)
{
  handleGetSensorCommand(ctx, seq, payload);
}

void handleCalibRouted(FirmwareContext& ctx, uint16_t seq, const std::vector<uint8_t>& payload)
{
  handleCalibCommand(ctx, seq, payload);
}

void handleSetAngleRouted(FirmwareContext& ctx, uint16_t seq, const std::vector<uint8_t>& payload)
{
  handleSetAngleCommand(ctx, seq, payload);
}

void handleSetJointTargetsRouted(FirmwareContext& ctx, uint16_t seq, const std::vector<uint8_t>& payload)
{
  handleSetJointTargetsCommand(ctx, seq, payload);
}

constexpr std::array<CommandRoute, 12> COMMAND_ROUTES{{
    {SET_POWER_RELAY, {PayloadPolicyType::ExactBytes, 1}, handleSetPowerRelayRouted},
    {SET_SERVOS_ENABLED, {PayloadPolicyType::ExactBytes, kProtocolServoEnablePayloadBytes}, handleSetServosEnabledRouted},
    {GET_SERVOS_ENABLED, {PayloadPolicyType::ExactBytes, 0}, handleGetServosEnabledRouted},
    {SET_SERVOS_TO_MID, {PayloadPolicyType::ExactBytes, 0}, handleSetServosToMidRouted},
    {GET_ANGLE_CALIBRATIONS, {PayloadPolicyType::ExactBytes, 0}, handleGetAngleCalibRouted},
    {GET_CURRENT, {PayloadPolicyType::ExactBytes, 0}, handleGetCurrentRouted},
    {GET_VOLTAGE, {PayloadPolicyType::ExactBytes, 0}, handleGetVoltageRouted},
    {GET_SENSOR, {PayloadPolicyType::ExactBytes, 1}, handleGetSensorRouted},
    {GET_FULL_HARDWARE_STATE, {PayloadPolicyType::ExactBytes, 0}, handleGetFullHardwareStateRouted},
    {SET_ANGLE_CALIBRATIONS, {PayloadPolicyType::ExactBytes, kProtocolCalibrationsPayloadBytes}, handleCalibRouted},
    {SET_TARGET_ANGLE, {PayloadPolicyType::ExactBytes, sizeof(uint8_t) + sizeof(float)}, handleSetAngleRouted},
    {SET_JOINT_TARGETS, {PayloadPolicyType::ExactBytes, kProtocolJointTargetsPayloadBytes}, handleSetJointTargetsRouted},
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

bool dispatchCommand(FirmwareContext& ctx, const DecodedPacket& packet)
{
  const CommandRoute* route = findRoute(packet.cmd);
  if(route == nullptr)
    return false;

  if(!route->payloadPolicy.accepts(packet.payload.size()))
  {
    ctx.serial.send_packet(packet.seq, NACK, {INVALID_PAYLOAD_LENGTH});
    return true;
  }

  route->handler(ctx, packet.seq, packet.payload);
  return true;
}

} // namespace

void runCommandLoop()
{
  firmware().state = HexapodState::WAITING_FOR_HOST;
  absolute_time_t lastHostActivity = get_absolute_time();

  while(1)
  {
    FirmwareContext& ctx = firmware();
    DecodedPacket packet;
    if(ctx.serial.recv_packet(packet))
    {
      lastHostActivity = get_absolute_time();

      if(ctx.state != HexapodState::ACTIVE)
      {
        if(packet.cmd == HELLO)
        {
          if(handleHandshake(packet.seq, packet.payload))
            ctx.state = HexapodState::ACTIVE;
        }
        else
        {
          continue;
        }
      }
      else if(packet.cmd == HELLO)
      {
        ctx.serial.send_packet(packet.seq, NACK, {ALREADY_PAIRED});
      }
      else if(packet.cmd == HEARTBEAT)
      {
        handleHeartbeatCommand(ctx, packet.seq);
      }
      else if(dispatchCommand(ctx, packet))
      {
      }
      else if(packet.cmd == KILL)
      {
        break;
      }
      else
      {
        ctx.serial.send_packet(packet.seq, NACK, {UNSUPPORTED_COMMAND});
      }

      continue;
    }

    if(ctx.state == HexapodState::ACTIVE &&
       absolute_time_diff_us(lastHostActivity, get_absolute_time()) > HOST_LIVENESS_TIMEOUT_US)
    {
      transitionToHostDisconnectedSafeState();
      lastHostActivity = get_absolute_time();
    }
  }
}
