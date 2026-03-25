#ifndef HEXAPOD_CLIENT_HOST_TEST
#include "pico/stdlib.h"
#endif
#include "hexapod-common.hpp"
#include "hexapod-client.hpp"
#include "command_router.hpp"
#include "command_dispatch_internal.hpp"
#include "firmware_context.hpp"

#include <array>
#include <cstddef>
#include <type_traits>

namespace {

template <typename>
constexpr bool kAlwaysFalse = false;

constexpr int64_t HOST_LIVENESS_TIMEOUT_US = 2000000;

int64_t readCurrentTimeUs()
{
#ifndef HEXAPOD_CLIENT_HOST_TEST
  return to_us_since_boot(get_absolute_time());
#else
  return 0;
#endif
}

void transitionToHostDisconnectedSafeState(FirmwareContext& ctx)
{
  ctx.servos.disable_all();
  gpio_put_masked(A0_GPIO_MASK, GPIO_LOW_MASK);
  ctx.state = HexapodState::WAITING_FOR_HOST;
}

void respondInvalidPayloadLength(FirmwareContext& ctx, uint16_t seq)
{
  ctx.serial.send_packet(seq, as_u8(CommandCode::NACK), {INVALID_PAYLOAD_LENGTH});
}

constexpr PayloadPolicy exactPayloadPolicyFromMetadata(CommandCode cmd)
{
  const CommandMetadata* metadata = find_command_metadata(cmd);
  return (metadata != nullptr)
      ? PayloadPolicy{metadata->exact_payload_size ? PayloadPolicyType::ExactBytes : PayloadPolicyType::Any,
                      metadata->payload_bytes}
      : PayloadPolicy{PayloadPolicyType::Any, 0};
}

template <auto Handler>
void routeCommand(FirmwareContext& ctx, uint16_t seq, const std::vector<uint8_t>& payload)
{
  if constexpr(std::is_invocable_v<decltype(Handler), FirmwareContext&, uint16_t, const std::vector<uint8_t>&>)
  {
    Handler(ctx, seq, payload);
  }
  else if constexpr(std::is_invocable_v<decltype(Handler), FirmwareContext&, uint16_t>)
  {
    (void)payload;
    Handler(ctx, seq);
  }
  else
  {
    static_assert(kAlwaysFalse<decltype(Handler)>,
                  "Unsupported command handler signature. Expected (ctx, seq) or (ctx, seq, payload).");
  }
}

constexpr std::array<CommandRoute, 14> COMMAND_ROUTES{{
    {CommandCode::SET_POWER_RELAY, exactPayloadPolicyFromMetadata(CommandCode::SET_POWER_RELAY), routeCommand<handleSetPowerRelayCommand>},
    {CommandCode::SET_SERVOS_ENABLED, exactPayloadPolicyFromMetadata(CommandCode::SET_SERVOS_ENABLED), routeCommand<handleSetServosEnabledCommand>},
    {CommandCode::GET_SERVOS_ENABLED, exactPayloadPolicyFromMetadata(CommandCode::GET_SERVOS_ENABLED), routeCommand<handleGetServosEnabledCommand>},
    {CommandCode::SET_SERVOS_TO_MID, exactPayloadPolicyFromMetadata(CommandCode::SET_SERVOS_TO_MID), routeCommand<handleSetServosToMidCommand>},
    {CommandCode::GET_LED_INFO, exactPayloadPolicyFromMetadata(CommandCode::GET_LED_INFO), routeCommand<handleGetLedInfoCommand>},
    {CommandCode::SET_LED_COLORS, exactPayloadPolicyFromMetadata(CommandCode::SET_LED_COLORS), routeCommand<handleSetLedColorsCommand>},
    {CommandCode::GET_ANGLE_CALIBRATIONS, exactPayloadPolicyFromMetadata(CommandCode::GET_ANGLE_CALIBRATIONS), routeCommand<handleGetAngleCalibCommand>},
    {CommandCode::GET_CURRENT, exactPayloadPolicyFromMetadata(CommandCode::GET_CURRENT), routeCommand<handleGetCurrentCommand>},
    {CommandCode::GET_VOLTAGE, exactPayloadPolicyFromMetadata(CommandCode::GET_VOLTAGE), routeCommand<handleGetVoltageCommand>},
    {CommandCode::GET_SENSOR, exactPayloadPolicyFromMetadata(CommandCode::GET_SENSOR), routeCommand<handleGetSensorCommand>},
    {CommandCode::GET_FULL_HARDWARE_STATE, exactPayloadPolicyFromMetadata(CommandCode::GET_FULL_HARDWARE_STATE), routeCommand<handleGetFullHardwareStateCommand>},
    {CommandCode::SET_ANGLE_CALIBRATIONS, exactPayloadPolicyFromMetadata(CommandCode::SET_ANGLE_CALIBRATIONS), routeCommand<handleCalibCommand>},
    {CommandCode::SET_TARGET_ANGLE, exactPayloadPolicyFromMetadata(CommandCode::SET_TARGET_ANGLE), routeCommand<handleSetAngleCommand>},
    {CommandCode::SET_JOINT_TARGETS, exactPayloadPolicyFromMetadata(CommandCode::SET_JOINT_TARGETS), routeCommand<handleSetJointTargetsCommand>},
}};

bool handleKnownActiveCommand(FirmwareContext& ctx, const DecodedPacket& packet, CommandCode command)
{
  switch(command)
  {
    case CommandCode::HELLO:
      ctx.serial.send_packet(packet.seq, as_u8(CommandCode::NACK), {ALREADY_PAIRED});
      break;
    case CommandCode::HEARTBEAT:
      handleHeartbeatCommand(ctx, packet.seq);
      break;
    case CommandCode::KILL:
      return false;
    default:
      if(!dispatchCommand(ctx,
                          command,
                          packet.seq,
                          packet.payload,
                          COMMAND_ROUTES.data(),
                          COMMAND_ROUTES.size(),
                          respondInvalidPayloadLength))
      {
        ctx.serial.send_packet(packet.seq, as_u8(CommandCode::NACK), {UNSUPPORTED_COMMAND});
      }
      break;
  }

  return true;
}

constexpr bool routesMatchSharedCommandMetadata()
{
  std::size_t route_count_from_metadata = 0;
  for(const CommandMetadata& metadata : kCommandMetadata)
  {
    if(!metadata.handled_by_firmware_dispatch)
      continue;

    ++route_count_from_metadata;
    bool found_route = false;
    for(const CommandRoute& route : COMMAND_ROUTES)
    {
      if(as_u8(route.cmd) == metadata.id)
      {
        found_route = true;
        break;
      }
    }
    if(!found_route)
      return false;
  }

  if(route_count_from_metadata != COMMAND_ROUTES.size())
    return false;

  for(const CommandRoute& route : COMMAND_ROUTES)
  {
    const CommandMetadata* metadata = find_command_metadata(route.cmd);
    if(metadata == nullptr || !metadata->handled_by_firmware_dispatch)
      return false;

    if(route.payloadPolicy.type == PayloadPolicyType::ExactBytes)
    {
      if(!metadata->exact_payload_size || metadata->payload_bytes != route.payloadPolicy.expectedBytes)
        return false;
    }
  }
  return true;
}

static_assert(routesMatchSharedCommandMetadata(),
              "Firmware route payload policies must stay aligned with shared command metadata.");

} // namespace

bool handleWaitingForHostPacket(FirmwareContext& ctx, const DecodedPacket& packet)
{
  CommandCode command = CommandCode::HELLO;
  const bool has_known_command = try_parse_command_code(packet.cmd, command);
  if(!has_known_command || command != CommandCode::HELLO)
    return true;

  if(handleHandshake(ctx, packet.seq, packet.payload))
    ctx.state = HexapodState::ACTIVE;
  return true;
}

bool handleActivePacket(FirmwareContext& ctx, const DecodedPacket& packet)
{
  CommandCode command = CommandCode::HELLO;
  const bool has_known_command = try_parse_command_code(packet.cmd, command);
  if(!has_known_command)
  {
    ctx.serial.send_packet(packet.seq, as_u8(CommandCode::NACK), {UNSUPPORTED_COMMAND});
    return true;
  }

  return handleKnownActiveCommand(ctx, packet, command);
}

void enforceHostLivenessTimeout(FirmwareContext& ctx, int64_t now_us, int64_t& last_host_activity_us)
{
  if(ctx.state != HexapodState::ACTIVE)
    return;

  if((now_us - last_host_activity_us) <= HOST_LIVENESS_TIMEOUT_US)
    return;

  transitionToHostDisconnectedSafeState(ctx);
  last_host_activity_us = now_us;
}

void runCommandLoop()
{
  firmware().state = HexapodState::WAITING_FOR_HOST;
  int64_t last_host_activity_us = readCurrentTimeUs();

  while(1)
  {
    FirmwareContext& ctx = firmware();
    DecodedPacket packet;
    if(ctx.serial.recv_packet(packet))
    {
      last_host_activity_us = readCurrentTimeUs();

      if(ctx.state != HexapodState::ACTIVE)
      {
        handleWaitingForHostPacket(ctx, packet);
      }
      else
      {
        if(!handleActivePacket(ctx, packet))
          return;
      }
    }

    enforceHostLivenessTimeout(ctx, readCurrentTimeUs(), last_host_activity_us);
  }
}
