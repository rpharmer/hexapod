#include "pico/stdlib.h"
#include "hexapod-common.hpp"
#include "hexapod-client.hpp"
#include "command_router.hpp"
#include "firmware_context.hpp"

#include <array>
#include <cstddef>
#include <type_traits>

namespace {

template <typename>
constexpr bool kAlwaysFalse = false;

constexpr int64_t HOST_LIVENESS_TIMEOUT_US = 2000000;

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
      CommandCode command = CommandCode::HELLO;
      const bool has_known_command = try_parse_command_code(packet.cmd, command);

      if(ctx.state != HexapodState::ACTIVE)
      {
        if(has_known_command && command == CommandCode::HELLO)
        {
          if(handleHandshake(ctx, packet.seq, packet.payload))
            ctx.state = HexapodState::ACTIVE;
        }
        else
        {
          continue;
        }
      }
      else
      {
        if(!has_known_command)
        {
          ctx.serial.send_packet(packet.seq, as_u8(CommandCode::NACK), {UNSUPPORTED_COMMAND});
          continue;
        }

        switch(command)
        {
          case CommandCode::HELLO:
            ctx.serial.send_packet(packet.seq, as_u8(CommandCode::NACK), {ALREADY_PAIRED});
            break;
          case CommandCode::HEARTBEAT:
            handleHeartbeatCommand(ctx, packet.seq);
            break;
          case CommandCode::KILL:
            return;
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
      }

      continue;
    }

    if(ctx.state == HexapodState::ACTIVE &&
       absolute_time_diff_us(lastHostActivity, get_absolute_time()) > HOST_LIVENESS_TIMEOUT_US)
    {
      transitionToHostDisconnectedSafeState(ctx);
      lastHostActivity = get_absolute_time();
    }
  }
}
