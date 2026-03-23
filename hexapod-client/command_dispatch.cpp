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
  ctx.serial.send_packet(seq, NACK, {INVALID_PAYLOAD_LENGTH});
}

constexpr PayloadPolicy exactPayloadPolicyFromMetadata(uint8_t cmd)
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
    {SET_POWER_RELAY, exactPayloadPolicyFromMetadata(SET_POWER_RELAY), routeCommand<handleSetPowerRelayCommand>},
    {SET_SERVOS_ENABLED, exactPayloadPolicyFromMetadata(SET_SERVOS_ENABLED), routeCommand<handleSetServosEnabledCommand>},
    {GET_SERVOS_ENABLED, exactPayloadPolicyFromMetadata(GET_SERVOS_ENABLED), routeCommand<handleGetServosEnabledCommand>},
    {SET_SERVOS_TO_MID, exactPayloadPolicyFromMetadata(SET_SERVOS_TO_MID), routeCommand<handleSetServosToMidCommand>},
    {GET_LED_INFO, exactPayloadPolicyFromMetadata(GET_LED_INFO), routeCommand<handleGetLedInfoCommand>},
    {SET_LED_COLORS, exactPayloadPolicyFromMetadata(SET_LED_COLORS), routeCommand<handleSetLedColorsCommand>},
    {GET_ANGLE_CALIBRATIONS, exactPayloadPolicyFromMetadata(GET_ANGLE_CALIBRATIONS), routeCommand<handleGetAngleCalibCommand>},
    {GET_CURRENT, exactPayloadPolicyFromMetadata(GET_CURRENT), routeCommand<handleGetCurrentCommand>},
    {GET_VOLTAGE, exactPayloadPolicyFromMetadata(GET_VOLTAGE), routeCommand<handleGetVoltageCommand>},
    {GET_SENSOR, exactPayloadPolicyFromMetadata(GET_SENSOR), routeCommand<handleGetSensorCommand>},
    {GET_FULL_HARDWARE_STATE, exactPayloadPolicyFromMetadata(GET_FULL_HARDWARE_STATE), routeCommand<handleGetFullHardwareStateCommand>},
    {SET_ANGLE_CALIBRATIONS, exactPayloadPolicyFromMetadata(SET_ANGLE_CALIBRATIONS), routeCommand<handleCalibCommand>},
    {SET_TARGET_ANGLE, exactPayloadPolicyFromMetadata(SET_TARGET_ANGLE), routeCommand<handleSetAngleCommand>},
    {SET_JOINT_TARGETS, exactPayloadPolicyFromMetadata(SET_JOINT_TARGETS), routeCommand<handleSetJointTargetsCommand>},
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
      if(route.cmd == metadata.id)
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

      if(ctx.state != HexapodState::ACTIVE)
      {
        if(packet.cmd == HELLO)
        {
          if(handleHandshake(ctx, packet.seq, packet.payload))
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
      else if(dispatchCommand(ctx, packet, COMMAND_ROUTES.data(), COMMAND_ROUTES.size(), respondInvalidPayloadLength))
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
      transitionToHostDisconnectedSafeState(ctx);
      lastHostActivity = get_absolute_time();
    }
  }
}
