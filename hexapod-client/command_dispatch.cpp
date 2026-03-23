#include "pico/stdlib.h"
#include "hexapod-common.hpp"
#include "hexapod-client.hpp"
#include "command_router.hpp"
#include "firmware_context.hpp"

#include <array>
#include <cstddef>

namespace {

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

template <void (*Handler)(FirmwareContext&, uint16_t)>
void routeNoPayload(FirmwareContext& ctx, uint16_t seq, const std::vector<uint8_t>&)
{
  Handler(ctx, seq);
}

template <void (*Handler)(FirmwareContext&, uint16_t, const std::vector<uint8_t>&)>
void routeWithPayload(FirmwareContext& ctx, uint16_t seq, const std::vector<uint8_t>& payload)
{
  Handler(ctx, seq, payload);
}

constexpr std::array<CommandRoute, 14> COMMAND_ROUTES{{
    {SET_POWER_RELAY, {PayloadPolicyType::ExactBytes, 1}, routeWithPayload<handleSetPowerRelayCommand>},
    {SET_SERVOS_ENABLED, {PayloadPolicyType::ExactBytes, kProtocolServoEnablePayloadBytes}, routeWithPayload<handleSetServosEnabledCommand>},
    {GET_SERVOS_ENABLED, {PayloadPolicyType::ExactBytes, 0}, routeNoPayload<handleGetServosEnabledCommand>},
    {SET_SERVOS_TO_MID, {PayloadPolicyType::ExactBytes, 0}, routeNoPayload<handleSetServosToMidCommand>},
    {GET_LED_INFO, {PayloadPolicyType::ExactBytes, 0}, routeNoPayload<handleGetLedInfoCommand>},
    {SET_LED_COLORS, {PayloadPolicyType::ExactBytes, kProtocolLedColorsPayloadBytes}, routeWithPayload<handleSetLedColorsCommand>},
    {GET_ANGLE_CALIBRATIONS, {PayloadPolicyType::ExactBytes, 0}, routeNoPayload<handleGetAngleCalibCommand>},
    {GET_CURRENT, {PayloadPolicyType::ExactBytes, 0}, routeNoPayload<handleGetCurrentCommand>},
    {GET_VOLTAGE, {PayloadPolicyType::ExactBytes, 0}, routeNoPayload<handleGetVoltageCommand>},
    {GET_SENSOR, {PayloadPolicyType::ExactBytes, 1}, routeWithPayload<handleGetSensorCommand>},
    {GET_FULL_HARDWARE_STATE, {PayloadPolicyType::ExactBytes, 0}, routeNoPayload<handleGetFullHardwareStateCommand>},
    {SET_ANGLE_CALIBRATIONS, {PayloadPolicyType::ExactBytes, kProtocolCalibrationsPayloadBytes}, routeWithPayload<handleCalibCommand>},
    {SET_TARGET_ANGLE, {PayloadPolicyType::ExactBytes, sizeof(uint8_t) + sizeof(float)}, routeWithPayload<handleSetAngleCommand>},
    {SET_JOINT_TARGETS, {PayloadPolicyType::ExactBytes, kProtocolJointTargetsPayloadBytes}, routeWithPayload<handleSetJointTargetsCommand>},
}};

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
