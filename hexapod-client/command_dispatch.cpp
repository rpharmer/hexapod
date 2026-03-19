#include "pico/stdlib.h"
#include "hexapod-common.hpp"
#include "hexapod-client.hpp"
#include "firmware_context.hpp"

namespace {

constexpr int64_t HOST_LIVENESS_TIMEOUT_US = 2000000;

void transitionToHostDisconnectedSafeState()
{
  firmware().servos.disable_all();
  gpio_put_masked(A0_GPIO_MASK, GPIO_LOW_MASK);
  firmware().state = HexapodState::WAITING_FOR_HOST;
}

bool dispatchPowerCommand(const DecodedPacket& packet)
{
  switch(packet.cmd)
  {
    case SET_POWER_RELAY:
      handleSetPowerRelayCommand(packet.seq, packet.payload);
      return true;
    case SET_SERVOS_ENABLED:
      handleSetServosEnabledCommand(packet.seq, packet.payload);
      return true;
    case GET_SERVOS_ENABLED:
      handleGetServosEnabledCommand(packet.seq);
      return true;
    case SET_SERVOS_TO_MID:
      handleSetServosToMidCommand(packet.seq);
      return true;
    default:
      return false;
  }
}

bool dispatchSensingCommand(const DecodedPacket& packet)
{
  switch(packet.cmd)
  {
    case GET_ANGLE_CALIBRATIONS:
      handleGetAngleCalibCommand(packet.seq);
      return true;
    case GET_CURRENT:
      handleGetCurrentCommand(packet.seq);
      return true;
    case GET_VOLTAGE:
      handleGetVoltageCommand(packet.seq);
      return true;
    case GET_SENSOR:
      handleGetSensorCommand(packet.seq, packet.payload);
      return true;
    case GET_FULL_HARDWARE_STATE:
      handleGetFullHardwareStateCommand(packet.seq);
      return true;
    default:
      return false;
  }
}

bool dispatchMotionCommand(const DecodedPacket& packet)
{
  switch(packet.cmd)
  {
    case SET_ANGLE_CALIBRATIONS:
      handleCalibCommand(packet.seq, packet.payload);
      return true;
    case SET_TARGET_ANGLE:
      handleSetAngleCommand(packet.seq, packet.payload);
      return true;
    case SET_JOINT_TARGETS:
      handleSetJointTargetsCommand(packet.seq, packet.payload);
      return true;
    default:
      return false;
  }
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
      else if(dispatchPowerCommand(packet) || dispatchSensingCommand(packet) || dispatchMotionCommand(packet))
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
