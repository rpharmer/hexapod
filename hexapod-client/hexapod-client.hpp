// Header guard
#ifndef HEXAPOD_CLIENT_H
#define HEXAPOD_CLIENT_H

#include <cstdint>
#include <vector>

// client device id
const uint8_t DEVICE_ID              = 0x01;

/* A0/A1/A2 Mapping */
const uint32_t A0_GPIO_PIN			  = 26;
const uint32_t A1_GPIO_PIN        = 27;
const uint32_t A2_GPIO_PIN        = 28;
const uint32_t A0_GPIO_MASK       = (1<<A0_GPIO_PIN);
const uint32_t A1_GPIO_MASK       = (1<<A1_GPIO_PIN);
const uint32_t A2_GPIO_MASK       = (1<<A2_GPIO_PIN);
const uint32_t GPIO_OUTPUT_MASK   = 0xFFFFFFFF;
const uint32_t GPIO_INPUT_MASK		= 0x00;
const uint32_t GPIO_HIGH_MASK		  = 0xFFFFFFFF;
const uint32_t GPIO_LOW_MASK		  = 0x00;

enum class HexapodState : uint8_t {
  BOOT = 0,
  WAITING_FOR_HOST = 1,
  ACTIVE = 2,
  STOPPING = 3,
  OFF = 4
};

void echoLoop();
void runCommandLoop();
bool handleHandshake(uint16_t seq, const std::vector<uint8_t>& payload);
void handleSetAngleCommand(uint16_t seq, const std::vector<uint8_t>& payload);
void handleGetAngleCalibCommand(uint16_t seq);
void handleSetPowerRelayCommand(uint16_t seq, const std::vector<uint8_t>& payload);
void handleGetCurrentCommand(uint16_t seq);
void handleGetVoltageCommand(uint16_t seq);
void handleGetSensorCommand(uint16_t seq, const std::vector<uint8_t>& payload);
void handleSetJointTargetsCommand(uint16_t seq, const std::vector<uint8_t>& payload);
void handleGetFullHardwareStateCommand(uint16_t seq);
void handleSetServosEnabledCommand(uint16_t seq, const std::vector<uint8_t>& payload);
void handleGetServosEnabledCommand(uint16_t seq);
void handleSetServosToMidCommand(uint16_t seq);

void handleHeartbeatCommand(uint16_t seq);

void handleCalibCommand(uint16_t seq, const std::vector<uint8_t>& payload);
void calibServos(float calibs[18][2]);

#endif
