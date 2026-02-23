// Header guard
#ifndef HEXAPOD_CLIENT_H
#define HEXAPOD_CLIENT_H

/* A0/A1/A2 Mapping */
#define A0_GPIO_PIN			26
#define A1_GPIO_PIN			27
#define A2_GPIO_PIN			28
#define A0_GPIO_MASK		(1<<A0_GPIO_PIN)
#define A1_GPIO_MASK		(1<<A1_GPIO_PIN)
#define A2_GPIO_MASK		(1<<A2_GPIO_PIN)
#define GPIO_OUTPUT_MASK	0xFFFFFFFF
#define GPIO_INPUT_MASK		0x00
#define GPIO_HIGH_MASK		0xFFFFFFFF
#define GPIO_LOW_MASK		0x00



void handleHandshake();
void handleSetAngleCommand();
void handleGetAngleCalibCommand();
void handleSetPowerRelayCommand();
void handleGetCurrentCommand();
void handleGetVoltageCommand();
void handleGetSensorCommand();

void handleCalibCommand();
void calibServos(float calibs[18][2]);

#endif