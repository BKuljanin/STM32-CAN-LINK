#ifndef CAN_DRIVER_H
#define CAN_DRIVER_H

#include "main.h"

// CAN Node IDs
#define CAN_TX_STD_ID          0x103
#define CAN_RX_FILTER_ID       0x446

// Bit Timing (APB1 = 36 MHz, Baud = 500 kbps)
#define CAN_PRESCALER          18
#define CAN_TIME_SEG1          CAN_BS1_2TQ
#define CAN_TIME_SEG2          CAN_BS2_1TQ
#define CAN_SYNC_JUMP_WIDTH    CAN_SJW_1TQ

// Filter Configuration
#define CAN_FILTER_BANK        10
#define CAN_SLAVE_START_BANK   14
#define CAN_RX_FIFO            CAN_FILTER_FIFO1
#define CAN_RX_IT              CAN_IT_RX_FIFO1_MSG_PENDING

// Message Lengths
#define CAN_MOTOR_CMD_DLC      8
#define CAN_MOTOR_STATUS_DLC   8

// Signal Scaling
#define CAN_SPEED_SCALE        0.1f     // RPM per bit
#define CAN_VOLTAGE_SCALE      0.01f    // Volts per bit
#define CAN_CURRENT_SCALE      0.01f    // Amps per bit

#define CAN_SPEED_TO_RAW(rpm)      ((uint16_t)((rpm) / CAN_SPEED_SCALE))
#define CAN_RAW_TO_SPEED(raw)      ((float)(raw) * CAN_SPEED_SCALE)
#define CAN_VOLTAGE_TO_RAW(v)      ((uint16_t)((v) / CAN_VOLTAGE_SCALE))
#define CAN_RAW_TO_VOLTAGE(raw)    ((float)(raw) * CAN_VOLTAGE_SCALE)
#define CAN_CURRENT_TO_RAW(a)      ((uint16_t)((a) / CAN_CURRENT_SCALE))
#define CAN_RAW_TO_CURRENT(raw)    ((float)(raw) * CAN_CURRENT_SCALE)

// Motor Direction
#define MOTOR_DIR_CW           0
#define MOTOR_DIR_CCW          1

// Motor Command: Nucleo (flight controller) -> Bluepill (motor controller)
// Byte 0-1: speed setpoint (0.1 RPM/bit)
// Byte 2:   enable (0=off, 1=on)
// Byte 3:   direction (0=CW, 1=CCW)
// Byte 4-7: reserved
typedef struct {
	uint16_t speed_setpoint;
	uint8_t enable;
	uint8_t direction;
} CAN_MotorCommand_t;

// Motor Status: Bluepill (motor controller) -> Nucleo (flight controller)
// Byte 0-1: actual speed (0.1 RPM/bit)
// Byte 2-3: DC bus voltage (0.01 V/bit)
// Byte 4-5: phase current (0.01 A/bit)
// Byte 6:   status flags
// Byte 7:   error code
typedef struct {
	uint16_t actual_speed;
	uint16_t dc_voltage;
	uint16_t current;
	uint8_t status;
	uint8_t error;
} CAN_MotorStatus_t;

HAL_StatusTypeDef CAN_Driver_Init(CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef CAN_Driver_Start(CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef CAN_SendMotorStatus(CAN_HandleTypeDef *hcan, const CAN_MotorStatus_t *status);
uint8_t CAN_HasNewCommand(void);
CAN_MotorCommand_t CAN_GetLastCommand(void);

#endif /* CAN_DRIVER_H */
