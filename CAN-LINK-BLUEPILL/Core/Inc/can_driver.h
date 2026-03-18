#ifndef CAN_DRIVER_H
#define CAN_DRIVER_H

#include "main.h"

/* CAN Node Identifiers */
#define CAN_TX_STD_ID          0x103
#define CAN_RX_FILTER_ID       0x446

/* Bit Timing (APB1 = 36 MHz, Baud = 500 kbps) */
#define CAN_PRESCALER          18
#define CAN_TIME_SEG1          CAN_BS1_2TQ
#define CAN_TIME_SEG2          CAN_BS2_1TQ
#define CAN_SYNC_JUMP_WIDTH    CAN_SJW_1TQ

/* Filter Configuration */
#define CAN_FILTER_BANK        10
#define CAN_SLAVE_START_BANK   14
#define CAN_RX_FIFO            CAN_FILTER_FIFO1
#define CAN_RX_IT              CAN_IT_RX_FIFO1_MSG_PENDING

/* LED Command Payload */
#define CAN_LED_CMD_LENGTH     2

typedef struct {
	uint8_t delay_ms;
	uint8_t blink_count;
} CAN_LedCommand_t;

HAL_StatusTypeDef CAN_Driver_Init(CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef CAN_Driver_Start(CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef CAN_Driver_SendLedCommand(CAN_HandleTypeDef *hcan, uint8_t delay_ms, uint8_t blink_count);
uint8_t CAN_Driver_HasNewMessage(void);
CAN_LedCommand_t CAN_Driver_GetLastMessage(void);

#endif /* CAN_DRIVER_H */
