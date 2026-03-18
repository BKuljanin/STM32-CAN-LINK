#include "can_driver.h"

static CAN_TxHeaderTypeDef tx_header;
static CAN_RxHeaderTypeDef rx_header;
static uint8_t rx_data[8];
static uint32_t tx_mailbox;

static volatile uint8_t new_command_flag = 0;
static CAN_MotorCommand_t last_command = {0};

HAL_StatusTypeDef CAN_Driver_Init(CAN_HandleTypeDef *hcan)
{
	hcan->Instance = CAN1;
	hcan->Init.Prescaler = CAN_PRESCALER;
	hcan->Init.Mode = CAN_MODE_NORMAL;
	hcan->Init.SyncJumpWidth = CAN_SYNC_JUMP_WIDTH;
	hcan->Init.TimeSeg1 = CAN_TIME_SEG1;
	hcan->Init.TimeSeg2 = CAN_TIME_SEG2;
	hcan->Init.TimeTriggeredMode = DISABLE;
	hcan->Init.AutoBusOff = DISABLE;
	hcan->Init.AutoWakeUp = DISABLE;
	hcan->Init.AutoRetransmission = DISABLE;
	hcan->Init.ReceiveFifoLocked = DISABLE;
	hcan->Init.TransmitFifoPriority = DISABLE;

	if (HAL_CAN_Init(hcan) != HAL_OK)
	{
		return HAL_ERROR;
	}

	CAN_FilterTypeDef filter;

	filter.FilterActivation = CAN_FILTER_ENABLE;
	filter.FilterBank = CAN_FILTER_BANK;
	filter.FilterFIFOAssignment = CAN_RX_FIFO;
	filter.FilterIdHigh = CAN_RX_FILTER_ID << 5;
	filter.FilterIdLow = 0x0000;
	filter.FilterMaskIdHigh = 0x7FF << 5;
	filter.FilterMaskIdLow = 0x0000;
	filter.FilterMode = CAN_FILTERMODE_IDMASK;
	filter.FilterScale = CAN_FILTERSCALE_32BIT;
	filter.SlaveStartFilterBank = CAN_SLAVE_START_BANK;

	if (HAL_CAN_ConfigFilter(hcan, &filter) != HAL_OK)
	{
		return HAL_ERROR;
	}

	tx_header.StdId = CAN_TX_STD_ID;
	tx_header.IDE = CAN_ID_STD;
	tx_header.RTR = CAN_RTR_DATA;
	tx_header.DLC = CAN_MOTOR_STATUS_DLC;

	return HAL_OK;
}

HAL_StatusTypeDef CAN_Driver_Start(CAN_HandleTypeDef *hcan)
{
	HAL_StatusTypeDef status;

	status = HAL_CAN_Start(hcan);
	if (status != HAL_OK)
	{
		return status;
	}

	return HAL_CAN_ActivateNotification(hcan, CAN_RX_IT);
}

HAL_StatusTypeDef CAN_SendMotorStatus(CAN_HandleTypeDef *hcan, const CAN_MotorStatus_t *status)
{
	uint8_t tx_data[CAN_MOTOR_STATUS_DLC];

	tx_data[0] = (uint8_t)(status->actual_speed >> 8);		// High byte
	tx_data[1] = (uint8_t)(status->actual_speed & 0xFF);	// Low byte
	tx_data[2] = (uint8_t)(status->dc_voltage >> 8);
	tx_data[3] = (uint8_t)(status->dc_voltage & 0xFF);
	tx_data[4] = (uint8_t)(status->current >> 8);
	tx_data[5] = (uint8_t)(status->current & 0xFF);
	tx_data[6] = status->status;
	tx_data[7] = status->error;

	return HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data, &tx_mailbox);
}

uint8_t CAN_HasNewCommand(void)
{
	return new_command_flag;
}

CAN_MotorCommand_t CAN_GetLastCommand(void)
{
	new_command_flag = 0;
	return last_command;
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header, rx_data) != HAL_OK)
	{
		return;
	}

	if (rx_header.DLC == CAN_MOTOR_CMD_DLC)
	{
		last_command.speed_setpoint = ((uint16_t)rx_data[0] << 8) | rx_data[1];
		last_command.enable = rx_data[2];
		last_command.direction = rx_data[3];
		new_command_flag = 1;
	}
}
