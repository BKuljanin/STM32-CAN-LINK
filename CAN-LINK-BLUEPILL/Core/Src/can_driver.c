#include "can_driver.h"

static CAN_TxHeaderTypeDef tx_header;
static CAN_RxHeaderTypeDef rx_header;
static uint8_t rx_data[8];
static uint32_t tx_mailbox;

static volatile uint8_t new_message_flag = 0;
static CAN_LedCommand_t last_command = {0};

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
	tx_header.DLC = CAN_LED_CMD_LENGTH;

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

HAL_StatusTypeDef CAN_Driver_SendLedCommand(CAN_HandleTypeDef *hcan, uint8_t delay_ms, uint8_t blink_count)
{
	uint8_t tx_data[CAN_LED_CMD_LENGTH];

	tx_data[0] = delay_ms;
	tx_data[1] = blink_count;

	return HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data, &tx_mailbox);
}

uint8_t CAN_Driver_HasNewMessage(void)
{
	return new_message_flag;
}

CAN_LedCommand_t CAN_Driver_GetLastMessage(void)
{
	new_message_flag = 0;
	return last_command;
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header, rx_data) != HAL_OK)
	{
		return;
	}

	if (rx_header.DLC == CAN_LED_CMD_LENGTH)
	{
		last_command.delay_ms = rx_data[0];
		last_command.blink_count = rx_data[1];
		new_message_flag = 1;
	}
}
