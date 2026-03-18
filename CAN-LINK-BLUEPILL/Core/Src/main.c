#include "main.h"
#include "can_driver.h"

CAN_HandleTypeDef hcan;

void SystemClock_Config(void);

// Dummy values, replaced in real time code with real feedback
static CAN_MotorStatus_t motor_status = {
	.actual_speed = 0,
	.dc_voltage   = CAN_VOLTAGE_TO_RAW(24.0f),
	.current      = CAN_CURRENT_TO_RAW(1.5f),
	.status       = 0,
	.error        = 0x00
};

static uint8_t status_send_flag = 0;

// Sending feedback on 100Hz
void HAL_SYSTICK_Callback(void)
{
	static uint8_t tick_count = 0;

	if (++tick_count >= 10)
	{
		tick_count = 0;
		status_send_flag = 1;
	}
}

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  CAN_Driver_Init(&hcan);
  CAN_Driver_Start(&hcan);

  while (1)
  {
	  if (CAN_HasNewCommand())
	  {
		  CAN_MotorCommand_t cmd = CAN_GetLastCommand(); // Collects new command if existing
		  motor_status.actual_speed = cmd.speed_setpoint;
		  motor_status.status = cmd.enable;
	  }

	  if (status_send_flag)
	  {
		  status_send_flag = 0;
		  CAN_SendMotorStatus(&hcan, &motor_status);
	  }
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
