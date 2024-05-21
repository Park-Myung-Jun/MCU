/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CMD_BUFFER_SIZE 64
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t uart2_rx_data = 0;

uint8_t print_test_idx = 0;
//led
uint8_t led_toggle = 0;
uint16_t led_duty = 0;
uint8_t led_pwm_is_start = 0;
// cmd
uint8_t cmd_buffer[CMD_BUFFER_SIZE] = {0};
uint8_t cmd_index = 0;
uint8_t cmd_activate = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_USB_DEVICE_Init();
  MX_ADC_Init();
  /* USER CODE BEGIN 2 */
  DMA_ADC_Initialize();
  ADC_Initialize();

  HAL_UART_Receive_IT(&huart2, &uart2_rx_data, 1);

  //HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  LL_TIM_EnableIT_CC1(TIM2); // Use Interrupt
  LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH1);
  LL_TIM_EnableCounter(TIM2);

  led_pwm_is_start = 1;

  printf("STM32L053R8T6 LL Initialize!!\r\n\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
#if 1
	if(cmd_activate == 1)
	{
		//__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, led_duty);
		LL_TIM_OC_SetCompareCH1(TIM2, led_duty);

		cmd_activate = 0;
	}
#elif 0
	printf("print test %d\r\n", print_test_idx++);
	HAL_Delay(500);

	led_duty += 100;
	if(led_duty > 1000)
	{
		led_duty = 0;
	}
#elif 0
	printf("print test %d\r\n", print_test_idx++);
	HAL_Delay(500);

	if(led_toggle == 0)
	{
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1000);
		led_toggle = 1;
	}
	else
	{
		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
		led_toggle = 0;
	}
#endif
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_USB;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
int _write(int fd, char *str, int len)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)str, len, 0xFFFF);
  CDC_Transmit_FS((uint8_t*)str, len);

  return len;
}

void Put_Char(uint8_t data)
{
  HAL_UART_Transmit(&huart2, &data, 1, HAL_MAX_DELAY);
  CDC_Transmit_FS((uint8_t*)&data, 1);
}

void Put_String(const char *format, ...)
{
  va_list ap;
  char buf[64] = {0};
  uint8_t len = 0;

  va_start(ap, format);
  vsprintf(buf, format, ap);

  len = strlen(buf);

  HAL_UART_Transmit(&huart2, (uint8_t *)buf, len, 0xFFFF);
  CDC_Transmit_FS((uint8_t*)buf, len);

  va_end(ap);
}

void Cmd_Input(uint8_t rx_data)
{
	if(rx_data == '\r' || rx_data == '\n')
	{
		led_duty = atoi((char*)cmd_buffer);

		if(led_duty > 1010)
		{
			if(led_pwm_is_start == 1)
			{
				//HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
				LL_TIM_DisableIT_CC1(TIM2); // Use Interrupt
				LL_TIM_CC_DisableChannel(TIM2, LL_TIM_CHANNEL_CH1);
				LL_TIM_DisableCounter(TIM2);

				led_pwm_is_start = 0;
			}
			else
			{
				//HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
				LL_TIM_EnableIT_CC1(TIM2); // Use Interrupt
				LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH1);
				LL_TIM_EnableCounter(TIM2);

				led_pwm_is_start = 1;
			}
		}
		else
		{
			cmd_activate = 1;
		}

		Put_String("\r\n%s\r\n > ", cmd_buffer);

		memset(cmd_buffer, 0x00, CMD_BUFFER_SIZE);
		cmd_index = 0;

		return;
	}

	if(cmd_buffer[cmd_index] == CMD_BUFFER_SIZE)
	{
		return;
	}

	if('0' <= rx_data && rx_data <= '9')
	{
		cmd_buffer[cmd_index] = rx_data;
		cmd_index++;
		Put_String("%c", rx_data);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USART2)
  {
	Cmd_Input(uart2_rx_data);

	HAL_UART_Receive_IT(&huart2, &uart2_rx_data, 1);
  }
}

void LL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == BUTTON_BLUE_PC13_Pin)
	{
		//if(HAL_GPIO_ReadPin(BUTTON_BLUE_PC13_GPIO_Port, BUTTON_BLUE_PC13_Pin) == GPIO_PIN_SET)
		if(LL_GPIO_ReadInputPort(BUTTON_BLUE_PC13_GPIO_Port) & GPIO_IDR_ID13)
		{
			//HAL_GPIO_WritePin(GPIO_OUTPUT_PC14_GPIO_Port, GPIO_OUTPUT_PC14_Pin, GPIO_PIN_RESET);
			LL_GPIO_ResetOutputPin(GPIO_OUTPUT_PC14_GPIO_Port, GPIO_OUTPUT_PC14_Pin);
			Put_String("Button Pull, GPIO_OUTPUT_PC14 RESET\r\n");
		}
		else
		{
			//HAL_GPIO_WritePin(GPIO_OUTPUT_PC14_GPIO_Port, GPIO_OUTPUT_PC14_Pin, GPIO_PIN_SET);
			LL_GPIO_SetOutputPin(GPIO_OUTPUT_PC14_GPIO_Port, GPIO_OUTPUT_PC14_Pin);
			Put_String("Button Push, GPIO_OUTPUT_PC14 SET\r\n");
		}

		Put_String("GPIO_OUTPUT_PC14 BSRR %ld\r\n", READ_BIT(GPIO_OUTPUT_PC14_GPIO_Port->BSRR, GPIO_OUTPUT_PC14_Pin) >> 14);
		Put_String("GPIO_OUTPUT_PC14 BRR %ld\r\n", READ_BIT(GPIO_OUTPUT_PC14_GPIO_Port->BRR, GPIO_OUTPUT_PC14_Pin) >> 14);
		Put_String("GPIO_OUTPUT_PC14 ODR %ld\r\n", READ_BIT(LL_GPIO_ReadOutputPort(GPIO_OUTPUT_PC14_GPIO_Port), GPIO_ODR_OD14) >> GPIO_ODR_OD14_Pos);
		//Put_String("GPIO_OUTPUT_PC14 IDR %ld\r\n", HAL_GPIO_ReadPin(GPIO_OUTPUT_PC14_GPIO_Port, GPIO_OUTPUT_PC14_Pin));
		Put_String("GPIO_OUTPUT_PC14 IDR %ld\r\n", READ_BIT(LL_GPIO_ReadInputPort(GPIO_OUTPUT_PC14_GPIO_Port), GPIO_IDR_ID14) >> GPIO_IDR_ID14_Pos);
		//Put_String("GPIO_INPUT_PC15 IDR %ld\r\n", HAL_GPIO_ReadPin(GPIO_INPUT_PC15_GPIO_Port, GPIO_INPUT_PC15_Pin));
		Put_String("GPIO_INPUT_PC15 IDR %ld\r\n", (LL_GPIO_ReadInputPort(GPIO_INPUT_PC15_GPIO_Port) & GPIO_IDR_ID15) >> GPIO_IDR_ID15_Pos);

		Put_String("ADC %d %d\r\n", ADC_Raw_Data[0], ADC_Raw_Data[1]);
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
