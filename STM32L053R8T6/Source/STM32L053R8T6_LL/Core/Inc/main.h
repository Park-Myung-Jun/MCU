/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"

#include "stm32l0xx_ll_adc.h"
#include "stm32l0xx_ll_dma.h"
#include "stm32l0xx_ll_tim.h"
#include "stm32l0xx_ll_system.h"
#include "stm32l0xx_ll_gpio.h"
#include "stm32l0xx_ll_exti.h"
#include "stm32l0xx_ll_bus.h"
#include "stm32l0xx_ll_cortex.h"
#include "stm32l0xx_ll_rcc.h"
#include "stm32l0xx_ll_utils.h"
#include "stm32l0xx_ll_pwr.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BUTTON_BLUE_PC13_Pin LL_GPIO_PIN_13
#define BUTTON_BLUE_PC13_GPIO_Port GPIOC
#define BUTTON_BLUE_PC13_EXTI_IRQn EXTI4_15_IRQn
#define GPIO_OUTPUT_PC14_Pin LL_GPIO_PIN_14
#define GPIO_OUTPUT_PC14_GPIO_Port GPIOC
#define GPIO_INPUT_PC15_Pin LL_GPIO_PIN_15
#define GPIO_INPUT_PC15_GPIO_Port GPIOC
#define USART2_TX_PA2_Pin LL_GPIO_PIN_2
#define USART2_TX_PA2_GPIO_Port GPIOA
#define USART2_RX_PA3_Pin LL_GPIO_PIN_3
#define USART2_RX_PA3_GPIO_Port GPIOA
#define LED_GREEN_PA5_Pin LL_GPIO_PIN_5
#define LED_GREEN_PA5_GPIO_Port GPIOA
#define USB_DM_PA11_Pin LL_GPIO_PIN_11
#define USB_DM_PA11_GPIO_Port GPIOA
#define USB_DP_PA12_Pin LL_GPIO_PIN_12
#define USB_DP_PA12_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */
void LL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
