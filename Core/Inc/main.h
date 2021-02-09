/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
#define RL2_Pin GPIO_PIN_2
#define RL2_GPIO_Port GPIOE
#define RL3_Pin GPIO_PIN_3
#define RL3_GPIO_Port GPIOE
#define RL4_Pin GPIO_PIN_4
#define RL4_GPIO_Port GPIOE
#define RL5_Pin GPIO_PIN_5
#define RL5_GPIO_Port GPIOE
#define RL6_Pin GPIO_PIN_6
#define RL6_GPIO_Port GPIOE
#define Vsense_Pin GPIO_PIN_2
#define Vsense_GPIO_Port GPIOC
#define US2_TX_Pin GPIO_PIN_0
#define US2_TX_GPIO_Port GPIOA
#define US2_RX_Pin GPIO_PIN_1
#define US2_RX_GPIO_Port GPIOA
#define US1_TX_Pin GPIO_PIN_2
#define US1_TX_GPIO_Port GPIOA
#define US1_RX_Pin GPIO_PIN_3
#define US1_RX_GPIO_Port GPIOA
#define US1_ADDR0_Pin GPIO_PIN_7
#define US1_ADDR0_GPIO_Port GPIOE
#define US1_ADDR1_Pin GPIO_PIN_8
#define US1_ADDR1_GPIO_Port GPIOE
#define US1_ADDR2_Pin GPIO_PIN_9
#define US1_ADDR2_GPIO_Port GPIOE
#define US2_ADDR0_Pin GPIO_PIN_10
#define US2_ADDR0_GPIO_Port GPIOE
#define US2_ADDR1_Pin GPIO_PIN_11
#define US2_ADDR1_GPIO_Port GPIOE
#define US2_ADDR2_Pin GPIO_PIN_12
#define US2_ADDR2_GPIO_Port GPIOE
#define BTN1_Pin GPIO_PIN_13
#define BTN1_GPIO_Port GPIOE
#define BTN2_Pin GPIO_PIN_14
#define BTN2_GPIO_Port GPIOE
#define LED_RED_Pin GPIO_PIN_12
#define LED_RED_GPIO_Port GPIOD
#define LED_GREEN_Pin GPIO_PIN_13
#define LED_GREEN_GPIO_Port GPIOD
#define LED_BLUE_Pin GPIO_PIN_14
#define LED_BLUE_GPIO_Port GPIOD
#define RS485_RE_Pin GPIO_PIN_8
#define RS485_RE_GPIO_Port GPIOC
#define RS485_DE_Pin GPIO_PIN_9
#define RS485_DE_GPIO_Port GPIOC
#define RL0_Pin GPIO_PIN_0
#define RL0_GPIO_Port GPIOE
#define RL1_Pin GPIO_PIN_1
#define RL1_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
