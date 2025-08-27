/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#define STATUS1_Pin GPIO_PIN_0
#define STATUS1_GPIO_Port GPIOC
#define RF_M0_Pin GPIO_PIN_0
#define RF_M0_GPIO_Port GPIOA
#define RF_M1_Pin GPIO_PIN_1
#define RF_M1_GPIO_Port GPIOA
#define SD_CS_Pin GPIO_PIN_4
#define SD_CS_GPIO_Port GPIOA
#define STATUS2_Pin GPIO_PIN_5
#define STATUS2_GPIO_Port GPIOC
#define SGU_LED2_Pin GPIO_PIN_2
#define SGU_LED2_GPIO_Port GPIOB
#define SGU_LED1_Pin GPIO_PIN_10
#define SGU_LED1_GPIO_Port GPIOB
#define W25_FLASH_CS_Pin GPIO_PIN_12
#define W25_FLASH_CS_GPIO_Port GPIOB
#define MCU_LED_Pin GPIO_PIN_8
#define MCU_LED_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */
#define L86_TX_Pin GPIO_PIN_12
#define L86_TX_GPIO_Port GPIOC
#define L86_RX_Pin GPIO_PIN_2
#define L86_RX_GPIO_Port GPIOD
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
