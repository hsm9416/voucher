/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#define sw0_Pin GPIO_PIN_0
#define sw0_GPIO_Port GPIOC
#define sw1_Pin GPIO_PIN_1
#define sw1_GPIO_Port GPIOC
#define sw2_Pin GPIO_PIN_2
#define sw2_GPIO_Port GPIOC
#define sw3_Pin GPIO_PIN_3
#define sw3_GPIO_Port GPIOC
#define AD_IO0_Pin GPIO_PIN_0
#define AD_IO0_GPIO_Port GPIOA
#define AD_IO1_Pin GPIO_PIN_1
#define AD_IO1_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_4
#define LED_GPIO_Port GPIOA
#define beep_Pin GPIO_PIN_5
#define beep_GPIO_Port GPIOA
#define AD_IO2_Pin GPIO_PIN_6
#define AD_IO2_GPIO_Port GPIOA
#define AD_IO3_Pin GPIO_PIN_7
#define AD_IO3_GPIO_Port GPIOA
#define sw4_Pin GPIO_PIN_4
#define sw4_GPIO_Port GPIOC
#define sw5_Pin GPIO_PIN_5
#define sw5_GPIO_Port GPIOC
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
