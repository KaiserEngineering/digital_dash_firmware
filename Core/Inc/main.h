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
#define FAN_EN_Pin GPIO_PIN_13
#define FAN_EN_GPIO_Port GPIOC
#define PWR_HOLD_Pin GPIO_PIN_1
#define PWR_HOLD_GPIO_Port GPIOC
#define DEBUG_LED_3_Pin GPIO_PIN_2
#define DEBUG_LED_3_GPIO_Port GPIOC
#define PI_GPIO32_Pin GPIO_PIN_2
#define PI_GPIO32_GPIO_Port GPIOA
#define PI_GPIO33_Pin GPIO_PIN_3
#define PI_GPIO33_GPIO_Port GPIOA
#define DEBUG_LED_1_Pin GPIO_PIN_4
#define DEBUG_LED_1_GPIO_Port GPIOA
#define DEBUG_LED_2_Pin GPIO_PIN_5
#define DEBUG_LED_2_GPIO_Port GPIOA
#define FORD_PWM_IN_Pin GPIO_PIN_6
#define FORD_PWM_IN_GPIO_Port GPIOA
#define BKLT_OUT_Pin GPIO_PIN_7
#define BKLT_OUT_GPIO_Port GPIOA
#define CAN_STBY_Pin GPIO_PIN_0
#define CAN_STBY_GPIO_Port GPIOB
#define USB_EN_Pin GPIO_PIN_1
#define USB_EN_GPIO_Port GPIOB
#define LCD_DR_EN_Pin GPIO_PIN_14
#define LCD_DR_EN_GPIO_Port GPIOB
#define LCD_DR_ENR_Pin GPIO_PIN_15
#define LCD_DR_ENR_GPIO_Port GPIOB
#define FAN_PWM_Pin GPIO_PIN_7
#define FAN_PWM_GPIO_Port GPIOC
#define GPIO34_RPI_FAN_EN_Pin GPIO_PIN_15
#define GPIO34_RPI_FAN_EN_GPIO_Port GPIOA
#define CARD_DETECT_SW_Pin GPIO_PIN_11
#define CARD_DETECT_SW_GPIO_Port GPIOC
#define CARD_DETECT_SW_EXTI_IRQn EXTI15_10_IRQn
#define PI_PWR_EN_Pin GPIO_PIN_3
#define PI_PWR_EN_GPIO_Port GPIOB
#define GPIO35_RPI_BKLT_Pin GPIO_PIN_5
#define GPIO35_RPI_BKLT_GPIO_Port GPIOB
#define PI_SHUTDOWN_Pin GPIO_PIN_6
#define PI_SHUTDOWN_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

#define PWR_HOLD_Active   GPIO_PIN_SET
#define PWR_HOLD_Inactive GPIO_PIN_RESET

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
