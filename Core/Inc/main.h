/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f3xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define user_button_Pin GPIO_PIN_13
#define user_button_GPIO_Port GPIOC
#define user_button_EXTI_IRQn EXTI15_10_IRQn
#define high_charge_relay_Pin GPIO_PIN_0
#define high_charge_relay_GPIO_Port GPIOC
#define low_charge_relay_Pin GPIO_PIN_1
#define low_charge_relay_GPIO_Port GPIOC
#define discharge_relay_Pin GPIO_PIN_2
#define discharge_relay_GPIO_Port GPIOC
#define led_low_charge_Pin GPIO_PIN_3
#define led_low_charge_GPIO_Port GPIOC
#define CS_Pin GPIO_PIN_0
#define CS_GPIO_Port GPIOA
#define RST_Pin GPIO_PIN_1
#define RST_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define DC_Pin GPIO_PIN_4
#define DC_GPIO_Port GPIOA
#define user_led_Pin GPIO_PIN_5
#define user_led_GPIO_Port GPIOA
#define CS_SD_CARD_Pin GPIO_PIN_6
#define CS_SD_CARD_GPIO_Port GPIOA
#define led_high_charge_Pin GPIO_PIN_4
#define led_high_charge_GPIO_Port GPIOC
#define led_discharge_Pin GPIO_PIN_5
#define led_discharge_GPIO_Port GPIOC
#define Button_Mode_Pin GPIO_PIN_7
#define Button_Mode_GPIO_Port GPIOC
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
