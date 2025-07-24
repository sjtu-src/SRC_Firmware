/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.h
  * @brief   This file contains all the function prototypes for
  *          the gpio.c file
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
#ifndef __GPIO_H__
#define __GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
#define GET_CAP_LOW_IO()	1

#define GET_BALL_DECET_IO()	((GPIOC->IDR & GPIO_PIN_0) == GPIO_PIN_0)

#define COMM_LED_ON()		HAL_GPIO_WritePin(LED_COMM_GPIO_Port, LED_COMM_Pin, GPIO_PIN_SET)
#define COMM_LED_OFF()	HAL_GPIO_WritePin(LED_COMM_GPIO_Port, LED_COMM_Pin, GPIO_PIN_RESET)

#define BEEP_ON()			HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET)
#define BEEP_OFF()		HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET)

#define GET_NRF_IRQ_IO()	((GPIOD->IDR & GPIO_PIN_0) == GPIO_PIN_0)

#define NRF_CE_LOW()		HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, GPIO_PIN_RESET)
#define NRF_CE_HIGH()		HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, GPIO_PIN_SET)

#define NRF_NCS_LOW()		HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_RESET)
#define NRF_NCS_HIGH()	HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_SET)

#define DIP_SW_CLK_LOW()	HAL_GPIO_WritePin(SW_CLK_GPIO_Port, SW_CLK_Pin, GPIO_PIN_RESET)
#define DIP_SW_CLK_HI()		HAL_GPIO_WritePin(SW_CLK_GPIO_Port, SW_CLK_Pin, GPIO_PIN_SET)	

#define DIP_SW_LD_LOW()		HAL_GPIO_WritePin(SW_LD_GPIO_Port, SW_LD_Pin, GPIO_PIN_RESET)
#define DIP_SW_LD_HI()		HAL_GPIO_WritePin(SW_LD_GPIO_Port, SW_LD_Pin, GPIO_PIN_SET)

#define GET_DIP_SW_D_IO()	((GPIOC->IDR & GPIO_PIN_9) == GPIO_PIN_9)

#define MOTOR_BALL_DIR_LOW()		HAL_GPIO_WritePin(MOTOR_DIR5_GPIO_Port, MOTOR_DIR5_Pin, GPIO_PIN_RESET)
#define MOTOR_BALL_DIR_HI()			HAL_GPIO_WritePin(MOTOR_DIR5_GPIO_Port, MOTOR_DIR5_Pin, GPIO_PIN_SET)

#define MOTOR_BALL_BRK_LOW()		HAL_GPIO_WritePin(MOTOR_BRK5_GPIO_Port, MOTOR_BRK5_Pin, GPIO_PIN_RESET)
#define MOTOR_BALL_BRK_HI()			HAL_GPIO_WritePin(MOTOR_BRK5_GPIO_Port, MOTOR_BRK5_Pin, GPIO_PIN_SET)

#define RGB_LED_CLK_LOW()	HAL_GPIO_WritePin(LED_CLK_GPIO_Port, LED_CLK_Pin, GPIO_PIN_RESET)
#define RGB_LED_CLK_HI()	HAL_GPIO_WritePin(LED_CLK_GPIO_Port, LED_CLK_Pin, GPIO_PIN_SET)

#define RGB_LED_DAT_LOW()	HAL_GPIO_WritePin(LED_DAT_GPIO_Port, LED_DAT_Pin, GPIO_PIN_RESET)
#define RGB_LED_DAT_HI()	HAL_GPIO_WritePin(LED_DAT_GPIO_Port, LED_DAT_Pin, GPIO_PIN_SET)

#define LED_POWER_ON_HIGH()		HAL_GPIO_WritePin(LED_PWR_GPIO_Port, LED_PWR_Pin, GPIO_PIN_SET)
#define LED_POWER_ON_LOW()		HAL_GPIO_WritePin(LED_PWR_GPIO_Port, LED_PWR_Pin, GPIO_PIN_RESET)
/* USER CODE END Private defines */

void MX_GPIO_Init(void);

/* USER CODE BEGIN Prototypes */
void init_gpio(void);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ GPIO_H__ */

