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
#define MOTOR_DIR3_Pin GPIO_PIN_2
#define MOTOR_DIR3_GPIO_Port GPIOE
#define MOTOR_DIR2_Pin GPIO_PIN_3
#define MOTOR_DIR2_GPIO_Port GPIOE
#define MOTOR_BRK1_Pin GPIO_PIN_4
#define MOTOR_BRK1_GPIO_Port GPIOE
#define MOTOR_BRK4_Pin GPIO_PIN_5
#define MOTOR_BRK4_GPIO_Port GPIOE
#define MOTOR_BRK3_Pin GPIO_PIN_6
#define MOTOR_BRK3_GPIO_Port GPIOE
#define SHOOT_OFF_Pin GPIO_PIN_13
#define SHOOT_OFF_GPIO_Port GPIOC
#define INT_Pin GPIO_PIN_14
#define INT_GPIO_Port GPIOC
#define FSYNC_Pin GPIO_PIN_15
#define FSYNC_GPIO_Port GPIOC
#define IR_BALL_DECT_Pin GPIO_PIN_0
#define IR_BALL_DECT_GPIO_Port GPIOC
#define IR_BALL_DECT_EXTI_IRQn EXTI0_IRQn
#define BUZZER_Pin GPIO_PIN_1
#define BUZZER_GPIO_Port GPIOC
#define ENCODER_A1_Pin GPIO_PIN_0
#define ENCODER_A1_GPIO_Port GPIOA
#define ENCODER_B1_Pin GPIO_PIN_1
#define ENCODER_B1_GPIO_Port GPIOA
#define SHOOT_Pin GPIO_PIN_2
#define SHOOT_GPIO_Port GPIOA
#define CHIP_Pin GPIO_PIN_3
#define CHIP_GPIO_Port GPIOA
#define LED_CLK_Pin GPIO_PIN_5
#define LED_CLK_GPIO_Port GPIOA
#define ENCODER_A4_Pin GPIO_PIN_6
#define ENCODER_A4_GPIO_Port GPIOA
#define ENCODER_B4_Pin GPIO_PIN_7
#define ENCODER_B4_GPIO_Port GPIOA
#define CHG_EN_Pin GPIO_PIN_4
#define CHG_EN_GPIO_Port GPIOC
#define ADC_CAP_Pin GPIO_PIN_0
#define ADC_CAP_GPIO_Port GPIOB
#define ADC_BAT_Pin GPIO_PIN_1
#define ADC_BAT_GPIO_Port GPIOB
#define MOTOR_BRK2_Pin GPIO_PIN_7
#define MOTOR_BRK2_GPIO_Port GPIOE
#define MOTOR_BRK5_Pin GPIO_PIN_8
#define MOTOR_BRK5_GPIO_Port GPIOE
#define MOTOR_PWM1_Pin GPIO_PIN_9
#define MOTOR_PWM1_GPIO_Port GPIOE
#define MOTOR_DIR5_Pin GPIO_PIN_10
#define MOTOR_DIR5_GPIO_Port GPIOE
#define MOTOR_PWM4_Pin GPIO_PIN_11
#define MOTOR_PWM4_GPIO_Port GPIOE
#define MOTOR_PWM3_Pin GPIO_PIN_13
#define MOTOR_PWM3_GPIO_Port GPIOE
#define MOTOR_PWM2_Pin GPIO_PIN_14
#define MOTOR_PWM2_GPIO_Port GPIOE
#define SCL_Pin GPIO_PIN_10
#define SCL_GPIO_Port GPIOB
#define SDA_Pin GPIO_PIN_11
#define SDA_GPIO_Port GPIOB
#define LED_PWR_Pin GPIO_PIN_10
#define LED_PWR_GPIO_Port GPIOD
#define SW_LD_Pin GPIO_PIN_7
#define SW_LD_GPIO_Port GPIOC
#define SW_CLK_Pin GPIO_PIN_8
#define SW_CLK_GPIO_Port GPIOC
#define SW_D_Pin GPIO_PIN_9
#define SW_D_GPIO_Port GPIOC
#define LED_COMM_Pin GPIO_PIN_8
#define LED_COMM_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_9
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_10
#define USART_RX_GPIO_Port GPIOA
#define ENCODER_A2_Pin GPIO_PIN_15
#define ENCODER_A2_GPIO_Port GPIOA
#define SCK_Pin GPIO_PIN_10
#define SCK_GPIO_Port GPIOC
#define MISO_Pin GPIO_PIN_11
#define MISO_GPIO_Port GPIOC
#define MOSI_Pin GPIO_PIN_12
#define MOSI_GPIO_Port GPIOC
#define IRQ_Pin GPIO_PIN_0
#define IRQ_GPIO_Port GPIOD
#define CSN_Pin GPIO_PIN_1
#define CSN_GPIO_Port GPIOD
#define CE_Pin GPIO_PIN_2
#define CE_GPIO_Port GPIOD
#define ENCODER_B2_Pin GPIO_PIN_3
#define ENCODER_B2_GPIO_Port GPIOB
#define LED_DAT_Pin GPIO_PIN_5
#define LED_DAT_GPIO_Port GPIOB
#define ENCODER_A3_Pin GPIO_PIN_6
#define ENCODER_A3_GPIO_Port GPIOB
#define ENCODER_B3_Pin GPIO_PIN_7
#define ENCODER_B3_GPIO_Port GPIOB
#define IR_PWM_Pin GPIO_PIN_8
#define IR_PWM_GPIO_Port GPIOB
#define MOTOR_PWM5_Pin GPIO_PIN_9
#define MOTOR_PWM5_GPIO_Port GPIOB
#define MOTOR_DIR1_Pin GPIO_PIN_0
#define MOTOR_DIR1_GPIO_Port GPIOE
#define MOTOR_DIR4_Pin GPIO_PIN_1
#define MOTOR_DIR4_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
