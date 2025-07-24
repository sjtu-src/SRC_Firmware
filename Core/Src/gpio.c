/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */
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

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, MOTOR_DIR3_Pin|MOTOR_DIR2_Pin|MOTOR_BRK1_Pin|MOTOR_BRK4_Pin
                          |MOTOR_BRK3_Pin|MOTOR_BRK2_Pin|MOTOR_BRK5_Pin|MOTOR_DIR5_Pin
                          |MOTOR_DIR1_Pin|MOTOR_DIR4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, BUZZER_Pin|CHG_EN_Pin|SW_LD_Pin|SW_CLK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_CLK_Pin|LED_COMM_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SCL_Pin|SDA_Pin|LED_DAT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED_PWR_Pin|CSN_Pin|CE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PEPin PEPin PEPin PEPin
                           PEPin PEPin PEPin PEPin
                           PEPin PEPin */
  GPIO_InitStruct.Pin = MOTOR_DIR3_Pin|MOTOR_DIR2_Pin|MOTOR_BRK1_Pin|MOTOR_BRK4_Pin
                          |MOTOR_BRK3_Pin|MOTOR_BRK2_Pin|MOTOR_BRK5_Pin|MOTOR_DIR5_Pin
                          |MOTOR_DIR1_Pin|MOTOR_DIR4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PCPin PCPin PCPin PCPin */
  GPIO_InitStruct.Pin = SHOOT_OFF_Pin|INT_Pin|FSYNC_Pin|SW_D_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = IR_BALL_DECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IR_BALL_DECT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PCPin PCPin PCPin PCPin */
  GPIO_InitStruct.Pin = BUZZER_Pin|CHG_EN_Pin|SW_LD_Pin|SW_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PAPin PAPin */
  GPIO_InitStruct.Pin = LED_CLK_Pin|LED_COMM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PBPin PBPin */
  GPIO_InitStruct.Pin = SCL_Pin|SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PDPin PDPin PDPin */
  GPIO_InitStruct.Pin = LED_PWR_Pin|CSN_Pin|CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = LED_DAT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_DAT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 2 */

/*******************************************************************************
*@author Xuanting Liu
*@brief 初始化机器人GPIO输出
*******************************************************************************/
void init_gpio()
{
	COMM_LED_OFF();
	LED_POWER_ON_LOW();
	RGB_LED_CLK_HI();
	RGB_LED_DAT_LOW();
	BEEP_OFF();
	DIP_SW_CLK_LOW();
	DIP_SW_LD_LOW();
	NRF_CE_LOW();
	NRF_CE_HIGH();
}

/* USER CODE END 2 */
