#include "motor.h"
#include "robot.h"
#include "misc.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include "typedef.h"
#include "tim.h"

u8 is_motor_run = 0;

/*******************************************************************************
*@author Xuanting Liu
*@brief 电机初始化
*******************************************************************************/
void init_motor(void)
{
	set_motor_break(1, 1, 1, 1);
	set_motor_dir(0, 0, 0, 0);
	set_motor_pwm(0, 0, 0, 0);

	is_motor_run = 0;
}

/******************************************************************************
*@author Xuanting Liu
*@brief 设置电机转动方向
 ******************************************************************************/
void set_motor_dir(u8 m1, u8 m2, u8 m3, u8 m4)
{
	u16 reset = 0;
	u16 set = 0;
	
	if(m1) set |= MOTOR_DIR1_Pin;
	else reset |= MOTOR_DIR1_Pin;

	if(m2) set |= MOTOR_DIR4_Pin;
	else reset |= MOTOR_DIR4_Pin;

	if(m3) set |= MOTOR_DIR3_Pin;
	else reset |= MOTOR_DIR3_Pin;

	if(m4) set |= MOTOR_DIR2_Pin;
	else reset |= MOTOR_DIR2_Pin;

	HAL_GPIO_WritePin(GPIOE, reset, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, set, GPIO_PIN_SET);
}


/*******************************************************************************
*@author Xuanting Liu
*@brief 设置电机急刹车
*******************************************************************************/
void set_motor_break(u8 m1, u8 m2, u8 m3, u8 m4)
{
	u16 reset = 0;
	u16 set = 0;
	
	if(m1) set |= MOTOR_BRK1_Pin;
	else reset |= MOTOR_BRK1_Pin;

	if(m2) set |= MOTOR_BRK4_Pin;
	else reset |= MOTOR_BRK4_Pin;

	if(m3) set |= MOTOR_BRK3_Pin;
	else reset |= MOTOR_BRK3_Pin;

	if(m4) set |= MOTOR_BRK2_Pin;
	else reset |= MOTOR_BRK2_Pin;

	HAL_GPIO_WritePin(GPIOE, reset, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, set, GPIO_PIN_SET);
}

/*******************************************************************************
*@author Xuanting Liu
*@brief 设置电机PWM
*******************************************************************************/
void set_motor_pwm(u16 m1, u16 m2, u16 m3, u16 m4)
{
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, m1);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, m2);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, m3);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, m4);
}
