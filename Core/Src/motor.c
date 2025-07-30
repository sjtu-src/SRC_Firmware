/* coding: utf-8 */
#include "motor.h"
#include "robot.h"
#include "misc.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include "typedef.h"
#include "tim.h"

u8 is_motor_run = 0;

/* encoder tim tab, 0 for wheel, 1 for wheel 2 ... */
TIM_TypeDef * encoder_tab[4] = {TIM5, TIM3, TIM4, TIM2};  //encoder1 encoder2 encoder3 encoder4 

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

/*******************************************************************************
*@author Xuanting Liu
*@brief 执行pid计算并驱动电机
*******************************************************************************/
void do_update_motor(void)
{
	u8 i;
	pid_t *pid_h;
	int pwm_val;
	u8 motor_dir[CHANNEL_NUM];
	u16	motor_pwm[CHANNEL_NUM];
	int cur_speed[CHANNEL_NUM];   //count/s

	if(is_motor_run == 0) return;
	
	/* update the speed */
	update_encoder(cur_speed);
	
	/* calcualte the pid output */
	for(i = 0; i < CHANNEL_NUM; i++)
	{
		g_robot.wheels[i].cur_speed = cur_speed[i];
		
		pid_h = &(g_robot.wheels[i].pid);
		//pwm_val = pid_step(pid_h, cur_speed[i], g_robot.bat_v);
		pwm_val = pid_step(pid_h, cur_speed[i], 16.0f);
		
		if(pwm_val < 0)    //比较寄存器值没有负值
		{
			motor_dir[i] = 1;
			motor_pwm[i] = -pwm_val;
		}
		else
		{
			motor_dir[i] = 0;
			motor_pwm[i] = pwm_val;
		}
	}

	/* update the motor ctrl */
	set_motor_dir(motor_dir[0], motor_dir[1], motor_dir[2], motor_dir[3]);
	set_motor_pwm(motor_pwm[0], motor_pwm[1], motor_pwm[2], motor_pwm[3]);
}


/*******************************************************************************
*@author Xuanting Liu
*@brief  更新编码器读数
*******************************************************************************/
void update_encoder(int *speed)
{
	u8 i;
	u16 time;
	s16 encoder_cnt;
	TIM_TypeDef* TIMx;
	float tmp_f;
	
	/* Disable the TIM Counter */
	__HAL_TIM_DISABLE(&htim7);
	__HAL_TIM_DISABLE(&htim2);
	__HAL_TIM_DISABLE(&htim3);
	__HAL_TIM_DISABLE(&htim4);
	__HAL_TIM_DISABLE(&htim5);

	time = TIM7->CNT;
	if(time == 0) 
	{
		/* timer overflow */
		time = 0xffff;
	}

	for(i = 0; i < 4; i++)  //电机正向计数器增加 反向计数器从0xffff开始递减
	{	
		TIMx = encoder_tab[i];
		encoder_cnt = (s16)(TIMx->CNT & 0xffff);  //TF1 TF2双向双边沿计数 计数值为编码器脉冲的4倍
		tmp_f = (float)encoder_cnt * ((float)ENCODER_TIM_CLK_FREQ *2 / (float)time);  //count/s tim7 计数时钟2M 需要乘以2
		
		*(speed + i) = (int)tmp_f;
	}
	
	start_encoder();//计数器清0 重新计数 
}

/*******************************************************************************
* @brief 开始计数
* @author Xuanting Liu
*******************************************************************************/
void start_encoder(void)
{
	/* Disable the TIM Counter */
	TIM2->CR1 &= (u16)~TIM_CR1_CEN;
	TIM3->CR1 &= (u16)~TIM_CR1_CEN;
	TIM4->CR1 &= (u16)~TIM_CR1_CEN;
	TIM5->CR1 &= (u16)~TIM_CR1_CEN;

	/* clear cnt */
	TIM2->CNT = 0;
	TIM3->CNT = 0;
	TIM4->CNT = 0;
	TIM5->CNT = 0;

	/* start measurement timer, TIM 7 */
	TIM7->CNT = 0x0;
	TIM7->SR = 0x0;
	TIM7->CR1 |= 0x1;   //tim7启动计数

	/* start encoder */
	TIM2->CR1 |= TIM_CR1_CEN;  //开始计数
	TIM3->CR1 |= TIM_CR1_CEN;
	TIM4->CR1 |= TIM_CR1_CEN;
	TIM5->CR1 |= TIM_CR1_CEN;
}