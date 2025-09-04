/* coding: utf-8 */
#include <math.h>
#include <stdlib.h>
#include "math.h"
#include "action.h"
#include "tim.h"
#include "cfg.h"
#include "gpio.h"
#include "robot.h"
#include "misc.h"
#include "motor.h"
#include "MPU6050_driver.h"

shooter_t g_shooter[ SHOOTER_NUM ];
char shooter;

/*******************************************************************************
* @brief 平射挑射功能关闭
* @author Xuanting Liu
*******************************************************************************/
int shooter_off(void)
{
	u8 i;
	
	shoot_off();
	chip_off();
	for(i = 0; i < SHOOTER_NUM; i++)
	{
		g_shooter[i].count_down = 0;
	}

	return 0;
}

/*******************************************************************************
* @brief 初始化平射挑射
* @author Xuanting Liu
*******************************************************************************/
int init_shooter(void)
{
	shooter_off();
	return 0;
}

/*******************************************************************************
* @brief 自测模式下调用
* @author Xuanting Liu
*******************************************************************************/
int set_test_shooter(void)
{
	int i; 
	
	for(i = 0; i < SHOOTER_NUM; i++)
	{
		g_shooter[i].count_down = 0;
	}
	return 0;
}

/*******************************************************************************
* @brief 设置吸球档位力度
* @note 给带球电机力度参数（g_robot.dribbler.set）赋值，输入为带球力度档数，档数分为0-1-2-3			
*******************************************************************************/
void do_dribbler( int dribbler )
{
  	int dribbler_temp;
	u8 dir = 0;

	if(dribbler < 0) 
	{
		dribbler_temp = -dribbler;
		dir = 1;
	}
	else 
	{
		dribbler_temp = dribbler;
		dir = 0;
	}		

    if(dribbler_temp == 0)
    {
        g_robot.dribbler = 0;
    }
    else if(dribbler_temp == 1)
    {
        g_robot.dribbler = MOTOR_PWM_PERIOD * 0.5;
    }
    else if(dribbler_temp == 2)
    {
        g_robot.dribbler = MOTOR_PWM_PERIOD * 0.5;
    }
    else if(dribbler_temp == 3)
    {
        g_robot.dribbler = MOTOR_PWM_PERIOD * 0.5;
    }
	else
	{
		g_robot.dribbler = 0;
	}

	set_dribbler(g_robot.dribbler, dir);
}

/*******************************************************************************
* @brief 设置吸球电机的方向和力度
* @author Xuanting Liu
*******************************************************************************/
void set_dribbler(u16 value, u8 dir)
{
#if (NEW_DRIBBLE_MOTOR_DIR)
	if(dir) 
	{
		MOTOR_BALL_DIR_HI();
	}
	else
	{
		MOTOR_BALL_DIR_LOW();
	}
#else
	if(dir) 
	{
		MOTOR_BALL_DIR_LOW();
	}
	else
	{
		MOTOR_BALL_DIR_HI();
	}
#endif
	__HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, value);
}

/*******************************************************************************
* @brief 执行平射挑射动作力度
* @param channel 为1时挑射,为0时平射
* @param value 射门力度范围
* @note 执行射门前检查电容电压和力度大小,只有电容电压足够且射门力度不为0时才能射门.
* @author Xuanting Liu
*******************************************************************************/
int set_shooter( int channel, int value )
{
	if(value <= 0) return 0;
	
	if(g_shooter[ channel ].count_down ) 
	{
		return -1;
	}

	if(GET_CAP_LOW_IO()){
		g_robot.is_cap_low = 0;
	}else{
		g_robot.is_cap_low = 1;
	}

	if((!g_robot.is_cap_low) && value )
	{
		g_shooter[ channel ].strength = value;
		g_shooter[ channel ].count_down = SHOOTER_INTERVAL;

		switch ( channel )
		{
			// channel 0 is shoot , high is active , 
			case SHOOT : 
			{
				shoot_on(value);
				shooter = 0x02;
				break;
			}
			
			// channel 1 is chip ,  high is active , 
			case CHIP :
			{
				chip_on(value);
				shooter = 0x01;
				break;
			}

			default:
			{
				chip_off();
				shoot_off();
				return -1;
			}
		}
	}

 	return 0;
}

/*******************************************************************************
* @brief 执行平射命令
* @note 形参shoot为1时,若嘴里有球才能平射
* @author Xuanting Liu
*******************************************************************************/
void do_shoot( int shoot, int chip )
{
	/* 先保存平射指令 */
	g_robot.shoot = shoot;
  
	if( shoot )
	{
		if ( g_robot.is_ball_detected == 1 )
		{
		   set_shooter( SHOOT, shoot );
		}
	}
}

/*******************************************************************************
* @brief 执行挑射命令
* @note 形参chip为1时,若嘴里有球才能挑射
* @author Xuanting Liu
*******************************************************************************/
void do_chip(int shoot, int chip )
{
  /* 先保存挑射指令 */
	g_robot.chip = chip;
    
  	if( chip )
	{	
		if( g_robot.is_ball_detected == 1 )
		{
			set_shooter( CHIP, chip );
		}
	}
}

/*******************************************************************************
* @brief 更新shoot计数 1ms减1
*					
* Input		      : None
* Output		  : None
* Return		  : None.
*******************************************************************************/
void update_shooter(void)
{
	int i; 
	
	for(i = 0; i < SHOOTER_NUM; i++)
	{
		if(g_shooter[i].count_down > 0)
			g_shooter[i].count_down--;
		else
			g_shooter[i].count_down = 0;
	}
}

/*******************************************************************************
* @brief 限制加速度执行运动
* @param speed_x x方向速度 单位[cm/s]
* @param speed_y y方向速度 单位[cm/s]
* @param speed_rot 旋转速度 单位[0.025rad/s]
* @note 限定小车xy方向上合加速度值小于MAX_ACC,并且设定小车x,y,z方向上移动速度分别为speed_x, speed_y, speed_rot
* @note 单位为国际单位，max_rot=256/40=6.4rad/s	
* @author Xuanting Liu
*******************************************************************************/
void do_acc_handle_move(int speed_x,int speed_y,int speed_rot)
{
	static float last_speed_x = 0;
	static float last_speed_y = 0;

	float acc_x = 0;
	float acc_y = 0;
	float acc_whole = 0;
	float sin_x = 0;
	float sin_y = 0;
	float tmp_float;
	
	acc_x = speed_x - last_speed_x;   //x方向加速度计算 时间单位为通讯包周期
	acc_y = speed_y - last_speed_y;
	acc_whole = acc_x * acc_x + acc_y * acc_y ;

	tmp_float = sqrtf(acc_whole); //计算合加速度
	acc_whole = tmp_float + 0.001f;
	
	sin_x = acc_x / acc_whole;
	sin_y = acc_y / acc_whole;

   	if(acc_whole > MAX_ACC)
  	{
		acc_whole = MAX_ACC;
		acc_x = acc_whole * sin_x;
		acc_y = acc_whole * sin_y;
		speed_x = ceil(last_speed_x + acc_x);
		speed_y = ceil(last_speed_y + acc_y); 
  	}

	do_move(speed_x,speed_y,speed_rot);
	last_speed_x = speed_x;
	last_speed_y = speed_y;

}

/*******************************************************************************
* @brief 执行运动函数
* @param speed_x x方向速度 单位[cm/s]
* @param speed_y y方向速度 单位[cm/s]
* @param speed_rot 旋转速度 单位[0.025rad/s]
* @author Xuanting Liu
*******************************************************************************/
void do_move( int speed_x, int speed_y, int speed_rot )
{
	int i = 0;
 
	/* 线速度 vx, vy, vz are all measured in m/s */
	float vx = (float)(speed_x) / 100;
	float vy = (float)(speed_y)  / 100;    //单位[m/s]
	float vz = (float)(speed_rot) * 0.025f * WHEEL_CENTER_OFFSET; //V=2*pi*r/t = w*r 单位[m/s]

	/* 各轮子线速度(m/s)设定值 */
	for( i = 0; i < CHANNEL_NUM; i++ )
	{
		/* trasnform wheel angle */
		g_robot.wheels[i].speed = ( g_robot.sin_angle[ i ] * vx + g_robot.cos_angle[ i ] * vy + vz );
		g_robot.wheels[i].set = V2N(g_robot.wheels[i].speed);//线速度转换为编码器速度
		
	}
  
	/* change wheels' speed set point, with dis_int() */
	DIS_INT();
	for( i = 0; i < CHANNEL_NUM; i++ )
	{
		g_robot.wheels[i].pid.set = g_robot.wheels[i].set;
	}
	EN_INT();

}

