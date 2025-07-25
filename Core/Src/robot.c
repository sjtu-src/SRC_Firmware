#include <string.h>
#include "math.h"
#include "tim.h"
#include "gpio.h"
#include "robot.h"
#include "cfg.h"
#include "action.h"
#include "param.h"
#include "pid.h"
#include "misc.h"
#include "motor.h"
#include "comm.h"
#include "packet.h"
#include "NRF24L01.h"
#include "error.h"
#include "pid.h"

int wheel_reduction_ratio_x_set; /*减速比*/		
int wheel_reduction_ratio_yz_set; /*减速比*/  //旧轮子减速比为70/22 为3.1818 外圈转1圈，内圈码盘转3.1818圈
int max_shot_strength_set;


/*******************************************************************************
*@author Xuanting Liu
*@brief 初始化机器人硬件配置
*******************************************************************************/
void SRC_Robot_Init(void)
{
	int delay;

	init_gpio();
	init_motor();
	init_dribbler();
	init_i2c();
	init_nrf24l01();

	for(delay = 0;delay < 50000000 ; delay++);
	
	EN_INT();

	init_robot();
	init_comm();
}

/*******************************************************************************
*@author Xuanting Liu
*@brief 初始化机器人参数
*******************************************************************************/
void init_robot(void)
{
	param_t param;
	u8 mode;
	u8 freq;
	u8 num;
	u8 i;
	float angle;
	
  	float wheel_angle[ 4 ] = { 
		 D_WHEEL_ANGLE_FRONT,     //左前轮
		-D_WHEEL_ANGLE_FRONT,      //右前轮
		-D_WHEEL_ANGLE_BACK_2013,   //右后轮
		 D_WHEEL_ANGLE_BACK_2013     //左后轮轮
	};
	
	float rotate_modify[4] = {
		FRONT_MODIFY_2024,
		FRONT_MODIFY_2024,
		BACK_MODIFY_2024,
		BACK_MODIFY_2024
	};
		
	
	/* initial parameter from eeprom */
	load_param(&param);
	wheel_reduction_ratio_x_set = param.dat[12];
 	wheel_reduction_ratio_yz_set = param.dat[13];
	if(param.dat[14] <= MAX_SHOT_STRENGTH)
    {
    	max_shot_strength_set = param.dat[14];
    }
    else 
    {
    	max_shot_strength_set = MAX_SHOT_STRENGTH;
    }

	/* initial g_robot */
	read_dip_sw(&freq, &num, &mode);
	
	memset(&g_robot, 0, sizeof(g_robot));
	g_robot.num = num;
	g_robot.frq = freq;
	g_robot.mode = (mode_t)(mode & 0x7);
    mode = mode & 0x7;

	pid_init(&(g_robot.wheels[0].pid), MOTOR_PID_KP1, MOTOR_PID_KI1, MOTOR_PID_KD1);
	pid_init(&(g_robot.wheels[1].pid), MOTOR_PID_KP2, MOTOR_PID_KI2, MOTOR_PID_KD2);
	pid_init(&(g_robot.wheels[2].pid), MOTOR_PID_KP3, MOTOR_PID_KI3, MOTOR_PID_KD3);
	pid_init(&(g_robot.wheels[3].pid), MOTOR_PID_KP4, MOTOR_PID_KI4, MOTOR_PID_KD4);

	g_robot.dribbler = 0;
	g_robot.kv2n = 74037;
	
	/* initial sin and cos table */
	for( i = 0; i < 2; i++ )
	{
	    angle = wheel_angle[ i ] / 180.0f * (float)PI;
	    g_robot.sin_angle[i] = sin( angle ) ;
	    g_robot.cos_angle[i] = cos( angle ) ;
	}
	
	
    for(i = 2; i < 4; i++)
    {
      	angle = wheel_angle[ i ] / 180.0f * (float)PI;
		g_robot.sin_angle[i] = sin(angle) ;
      	g_robot.cos_angle[i] = cos(angle) ;
    }
	
	for(i = 0; i < 4; i++)
	{
		angle = rotate_modify[i] / 180.0f * (float)PI;
		g_robot.cos_mod_angle[i] = cos(angle);
	}
	
    g_robot.firmware_version = software_verison;
	/* initial other */
	init_shooter();

	/* initial ir pwm */
	start_ir_pwm();
}

/*******************************************************************************
*@author Xuanting Liu
*@brief 用于Debug的标志，该函数会使蜂鸣器快速鸣叫5次
*******************************************************************************/
void Debug_Here(void)
{
	int delay;
	for(delay = 0; delay < 5; delay ++)
	{
		BEEP_ON();
		wait_ms_with_dis_int(100);
		BEEP_OFF();
		wait_ms_with_dis_int(100);
	}
}
