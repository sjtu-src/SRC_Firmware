#include <string.h>
#include "math.h"
#include "gpio.h"
#include "robot.h"
#include "cfg.h"
#include "action.h"
#include "param.h"
#include "pid.h"
#include "misc.h"
#include "motor.h"
#include "comm.h"
#include "MPU6050_driver.h"
#include "packet.h"
#include "NRF24L01.h"
#include "error.h"
#include "simulate_i2c.h"


/*******************************************************************************
*@author Xuanting Liu
*@brief 初始化机器人硬件配置
*******************************************************************************/
void SRC_Robot_Init(void)
{
	int rtn = 0;
	int delay;

	init_gpio();
	init_kiker();
	init_motor();
	init_dribbler();
	init_i2c();
	init_nrf24l01();

	for(delay = 0;delay < 50000000 ; delay++);
	
	#if MPU6050_GYRO_USED
		if(InitMPU6050() == 0) //陀螺仪初始化失败
		{  
			int i = 0;
			error_flag.bit.mpu6050_flag = 1;
			BEEP_ON();
			for(i = 0;i < 10000000 ; i++);
			BEEP_OFF();
			for(i = 0;i < 10000000 ; i++);
		}
	#endif
	EN_INT();

	init_robot();


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
    eeprom_blue_init_flag = param.dat[15];
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
	if(mode == 2) 
		g_robot.rf_mode = RF_BTE;
	else if(mode == 0 || mode == 5 || mode == 7)
		g_robot.rf_mode = RF_24L01;
	     else
	     {
	     }

	//for(i = 0; i < CHANNEL_NUM; i++)
	
		pid_init(&(g_robot.wheels[0].pid), MOTOR_PID_KP1, MOTOR_PID_KI1, MOTOR_PID_KD1);
		pid_init(&(g_robot.wheels[1].pid), MOTOR_PID_KP2, MOTOR_PID_KI2, MOTOR_PID_KD2);
		pid_init(&(g_robot.wheels[2].pid), MOTOR_PID_KP3, MOTOR_PID_KI3, MOTOR_PID_KD3);
		pid_init(&(g_robot.wheels[3].pid), MOTOR_PID_KP4, MOTOR_PID_KI4, MOTOR_PID_KD4);
	#if MPU6050_GYRO_USED
	gyro_pid_init(&gyro_pid, GYRO_PID_KP, GYRO_PID_KI, GYRO_PID_KD);
	#endif
	g_robot.dribbler = 0;

//	g_robot.kv2n = ( (float)wheel_reduction_ratio_x_set + (float)wheel_reduction_ratio_yz_set 
//		* 0.01f ) * ( 4 * ENCODER_COUNTS_PER_TURN_SET) / 2 / (float)PI / WHEEL_RADIUS;

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
