/* coding: utf-8 */
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
#include "cmsis_os.h"

int wheel_reduction_ratio_x_set; /*减速比*/		
int wheel_reduction_ratio_yz_set; /*减速比*/  //旧轮子减速比为70/22 为3.1818 外圈转1圈，内圈码盘转3.1818圈
int max_shot_strength_set;
u8 is_low_power_cnt = 0;

volatile char g_do_set_receive_mode_flag = 0;
volatile char g_set_receive_mode_flag = 0;

extern timer_t power_mon_timer;
extern timer_t heart_led_timer;
extern timer_t rf_comm_tim;           //发射机通信超时时间
extern timer_t identify_cpuid_tim;   //cpuid认证超时时间 设置为10S
extern timer_t shoot_interval_timer;

int forcestopcounter=0;


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
		osDelay(50);
		BEEP_OFF();
		osDelay(50);
	}
	osDelay(300);
}

/*******************************************************************************
*@author Xuanting Liu
*@brief 通过蜂鸣器显示一个8位二进制数
*@param val 需要显示的8位二进制数
*******************************************************************************/
void Beep_Show_8bit(u8 val)
{
	int i;
	BEEP_ON();
	osDelay(1000);
	BEEP_OFF();
	for(i = 0; i < 8; i++)
	{
		int tmp = (val >> i) & 0x01;
		if(tmp) 
		{
			BEEP_ON();
			osDelay(500);
			BEEP_OFF();
		} 
		else
		{
			BEEP_ON();
			osDelay(100);
			BEEP_OFF();
		}
		osDelay(500);
	}
}

/*******************************************************************************
*@author Xuanting Liu
*@brief 通过蜂鸣器显示一个32位二进制数
*@param val 需要显示的32位二进制数
*******************************************************************************/
void Beep_Show_32bit(u32 val)
{
	int i;
	BEEP_ON();
	osDelay(1000);
	BEEP_OFF();
	for(i = 0; i < 32; i++)
	{
		int tmp = (val >> i) & 0x01;
		if(tmp) 
		{
			BEEP_ON();
			osDelay(500);
			BEEP_OFF();
		} 
		else
		{
			BEEP_ON();
			osDelay(100);
			BEEP_OFF();
		}
		osDelay(500);
	}
}

/*******************************************************************************
*@author Xuanting Liu
*@brief 若receive_mode_flag被置位，则增加g_set_receive_mode_flag
*******************************************************************************/
void inc_receive_mode_flag(void)
{
	if(g_do_set_receive_mode_flag)
	{
		g_set_receive_mode_flag++;
	}
}

/*******************************************************************************
*@author Xuanting Liu
*@brief 机器人主循环程序
*******************************************************************************/
void do_robot_run(void)
{
		/* check battery voltage */
    #ifdef ENABLE_POWERMON 
        if(check_timer(power_mon_timer))
        {
          power_mon_timer = get_one_timer(POWER_MON_TIME);
          if(do_power_monitor() == 2)//电池电压降低到设定阈值1 则蜂鸣器报警 降到阈值2则停止机器人
			{
				/* battery voltage is too low, I will stop the robot */
				forcestopcounter++;
				if(forcestopcounter >= 5000)
				{
					do_dribbler(0);
					do_move(0,0,0);
					do_shoot(0,0);
					do_chip(0,0);
					BEEP_OFF();
				
					while(1) /* stop robot */
					{
						BEEP_ON();
						set_heart_led(100, 0, 0, 0x1f);
						set_heart_led(100, 0, 0, 0x1f);						
						osDelay(100);
						BEEP_OFF();
						set_heart_led(0, 0, 0, 0x00);
						set_heart_led(0, 0, 0, 0x00);
						osDelay(1000);
					}
				}
			}
		  else
			{
				if(forcestopcounter > 0) forcestopcounter--;
			}
        }
    #endif
		
    #ifdef ENABLE_HEARTBEAT
        /* heart led */
        if(check_timer(heart_led_timer))
        {
          heart_beat();
          heart_led_timer = get_one_timer(HEARTBEAT_TIME);
        }
    #endif
    
	
		/* do robot job */
		switch(g_robot.mode)
		{
			case NORMAL_MODE :
      		case CRAY_MODE :
			{
				// /* process message from comm device( rs232, wireless, etc. ) */
				// do_comm();

			    // /*通讯超时设置为500ms 上位机8ms下发一个数据包 (共2包一包(25Byte)给一队车，另外一包给另一队车)*/
				// if(check_timer(rf_comm_tim)) 
				// {
				// 	/* 防止通讯中断，置位设置通讯为接收模块标志位 */
				// 	g_do_set_receive_mode_flag = 1; 
					
				// 	do_dribbler(0);
				// 	do_move(0,0,0);
				// 	do_shoot(0,0);
				// 	do_chip(0,0);

				// 	start_nRF24L01_RX();	
				// 	rf_comm_tim = get_one_timer(COMM_TIMEOUT_TIME);
				// 	identify_cpuid_tim = get_one_timer(IDENTIFY_CPUID_TIMEOUT_TIME);
				// }

				
				// if(g_do_set_receive_mode_flag)	//发送数据包后置1等待数据发送出去后将模式修改为接收模式				
				// {
				// 	/* 将通讯设置为接收模式，并置位可接受标志位 */
				// 	if(g_set_receive_mode_flag >= 3)
				// 	{				
				// 		start_nRF24L01_RX();
					
				// 		g_set_receive_mode_flag = 0;
				// 		g_do_set_receive_mode_flag = 0; 	//置位可接收标志位
				// 	}
				// }

				break;
			} 

			case SELFTEST_MODE: //自检模式
			{ 
				static int test_time = 0;	
				test_time++;
				COMM_LED_ON();
				osDelay(2000);
			    COMM_LED_OFF();	

				set_test_shooter();
                do_dribbler(1);//设置控制档位
                osDelay(2000);
				do_dribbler(0);
				do_shoot(20, 0);//平射
		        osDelay(2000);
				do_chip(0, 20); //挑射
                osDelay(2000);		

                if(test_time == 1)
                {
					do_acc_handle_move(0, 0, 100);
					osDelay(2000);
					do_acc_handle_move(0, 0, 0);		
					do_acc_handle_move(0, 0, -100);
					osDelay(2000);
					do_acc_handle_move(0, 0, 0);				
			    }
				else if(test_time == 2)
				{
					do_acc_handle_move(0, 0,-100);
					osDelay(2000);
					do_acc_handle_move(0, 0, 0);
					do_acc_handle_move(0, 0, 100);
					osDelay(2000);
					do_acc_handle_move(0, 0,0);
					test_time = 0;
				}
				break;								
		   }				
	  }

		/* 对射门完成之后的延时进行计时 */
    #ifdef ENABLE_SHOOTER
        if(g_robot.mode != SELFTEST_MODE)
            {
              if(check_timer(shoot_interval_timer)) 
				{
					update_shooter();
					shoot_interval_timer = get_one_timer(1);
				}
            }
    #endif
}

/*******************************************************************************
* @brief 电池电压监控
* @return 返回1表示电池电压降到阈值1 返回2表示电池电压降到阈值2 返回0 电池电压正常
* @author Xuanting Liu
*******************************************************************************/
int do_power_monitor(void)
{
	char retflag;
	u8 cap_v;
	
	/* check the cap voltage */
	cap_v = get_cap_v();

	DIS_INT();
	g_robot.cap_v_f = cap_v * CAP_V_ADC_GAIN;
	g_robot.cap_v = cap_v;
	EN_INT();

	retflag = is_power_low();
    if( retflag )
	{ 
		if(is_low_power_cnt < 5)
			is_low_power_cnt++;
		else
		{
			is_low_power_cnt = 5;
		
		#ifdef ENABLE_BEEP
			Debug_Here();	
		#endif
			return retflag;
		}
	}
	else
	{
        is_low_power_cnt = 0;		
		BEEP_OFF();
	}

	return 0;
}