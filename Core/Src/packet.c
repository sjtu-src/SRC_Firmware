/******************************************************************************
 * packet.c - 
 * 
 * Copyright 2008-2015 Shouxian Chen.
 * 
 * DESCRIPTION: - 
 * 
 * modification history
 * --------------------
 * v1.0   2015/09/11, Shouxian Chen create this file
 * 
 ******************************************************************************/
#include "packet.h"
#include "cfg.h"
#include "robot.h"
#include "misc.h"
#include "timer.h"
#include "pid.h"
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include <stdlib.h>

char packet_flag;

extern int shoot_power_curve_a;

extern int shoot_power_curve_b;

extern float dribbler_power_amp;

extern int max_shot_strength_set;

extern char shooter;

extern timer_t rf_comm_tim;

extern timer_t identify_cpuid_tim;

extern u8  encrpty_cpuid[8];

unsigned int identify_buf_ptr;

unsigned char identify_success = 1;   //认证成功标志位 1 认证成功 0 认证失败 初始值为1 可以先运行10S再进行认证

extern int do_power_monitor(void);

/******************************************************************************
 * stop_mode_packet: 
 *		接收到stop标志位，进行另一种协议的回包,参见协议0.3比赛暂停时分帧返回包格式
 *
 * Parameter:
 * Input: 
 * Output: 
 * Returns: 
 * 
 * modification history
 * --------------------
 *    2015/09/11, Shouxian Chen create this function
 *    2019/04/03, Jiawei Lin 修改解包函数，增加PID调节模式包的格式
 ******************************************************************************/
int stop_mode_packet(char *q)
{
	u8 bat_v;
	u8 cap_v;
	
	q[0] = 0xff;
	q[1] = 0x01;
	q[2] = g_robot.num & 0x0f;
	
	bat_v = get_bat_v();
	cap_v = get_cap_v();

	DIS_INT();
	g_robot.bat_v = bat_v;
	g_robot.cap_v = cap_v;
	
	if(g_robot.bat_v == 0xFF)
	{
		g_robot.bat_v = 0xFE;
	}
	q[3] = g_robot.bat_v;
	
	if(g_robot.cap_v == 0xFF)
	{
		g_robot.cap_v  = 0xFE;
	}
	q[4] = g_robot.cap_v ;
	
	g_robot.bat_v_f = g_robot.bat_v * BAT_V_ADC_GAIN;
	g_robot.cap_v_f = g_robot.cap_v * CAP_V_ADC_GAIN;

	EN_INT();
	
	q[5] = (is_infra_broken() << 7);	//这里只做了红外标志一位，别的还有待添加
	packet_flag = 1;
	
	return 0;
}


/******************************************************************************
 * packet: 
 *		打包函数，将所需返回上位机的信息打包，并检测是否有状态变化(平射、挑射、红外
 *	有一种变化)，当发生状态变化时，置位上传标志 packet_flag.
 *	参见协议0.2比赛时分帧返回包格式.
 *
 * Parameter:
 * Input: 
 * Output: 
 * Returns: 
 * 
 * modification history
 * --------------------
 *    2015/09/11, Shouxian Chen create this function
 * 
 ******************************************************************************/
int packet(char *q)
{
	static int last_infra = 0;
	static int now_infra = 0;

	static int to_shoot = 0;
	static int to_chip = 0;

	static int finish_shoot = 0;
	static int finish_chip = 0;

	static int m = 0;
	static int n = 5;
	
	int speed = 0;
    now_infra = is_ball_detected();

	// 应急策略：告诉上位机红外一直没有测到球
	//now_infra = 0;

	/* 首先，射门命令如果已提交，设置to_shoot */
	if(shooter == 0x02) //shoot
	{
	    to_shoot = 1;
		shooter = 0x00;
	}
        
	if(shooter == 0x01)//chip
	{
		to_chip = 1;
    	shooter = 0x00;
	}
	
    if(now_infra == 0)
    {
		/* 如果现在嘴里没球，但shoot命令有，说明球已经踢出去了 */
        if(to_shoot == 1)
        {
            finish_shoot = 1;
            to_shoot = 0;
			n = 0;
			m++;
			packet_flag = 1;
			// packet_flag = 0;
        }
		
        if(to_chip == 1)
        {
            finish_chip = 1;
            to_chip = 0;
			n = 0;
			m++;
			packet_flag = 1;
		    // packet_flag = 0;
        }    
		
		/* ENABLE_INFRA_BEEP红外叫声 */
		#ifdef ENABLE_INFRA_BEEP
			if(do_power_monitor() == 0)	BEEP_OFF();
		#endif
	}
	else  //吸住球蜂鸣器响
	{
		#ifdef ENABLE_INFRA_BEEP
			BEEP_ON();
		#endif
	}

	/* n记录每个packet发送时的次数, 每个新发的包执行5次 */
	if(n >= 5)
	{
		if(finish_shoot == 1) finish_shoot = 0;
		if(finish_chip == 1) finish_chip = 0;
		
		//if((last_infra != now_infra))
		if((now_infra == 1 ))	// 原逻辑：红外变化则发包，现在逻辑：红外有则发包
		{
			n = 1;
			m++;
			packet_flag = 1;
		}
		else
		{
			packet_flag = 0;
		}
	}
	else
	{
		n++;
		packet_flag = 1;
	}

	if(m == 127)
	{
		m = 0;
	}

	q[0] = 0xff;
    q[1] = 0x02;
    q[2] = (g_robot.num-1) & 0x0F;
	q[3] = (now_infra << 6) + (finish_shoot << 5) + (finish_chip << 4);
	q[4] = m;
	q[5] = n;
	q[6] = 0xf0;
	speed = abs(g_robot.wheels[0].cur_speed) / 10;   
	q[7] = speed % 255;
	q[8] = speed / 255;
	speed = abs(g_robot.wheels[1].cur_speed) / 10;
	q[9] = speed % 255;
	q[10] = speed / 255;
	speed = abs(g_robot.wheels[2].cur_speed) / 10;
	q[11] = speed % 255;
	q[12] = speed / 255;
	speed = abs(g_robot.wheels[3].cur_speed) / 10;
	q[13] = speed % 255;
	q[14] = speed / 255;

	last_infra = now_infra;

	return 0;
}

/******************************************************************************
 * packet_cray: 
 *		打包函数，用于Cray调试模式的打包函数
 *
 * Parameter:
 * Input: 
 * Output: 
 * Returns: 
 * 
 * modification history
 * --------------------
 *    2015/09/11, Shouxian Chen create this function
 * 
 ******************************************************************************/
int packet_cray(char *q)
{
	static int last_infra = 0;
	static int now_infra = 0;

	static int to_shoot = 0;
	static int to_chip = 0;

	static int finish_shoot = 0;
	static int finish_chip = 0;

	static int m = 0;
	static int n = 5;
	
	int speed = 0;
    now_infra = is_ball_detected();

	/* 首先，射门命令如果已提交，设置to_shoot */
	if(shooter == 0x02) //shoot
	{
	    to_shoot = 1;
		shooter = 0x00;
	}
        
	if(shooter == 0x01)//chip
	{
		to_chip = 1;
    	shooter = 0x00;
	}
	
    if(now_infra == 0)
    {
		/* 如果现在嘴里没球，但shoot命令有，说明球已经踢出去了 */
        if(to_shoot == 1)
        {
            finish_shoot = 1;
            to_shoot = 0;
			n = 0;
			m++;
			packet_flag = 1;
        }
		
        if(to_chip == 1)
        {
            finish_chip = 1;
            to_chip = 0;
			n = 0;
			m++;
			packet_flag = 1;
        }    
		
		/* ENABLE_INFRA_BEEP红外叫声 */
		#ifdef ENABLE_INFRA_BEEP
			if(do_power_monitor() == 0)	BEEP_OFF();
		#endif
	}
	else  //吸住球蜂鸣器响
	{
		#ifdef ENABLE_INFRA_BEEP
			BEEP_ON();
		#endif
	}

	/* n记录每个packet发送时的次数, 每个新发的包执行5次 */
	if(n >= 5)
	{
		if(finish_shoot == 1) finish_shoot = 0;
		if(finish_chip == 1) finish_chip = 0;
		
		// if((last_infra != now_infra))
		if((now_infra == 1))	// 原逻辑：红外变化则发包，现在逻辑：红外有则发包
		{
			n = 1;
			m++;
			packet_flag = 1;
		}
		else
		{
			packet_flag = 0;
		}
	}
	else
	{
		n++;
		packet_flag = 1;
	}

	if(m == 127)
	{
		m = 0;
	}

	q[0] = 0xff;
    q[1] = 0x02;
    q[2] = (g_robot.num) & 0x0F;
	q[3] = (now_infra << 6) + (finish_shoot << 5) + (finish_chip << 4);
	q[4] = m;
	q[5] = n;
	q[6] = 0xf0;
	speed = abs(g_robot.wheels[0].cur_speed) / 10;   
	q[7] = speed % 255;
	q[8] = speed / 255;
	speed = abs(g_robot.wheels[1].cur_speed) / 10;
	q[9] = speed % 255;
	q[10] = speed / 255;
	speed = abs(g_robot.wheels[2].cur_speed) / 10;
	q[11] = speed % 255;
	q[12] = speed / 255;
	speed = abs(g_robot.wheels[3].cur_speed) / 10;
	q[13] = speed % 255;
	q[14] = speed / 255;

	last_infra = now_infra;

	return 0;
}

 /*******************************************************************************
* Function Name   : get_packet_type
* Description	  : 获取包的模式，便于之后解包
*					
* Input		      : unsigned char* data, int len 
* Output		  : None
* Return		  : packet_type_t
*******************************************************************************/
packet_type_t get_packet_type( unsigned char* data, int len )
{
	/* check run-time mode */
	volatile unsigned char temp;
	temp = data[1];
	temp = temp & 0xF0;
	temp = temp >> 4;
	
	switch(temp)
	{
		case DEBUG_FUNCTION_Normal:                  	//比赛模式
			return PACKET_Normal;

		case DEBUG_FUNCTION_Set_9557_error:
			return PACKET_DEBUG_Set_9557_error; 

		default:
			return PACKET_Normal;                    	//错误数据包
	}
}


/*******************************************************************************
* Function Name   : decode_packet_robot
* Description	  : 旧的解包，给对应车号的小车传递运动参数，一个包里有三个车的信息
*					
* Input		      : packet_robot_t *packet, unsigned char *data, int len
* Output		  : None
* Return		  : 0
*******************************************************************************/
int decode_packet_robot( packet_robot_t *packet, unsigned char *data, int len )
{
	
	unsigned short temp = 0;
	unsigned char  i=0;  
	u8 pos; 
	unsigned short high_value_x;
	unsigned short high_value_y;
	unsigned short high_value_r;
  
	if(packet == NULL || data == NULL)
		return -1; 

	// 如果车号大于8，那packet中车号放在data[1]的低4位，否则在data[2]中 
    if(g_robot.num > 8)
    {
	    if( ( (data[1] & 0x0f) & (0x01 << (g_robot.num - 9)) ) == 0 )
	    {
		    return  -1;  		//下发数据包没有自己的车号数据则不接受该数据包
	    }
    }
    else
    {
		// 判断数据区是否有自己的数据 
 	    if( ((data[2] & 0xff) & (0x01 << (g_robot.num - 1)) ) == 0 )  
	    { 
	      	return  -1;  	
	    }
    }

	// 收到自己的数据，通讯溢出清零。
	rf_comm_tim = get_one_timer(COMM_TIMEOUT_TIME);

  	// 查找packet中，一共包含多少个车的数据，并找到自己车数据的位置 
	pos = 0;
    if(g_robot.num < 9) //ROBOT1~8
    {
	    for(i = 0; i < (g_robot.num - 1); i++)
	  	{
	  	    if(data[2] & (0x01 << i))
		        pos++; //找车  
		 }
    }
    else  //robot9~12
    {
 	    for(i = 0; i < 8; i++)
	  	{
	  	 	if(data[2] & (0x01 << i))
		 	pos++;	
		}
        for(i = 0; i < (g_robot.num - 9); i++)
    	{
        	if((data[1] & 0x0f) & (0x01 << i))
        	pos++;
     	}
  	}
  	i = pos * 4 + 3; //数据起始处
	
   	packet->robot_num = g_robot.num;

	// set robot value from packet data 
	temp = data[i];
	packet->dribbler = ((( temp >> 4 ) & 0x03));	
	packet->dribbler = (( temp & 0x80) ? (-packet->dribbler) : packet->dribbler);
	//temp = data[i+15]; //射门力度
	temp = data[pos+18]; //射门力度
	
	if( (data[i] >> 6) & 0x1 ) //挑射
	{
		// chip 
    	if(temp >= 127 )
			packet->chip = MAX_SHOT_STRENGTH;
	    else
			packet->chip= temp;

	}
	else //平射
	{   
		// shoot 
  
		
		if(temp >= 127 )
			packet->shoot = MAX_SHOT_STRENGTH;

		else
			packet->shoot = temp;
	}					


	
	temp = data[i+1]; //Speed_x
	packet->speed_x = temp & 0x7F;
	high_value_x = (unsigned short)data[pos + 15];
	high_value_x = ((unsigned short)(high_value_x & 0xc0)) << 1;
	packet->speed_x = packet->speed_x + high_value_x; //速度值+max(0x80) 127+128=256
	temp = data[i+1];
	packet->speed_x = ( ( temp & 0x80 ) ? ( -packet->speed_x ) : packet->speed_x );
		 
	temp = data[i+2]; //speed_y
	packet->speed_y = temp & 0x7F;
	high_value_y = (unsigned short)data[pos + 15];
	high_value_y = ((unsigned short)(high_value_y & 0x30)) << 3;
	packet->speed_y = packet->speed_y+high_value_y;
	temp = data[i+2];
	packet->speed_y = ( ( temp & 0x80 ) ? ( -packet->speed_y ) : packet->speed_y );
		 
	temp = data[i+3]; //speed_rote
	packet->speed_rot = temp & 0x7F;
	high_value_r = (unsigned short)data[pos + 15];
 	high_value_r = ((unsigned short)(high_value_r & 0x0c)) << 5;
	packet->speed_rot = packet->speed_rot+high_value_r;
	temp = data[i+3];
	packet->speed_rot = ( ( temp & 0x80 ) ? ( -packet->speed_rot ) : packet->speed_rot );

	return 0;
	
}



/*******************************************************************************
* Function Name   : decode_packet_robot_new
* Description	  : 新的解包，给对应车号的小车传递运动参数，一个包里有四辆车的信息
* Update	 	  : lxt & zfq 2025-3-23 修改解包逻辑，用三个四位二进制数表示车号，可同时控制15辆车				
* Input		      : packet_robot_t *packet, unsigned char *data, int len
* Output		  : None
* Return		  : 0
*******************************************************************************/
int decode_packet_robot_new( packet_robot_t *packet, unsigned char *data, int len )
{
	unsigned char num1, num2, num3;
	unsigned short temp = 0;
	unsigned char  i=0;  
	u8 pos; 
	unsigned short high_value_x;
	unsigned short high_value_y;
	unsigned short high_value_r;
  
	if(packet == NULL || data == NULL)
		return -1; 
	
	// 判断是否有自自己的数据
	pos = 0 ;

	if( (g_robot.num != (data[1] & 0x0f)) && 
	    (g_robot.num != ((data[2] & 0xf0) >> 4)) &&
	    (g_robot.num != (data[2] & 0x0f)) )
    return -1;

	// 收到自己的数据，通讯溢出清零。
	rf_comm_tim = get_one_timer(COMM_TIMEOUT_TIME);

  	// 查找packet中，一共包含多少个车的数据，并找到自己车数据的位置 
	pos = 0;
	num1 = data[1] & 0x0f;
	num2 = (data[2] & 0xf0) >> 4;
	num3 = data[2] & 0x0f;

	if(num1 < g_robot.num && num1 != 0) pos++;
	if(num2 < g_robot.num && num2 != 0) pos++;
	if(num3 < g_robot.num && num3 != 0) pos++;
		
    i = pos * 4 + 3; //数据起始处
	
   	packet->robot_num = g_robot.num;

	// set robot value from packet data 
	temp = data[i];
	packet->dribbler = ((( temp >> 4 ) & 0x03));	//吸球力度
	packet->dribbler = (( temp & 0x80) ? (-packet->dribbler) : packet->dribbler); //滚筒向前or向后转
	//temp = data[i+15]; //射门力度
	temp = data[pos+20]; //射门力度
	
	if( (data[i] >> 6) & 0x01 ) //挑射
	{
		// chip 
    	if(temp >= 127 )
			packet->chip = MAX_SHOT_STRENGTH;
	    else
			packet->chip= temp;
	}
	else //平射
	{   
		// shoot 
  	if(temp >= 127 )
			packet->shoot = MAX_SHOT_STRENGTH;
		else
			packet->shoot = temp;
	}					
	
	temp = data[i+1]; //Speed_x
	packet->speed_x = temp & 0x7F;
	high_value_x = (unsigned short)data[i];
	high_value_x = ((unsigned short)(high_value_x & 0x0c)) << 5;
	packet->speed_x = packet->speed_x + high_value_x; //速度值+max(0x80) 127+128=256
	temp = data[i+1];
	packet->speed_x = ( ( temp & 0x80 ) ? ( -packet->speed_x ) : packet->speed_x );
		 
	temp = data[i+2]; //speed_y
	packet->speed_y = temp & 0x7F;
	high_value_y = (unsigned short)data[i];
	high_value_y = ((unsigned short)(high_value_y & 0x03)) << 7;
	packet->speed_y = packet->speed_y+high_value_y;
	temp = data[i+2];
	packet->speed_y = ( ( temp & 0x80 ) ? ( -packet->speed_y ) : packet->speed_y );
		 
	temp = data[i+3]; //speed_rote
	packet->speed_rot = temp & 0x7F;
	high_value_r = (unsigned short)data[19];
	switch (pos)
	{
		case 0:high_value_r = ((unsigned short)(high_value_r & 0xc0)) << 1;
			break;
		case 1:high_value_r = ((unsigned short)(high_value_r & 0x30)) << 3;
			break;
		case 2:high_value_r = ((unsigned short)(high_value_r & 0x0c)) << 5;
			break;
		case 3:high_value_r = ((unsigned short)(high_value_r & 0x03)) << 7;
			break;
	}
	packet->speed_rot = packet->speed_rot+high_value_r;
	temp = data[i+3];
	packet->speed_rot = ( ( temp & 0x80 ) ? ( -packet->speed_rot ) : packet->speed_rot );

	return 0;
	
}

/*******************************************************************************
* Function Name   : decode_packet_robot_cray
* Description	  : lxt 2025-3-23 用于电控CrayR使用的解包，车号范围为1~12号（对应拨码0~11）
* Input		      : packet_robot_t *packet, unsigned char *data, int len
* Output		  : None
* Return		  : 0
*******************************************************************************/
int decode_packet_robot_cray( packet_robot_t *packet, unsigned char *data, int len )
{
	
	unsigned short temp = 0;
	unsigned char  i=0;  
	u8 pos; 
	unsigned short high_value_x;
	unsigned short high_value_y;
	unsigned short high_value_r;
  
	if(packet == NULL || data == NULL)
		return -1; 

	// 如果车号大于8，那packet中车号放在data[1]的低4位，否则在data[2]中 
    if(g_robot.num > 8)
    {
	    if( ( (data[1] & 0x0f) & (0x01 << (g_robot.num - 9)) ) == 0 )
	    {
		    return  -1;  		//下发数据包没有自己的车号数据则不接受该数据包
	    }
    }
    else
    {
		// 判断数据区是否有自己的数据 
 	    if( ((data[2] & 0xff) & (0x01 << (g_robot.num - 1)) ) == 0 )  
	    { 
	      	return  -1;  	
	    }
    }

	// 收到自己的数据，通讯溢出清零。
	rf_comm_tim = get_one_timer(COMM_TIMEOUT_TIME);


  	// 查找packet中，一共包含多少个车的数据，并找到自己车数据的位置 
	pos = 0;
    if(g_robot.num < 9) //ROBOT1~8
    {
	    for(i = 0; i < (g_robot.num - 1); i++)
	  	{
	  	    if(data[2] & (0x01 << i))
		        pos++; //找车  
		 }
    }
    else  //robot9~12
    {
 	    for(i = 0; i < 8; i++)
	  	{
	  	 	if(data[2] & (0x01 << i))
		 	pos++;	
		}
        for(i = 0; i < (g_robot.num - 9); i++)
    	{
        	if((data[1] & 0x0f) & (0x01 << i))
        	pos++;
     	}
  	}
  	i = pos * 4 + 3; //数据起始处
	
   	packet->robot_num = g_robot.num;

	// set robot value from packet data 
	temp = data[i];
	packet->dribbler = ((( temp >> 4 ) & 0x03));	//吸球力度
	packet->dribbler = (( temp & 0x80) ? (-packet->dribbler) : packet->dribbler); //滚筒向前or向后转
	//temp = data[i+15]; //射门力度
	temp = data[pos+20]; //射门力度
	
	if( (data[i] >> 6) & 0x01 ) //挑射
	{
		// chip 
    	if(temp >= 127 )
			packet->chip = MAX_SHOT_STRENGTH;
	    else
			packet->chip= temp;
	}
	else //平射
	{   
		// shoot 
  	if(temp >= 127 )
			packet->shoot = MAX_SHOT_STRENGTH;
		else
			packet->shoot = temp;
	}					
	
	temp = data[i+1]; //Speed_x
	packet->speed_x = temp & 0x7F;
	high_value_x = (unsigned short)data[i];
	high_value_x = ((unsigned short)(high_value_x & 0x0c)) << 5;
	packet->speed_x = packet->speed_x + high_value_x; //速度值+max(0x80) 127+128=256
	temp = data[i+1];
	packet->speed_x = ( ( temp & 0x80 ) ? ( -packet->speed_x ) : packet->speed_x );
		 
	temp = data[i+2]; //speed_y
	packet->speed_y = temp & 0x7F;
	high_value_y = (unsigned short)data[i];
	high_value_y = ((unsigned short)(high_value_y & 0x03)) << 7;
	packet->speed_y = packet->speed_y+high_value_y;
	temp = data[i+2];
	packet->speed_y = ( ( temp & 0x80 ) ? ( -packet->speed_y ) : packet->speed_y );
		 
	temp = data[i+3]; //speed_rote
	packet->speed_rot = temp & 0x7F;
	high_value_r = (unsigned short)data[19];
	switch (pos)
	{
		case 0:high_value_r = ((unsigned short)(high_value_r & 0xc0)) << 1;
			break;
		case 1:high_value_r = ((unsigned short)(high_value_r & 0x30)) << 3;
			break;
		case 2:high_value_r = ((unsigned short)(high_value_r & 0x0c)) << 5;
			break;
		case 3:high_value_r = ((unsigned short)(high_value_r & 0x03)) << 7;
			break;
	}
	packet->speed_rot = packet->speed_rot+high_value_r;
	temp = data[i+3];
	packet->speed_rot = ( ( temp & 0x80 ) ? ( -packet->speed_rot ) : packet->speed_rot );

	return 0;
}

/*******************************************************************************
* Function Name   : decode_pid_packet_robot
* Description	  : 解包，给对应车号的小车传递运动参数
*					
* Input		      : packet_robot_t *packet, unsigned char *data, int len
* Output		  : None
* Return		  : 0
*******************************************************************************/
int decode_pid_packet_robot( packet_robot_t *packet, unsigned char *data, int len )
{
	
	unsigned short temp = 0;
	unsigned char  i = 0; 
	unsigned short high_value_x;
	unsigned short high_value_y;
	unsigned short high_value_r;
 	
	//pid参数
	int num = 0; // 轮子编号
	char ctemp[4];
	static char pchar[4], ichar[4], dchar[4];
	float P, I, D;
	// bool changeFlag = false;
	
	if(packet == NULL || data == NULL)
		return -1; 
 
	//判断数据包车号是否与本车相符合
	if( (g_robot.num-1 != (data[1] & 0x0f)) && 
	    (g_robot.num-1 != ((data[2] & 0xf0) >> 4)) &&
	    (g_robot.num-1 != (data[2] & 0x0f)) )
    return -1;
	
	rf_comm_tim = get_one_timer(COMM_TIMEOUT_TIME);
	
	packet->robot_num = g_robot.num;
	
	//吸球
	temp = data[3];
	packet->dribbler = (temp >> 4 & 0x3);
	packet->dribbler = (temp & 0x80 ? -packet->dribbler : packet->dribbler);
	//挑射或平射
	if (temp >> 6 & 0x1)
	{
		//挑射
		if (data[8] > MAX_SHOT_STRENGTH)
			packet->chip = MAX_SHOT_STRENGTH;
		else
			packet->chip = data[8];
	}
	else
	{
		//平射
		if (data[8] > MAX_SHOT_STRENGTH)
			packet->shoot = MAX_SHOT_STRENGTH;
		else
			packet->shoot = data[8];	
	}
	//x速度
	temp = data[4];
	packet->speed_x = temp & 0x7F;
	high_value_x = (unsigned short)data[7];
	high_value_x = ((unsigned short)(high_value_x & 0xc0)) << 1;
	packet->speed_x = packet->speed_x + high_value_x;
	temp = data[4];
	packet->speed_x = ( ( temp & 0x80 ) ? ( -packet->speed_x ) : packet->speed_x );
	//y速度
	temp = data[5];
	packet->speed_y = temp & 0x7F;
	high_value_y = (unsigned short)data[7];
	high_value_y = ((unsigned short)(high_value_y & 0x30)) << 3;
	packet->speed_y = packet->speed_y + high_value_y;
	temp = data[5];
	packet->speed_y = ( ( temp & 0x80 ) ? ( -packet->speed_y ) : packet->speed_y );
	//r速度
	temp = data[6];
	packet->speed_rot = temp & 0x7F;
	high_value_r = (unsigned short)data[7];
 	high_value_r = ((unsigned short)(high_value_r & 0x0c)) << 5;
	packet->speed_rot = packet->speed_rot + high_value_r;
	temp = data[6];
	packet->speed_rot = ( ( temp & 0x80 ) ? ( -packet->speed_rot ) : packet->speed_rot );
	
	for(i = 0; i < 4; i++)ctemp[i] = data[9 + i];
	//if(!strcmp(pchar, ctemp))changeFlag = true;
	memcpy(pchar, ctemp, sizeof(ctemp));
	memcpy(&P, pchar, sizeof(pchar));
	for(i = 0; i < 4; i++)ctemp[i] = data[13 + i];
	//if(!strcmp(ichar, ctemp))changeFlag = true;
	memcpy(ichar, ctemp, sizeof(ctemp));
	memcpy(&I, ichar, sizeof(ichar));
	for(i = 0; i < 4; i++)ctemp[i] = data[17 + i];
	//if(!strcmp(dchar, ctemp))changeFlag = true;
	memcpy(dchar, ctemp, sizeof(ctemp));
	memcpy(&D, dchar, sizeof(dchar));

	//若参数改变，则重新设置PID参数

	temp = data[21];
	num = temp >> 1;
	

	pid_set_param(&(g_robot.wheels[num-1].pid), P, I, D);
	//pid_init(&(g_robot.wheels[i].pid), P, I, D);
	
	
	return 0;

	
}


/*******************************************************************************
* Function Name   : decode_shoot_power_curve_packet_robot
* Description	  : 解包，给对应车号的小车传递运动参数
*					
* Input		      : packet_robot_t *packet, unsigned char *data, int len
* Output		  : None
* Return		  : 0
*******************************************************************************/
int decode_shoot_power_curve_packet_robot( packet_robot_t *packet, unsigned char *data, int len )
{
	
	unsigned short temp = 0;
	unsigned char  i=0;  
	unsigned short high_value_x;
	unsigned short high_value_y;
	unsigned short high_value_r;
 	
		//吸球电机转速增益

	char ctemp[4];
	float amp;
	
	if(packet == NULL || data == NULL)
		return -1; 
 
	//判断数据包车号是否与本车相符合
	if(g_robot.num > 8)
	{
		if (((data[1] & 0x0f) & (0x01 << (g_robot.num - 9))) == 0)return -1;
	}
	else 
	{
		if ((data[2] & (0x01 << (g_robot.num - 1))) == 0)return -1;
	}
	rf_comm_tim = get_one_timer(COMM_TIMEOUT_TIME);
	
	packet->robot_num = g_robot.num;
	
	//吸球
	temp = data[3];
	packet->dribbler = (temp >> 4 & 0x3);
	packet->dribbler = (temp & 0x80 ? -packet->dribbler : packet->dribbler);
	//挑射或平射
	if (temp >> 6 & 0x1)
	{
		//挑射
		if (data[8] > MAX_SHOT_STRENGTH)
			packet->chip = MAX_SHOT_STRENGTH;
		else
			packet->chip = data[8];
	}
	else
	{
		//平射
		if (data[8] > MAX_SHOT_STRENGTH)
			packet->shoot = MAX_SHOT_STRENGTH;
		else
			packet->shoot = data[8];	
	}
	//x速度
	temp = data[4];
	packet->speed_x = temp & 0x7F;
	high_value_x = (unsigned short)data[7];
	high_value_x = ((unsigned short)(high_value_x & 0xc0)) << 1;
	packet->speed_x = packet->speed_x + high_value_x;
	temp = data[4];
	packet->speed_x = ( ( temp & 0x80 ) ? ( -packet->speed_x ) : packet->speed_x );
	//y速度
	temp = data[5];
	packet->speed_y = temp & 0x7F;
	high_value_y = (unsigned short)data[7];
	high_value_y = ((unsigned short)(high_value_y & 0x30)) << 3;
	packet->speed_y = packet->speed_y + high_value_y;
	temp = data[5];
	packet->speed_y = ( ( temp & 0x80 ) ? ( -packet->speed_y ) : packet->speed_y );
	//r速度
	temp = data[6];
	packet->speed_rot = temp & 0x7F;
	high_value_r = (unsigned short)data[7];
 	high_value_r = ((unsigned short)(high_value_r & 0x0c)) << 5;
	packet->speed_rot = packet->speed_rot + high_value_r;
	temp = data[6];
	packet->speed_rot = ( ( temp & 0x80 ) ? ( -packet->speed_rot ) : packet->speed_rot );
	//射门力度曲线参数
	temp = data[9];	
	shoot_power_curve_a = temp & 0x7F;
	temp = data[10];	
	shoot_power_curve_b = temp & 0x7F;
	
	for(i = 0; i < 4; i++)ctemp[i] = data[11 + i];
	memcpy(&amp, ctemp, sizeof(ctemp));
	dribbler_power_amp = amp;
	
	return 0;

	
}

/*******************************************************************************
* Function Name   : compare_data
* Description	  : 数据比较
*					
* Input		      : u8 data[],u8 data1[],int len 
* Output		  : None
* Return		  : 返回1表示输入的2个数组元素值相同 0表示不同
*******************************************************************************/
int compare_data(u8 data[],u8 data1[],int len )
{
     int i;

	 if(data == NULL || data1 == NULL)
		return -1; 
	 for( i = 0; i < len ; i ++)
	 {
	   if(data[i] != data1[i])
	   {
	       return 0; 
	   }
	   
	 }

	 return 1;
	 
}

/*******************************************************************************
* Function Name   : cpuid_identify
* Description	  : cpuid认证包识别
*					
* Input		      : idenfity_cpuid_struct *id_code, unsigned char *data, int len
* Output		  : None
* Return		  : 返回1表示CPUID认证正确 返回0表示认证失败
*******************************************************************************/
int cpuid_identify( idenfity_cpuid_struct *id_code )
{
    int i;
	u8 *cpuid_p;
	u8 rtn = 0;
	cpuid_p = &id_code->recv_cpuid[0];

	for(i = 0; i < (MAX_IDENTIFY_LEN/8) ; i ++) //最多支持32个robot的id认证
	{
	 
	   rtn = compare_data(&encrpty_cpuid[0],cpuid_p,8);
	   if(rtn == 0) //该8个Byte与加密cpuid不相同
	   {
	      cpuid_p += 8; 
	   }
	   else if(rtn == 1)//找到相同的ID则认证成功
	   {
	       return rtn;
	   }
	}
	
	return rtn; //ID 数组表未找到符合的ID
	 

}


/*******************************************************************************
* Function Name   : decode_identify_packet
* Description	  : 解压cpuid认证包
*					详见小型机器人控制器认证文档
* Input		      : idenfity_cpuid_struct *id_code, unsigned char *data, int len
* Output		  : None
* Return		  : 
*******************************************************************************/
int decode_identify_packet( idenfity_cpuid_struct *id_code, unsigned char *data )
{
	
	char i;
    static short identify_packet_cnt = 0;
  
	if(id_code == NULL || data == NULL)
		return -1; 

	/*data[21]表示cpuid认证数据*/
    if(data[IDENTIFY_START_ADDR] & 0x80) //最高位为1则表示身份认证数据包开始
    {  
       identify_buf_ptr = 0;
       identify_packet_cnt = 0;
       id_code->recv_packet_cnt = (data[IDENTIFY_START_ADDR] & 0x7f) + 1;  //认证包需要传的packet个数
       id_code->recv_cpuid_start_flag = 1;
	   
    }
	if(id_code->recv_cpuid_start_flag)
	{   
	    identify_packet_cnt++;  //
	    for(i = 0; i < 2; i ++)
	    {
	        id_code->recv_cpuid[identify_buf_ptr++] = data[IDENTIFY_START_ADDR+1+i];
	    }
		if(identify_packet_cnt == id_code->recv_packet_cnt)//接收完认证数据包则进行认证
		{
		  id_code->recv_cpuid_ok = 1;//认证数据包接收完成
		  id_code->recv_cpuid_start_flag = 0;
		  identify_packet_cnt = 0;
		}
		
	      
	}
	if( id_code->recv_cpuid_ok == 1)
	{
	    if(cpuid_identify(id_code) == 1)//认证成功
	    {
	          /* 身份识别成功，延迟时间清掉重新计时10s*/
	         identify_cpuid_tim = get_one_timer(IDENTIFY_CPUID_TIMEOUT_TIME);//10s
	         identify_success = 1;
			 
	    }
		else  //认证失败 CPUID未注册过
		{
		     identify_success = 0;
			
		}
	     memset(id_code , 0, sizeof(idenfity_cpuid_struct));
	}
	if(identify_buf_ptr >= (MAX_IDENTIFY_LEN - 1))
	{
	    identify_buf_ptr = 0;
	}



	return 0;
}


