/* coding: utf-8 */
#include "packet.h"
#include "action.h"
#include "cfg.h"
#include "robot.h"
#include "misc.h"
#include "tim.h"
#include "pid.h"
#include "FreeRTOS.h"
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include <stdlib.h>

extern timer_t rf_comm_tim; 
char packet_flag;
extern char shooter;


/*******************************************************************************
* @brief 通过包头(data[1])获取包类型
* @param data 接收到的数据包
* @param len 数据包长度 
* @author Xuanting Liu
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
		case DEBUG_FUNCTION_Normal:  
			return PACKET_Normal;

		case DEBUG_FUNCTION_Set_9557_error:
			return PACKET_DEBUG_Set_9557_error; 

		default:
			return PACKET_NONE;
	}
}

/******************************************************************************
 * @brief 接收到stop标志位，进行另一种协议的回包,参见协议0.3比赛暂停时分帧返回包格式
 * @author Xuanting Liu
 ******************************************************************************/
void stop_mode_packet(char *q)
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
	
	q[5] = (g_robot.is_ball_detected << 7);	//这里只做了红外标志一位，别的还有待添加
	packet_flag = 1;
}

/******************************************************************************
 * @brief 打包函数，将所需返回上位机的信息打包，并检测是否有状态变化(平射、挑射、红外有一种变化)，当发生状态变化时，置位上传标志 packet_flag.
 * @author Xuanting Liu
 ******************************************************************************/
void packet(char *q)
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
    now_infra = g_robot.is_ball_detected;

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
    q[2] = (g_robot.mode == NORMAL_MODE) ? ((g_robot.num-1) & 0x0F) : (g_robot.num & 0x0F);
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
}

/*******************************************************************************
* @brief 解包函数，在 NORMAL_MODE 和 CRAY_NODE 下规则不同
* @return 0 成功，-1 失败
* @author Xuanting Liu
*******************************************************************************/
int decode_packet( packet_robot_t *packet, unsigned char *data, int len )
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
	
    switch (g_robot.mode)
    {
        case NORMAL_MODE:
        {
            // 判断是否有自自己的数据
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
            break;
        }
        case CRAY_MODE:
        {
            // 如果车号大于8，那packet中车号放在data[1]的低4位，否则在data[2]中 
            if(g_robot.num > 8)
                if( ( (data[1] & 0x0f) & (0x01 << (g_robot.num - 9)) ) == 0 )
                    return  -1;  	
            else
                if( ((data[2] & 0xff) & (0x01 << (g_robot.num - 1)) ) == 0 )  
                    return  -1;  	

            // 收到自己的数据，通讯溢出清零。
            rf_comm_tim = get_one_timer(COMM_TIMEOUT_TIME);

            // 查找packet中，一共包含多少个车的数据，并找到自己车数据的位置 
            pos = 0;
            if(g_robot.num < 9) //ROBOT1~8
            {
                for(i = 0; i < (g_robot.num - 1); i++)
                    if(data[2] & (0x01 << i))   pos++;  
            }
            else  //robot9~12
            {
                for(i = 0; i < 8; i++)
                    if(data[2] & (0x01 << i))   pos++;	

                for(i = 0; i < (g_robot.num - 9); i++)
                    if((data[1] & 0x0f) & (0x01 << i))  pos++;
            }
            break;
        }
    }
	
		
    i = pos * 4 + 3; //数据起始处
	
   	packet->robot_num = g_robot.num;

	// set robot value from packet data 
	temp = data[i];
	packet->dribbler = ((( temp >> 4 ) & 0x03));	//吸球力度
	packet->dribbler = (( temp & 0x80) ? (-packet->dribbler) : packet->dribbler); //滚筒向前or向后转
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