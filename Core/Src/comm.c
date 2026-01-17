/* coding: utf-8 */
#include <string.h>
#include "packet.h"
#include "robot.h"
#include "misc.h"
#include "tim.h"
#include "comm.h"
#include "action.h"
#include "pid.h"
#include "NRF24L01.h"
#include "math.h"

rf_comm_t g_rf_comm;
unsigned char packet_cnt=0;                 //收包数计数器  
packet_type_t type;                         //数据包类型

extern nRF24L01 nRF24L01_dev;
extern char g_do_set_receive_mode_flag;
extern char packet_flag;

packet_robot_t src_robot_packet;
idenfity_cpuid_struct identify_data = {0};

/*******************************************************************************
* @brief 无线通信部分初始化
* @note 通信模块初始化函数,初始化中首先将通信模块初始化为接受模式.				
*******************************************************************************/
void init_comm(void)
{
	nRF24L01_dev.buf.len = nRF2401_BUFFER_LEN;
	nRF24L01_dev.buf.pos = 0;
	nRF24L01_dev.get_packet = get_nRF24L01_packet;
	nRF24L01_dev.send_packet = send_nRF24L01_packet;
	nRF24L01_dev.init_dev = nrf24l01_init;
	nRF24L01_dev.packet_error = 0;

	nrf24l01_init();
	
	start_nRF24L01_RX();
	set_receive_flag();
	
}

/*******************************************************************************
* @brief 通讯接受函数,首先判定是否收到通讯包,如果收到,将通讯包拷贝至g_rf_comm.buffer中进行解包,并上传一次数据			
* @author Xuanting Liu
*******************************************************************************/
void do_communication(void)
{
	if(get_receive_flag())//接收模式
	{     
		if( ( nRF24L01_dev.get_packet( &nRF24L01_dev ) ) > 0 ) //接收到一个数据包
		{
			clr_receive_flag();
			memcpy( g_rf_comm.buffer, nRF24L01_dev.buf.buf, nRF24L01_dev.buf.pos );
			g_rf_comm.buffer_pos = (unsigned char)nRF24L01_dev.buf.pos;
	                             
			do_packet_process( g_rf_comm.buffer, g_rf_comm.buffer_pos ); 

			if(packet_cnt > 254) packet_cnt = 0;
			else packet_cnt++;
		}
	}
}

/*******************************************************************************
* @brief 对接收到的数据data进行解包,该函数是上层封装，具体的解包函数均位于Packet.c中
* @param data 接收到的数据包
* @param len 数据包长度
* @return 0表示解包成功，-1表示解包失败
* @author Xuanting Liu
*******************************************************************************/
int do_packet_process( unsigned char *data, int len )
{   
	/* determine the packet type and parse it */
    static int temp = 0;
	
    if(temp == 0)
    {
        type = get_packet_type( data, len );//获取数据包模式
        if(type == PACKET_Normal) //比赛模式
        { 
            temp = 1;
        }
    }

	do_comm_up();

	decode_identify_packet( &identify_data, data );
	
    /*----------------------------通常模式下的数据解包--------------------------------*/
	/*-----------------------参见协议0.1比赛时通讯包格式-----------------------------*/
	if(((type == PACKET_Normal) && ((g_robot.mode == NORMAL_MODE)) || (g_robot.mode == CRAY_MODE)))
	{
		/* parse robot command */

		memset( &src_robot_packet, 0, sizeof( src_robot_packet ) ); //每个周期 下发速度值等清0

		if( decode_packet( &src_robot_packet, data, len ) < 0 )
		{
			/* parse error */
			g_rf_comm.packet_error++;
			return -1;
		}
		
		on_robot_command(&src_robot_packet);
		Communication_Success();
	}
	
	return 0;
}

/*******************************************************************************
* @brief 上传执行函数，根据全局变量type的数值来决定运用何种模式进行上传
* @author Xuanting Liu
*******************************************************************************/
void do_comm_up(void)
{
	static char data[24]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; 

	nRF24L01_dev.buf.pos = PACKET_LEN_UP;

	/*----------------------------通常模式下的数据上传--------------------------------*/

	if(type == PACKET_Normal)
	{
			/* 调用packet()打包上传数据 */
			packet(data);
			
		     if(packet_flag == 1)
			 {
				nRF24L01_dev.buf.buf[0]=0xFF;	            
				nRF24L01_dev.buf.buf[1]=0x02;	                
				nRF24L01_dev.buf.buf[2]=data[2]; 
				nRF24L01_dev.buf.buf[3]=data[3];
				nRF24L01_dev.buf.buf[4]=data[4];
				nRF24L01_dev.buf.buf[5]=data[5];
				nRF24L01_dev.buf.buf[6]=data[6];
				nRF24L01_dev.buf.buf[7]=data[7];
				nRF24L01_dev.buf.buf[8]=data[8];
				nRF24L01_dev.buf.buf[9]=data[9];
				nRF24L01_dev.buf.buf[10]=data[10];
				nRF24L01_dev.buf.buf[11]=data[11];
				nRF24L01_dev.buf.buf[12]=data[12];
				nRF24L01_dev.buf.buf[13]=data[13];
				nRF24L01_dev.buf.buf[14]=data[14];
				// memset(&nRF24L01_dev.buf.buf[15], 0, sizeof(unsigned char) * (PACKET_LEN_UP - 15));		
				//memset(&nRF24L01_dev.buf.buf[6], 0, sizeof(unsigned char) * (PACKET_LEN_UP - 6));
				nRF24L01_dev.buf.buf[23]=data[23];	
				nRF24L01_dev.send_packet( &nRF24L01_dev );
				//BEEP_ON();
			 }
			 //else
			 //BEEP_OFF();
	}
	
	g_do_set_receive_mode_flag = 1;
}
