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
	unsigned char num1, num2, num3;
	short temp;
	int i;
	static char data[15]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; 

	nRF24L01_dev.buf.pos = PACKET_LEN_UP;

	/*----------------------------通常模式下的数据上传--------------------------------*/

	if(type == PACKET_Normal)
	{
		/* 比赛时数据上传 */
		/* 参见协议3.0比赛暂停时分帧返回包格式 */

		/* 判断stop标志 test-lnc */
		if( (((g_rf_comm.buffer[3] & 0x8) >> 3) == 1) || (((g_rf_comm.buffer[7] & 0x8) >> 3) == 1) 
			|| (((g_rf_comm.buffer[11] & 0x8) >> 3) == 1)) //bit3 为1 则上传模式为2上传电池电容电压
		{
			temp=0;
            packet_flag=0;

			// 根据MODE，选用不同的解包方法判断是否有自己的包
			switch(g_robot.mode)
			{
				case NORMAL_MODE:
				{
					// 判断下发数据包是否有自己的车号 
					if( (g_robot.num != (g_rf_comm.buffer[1] & 0x0f)) && 
						(g_robot.num != ((g_rf_comm.buffer[2] & 0xf0) >> 4)) &&
						(g_robot.num != (g_rf_comm.buffer[2] & 0x0f)) )
					temp = -1;

					if(temp != -1)  //下发数据包与车本身ID一致
					{
						/* 具有自己车的包 */
						/* 将自己的车号在三辆中排序 */
						temp = 0;

						num1 = g_rf_comm.buffer[1] & 0x0f;
						num2 = (g_rf_comm.buffer[2] & 0xf0) >> 4;
						num3 = g_rf_comm.buffer[2] & 0x0f;

						if(num1 < g_robot.num && num1 != 0) temp++;
						if(num2 < g_robot.num && num2 != 0) temp++;
						if(num3 < g_robot.num && num3 != 0) temp++;
					}
					break;
				}

				case CRAY_MODE:
				{
					if(g_robot.num > 8)
						if(((g_rf_comm.buffer[1] & 0x0f) & (0x01 << (g_robot.num - 9))) == 0) 
							temp = -1;   //下发数据包robot ID与控制车ID不一致
					else
						if(((g_rf_comm.buffer[2] & 0xff ) & (0x01 << (g_robot.num - 1))) == 0)	
							temp = -1;  //下发数据包robot ID与控制车ID不一致

					if(temp != -1)  //下发数据包与车本身ID一致
					{
						/* 具有自己车的包，将自己的车号在三辆中排序 */
						temp = 0;
						if(g_robot.num < 9)
						{
							for(i = 0; i < (g_robot.num - 1); i++)
								if(g_rf_comm.buffer[2] & (0x01 <<i )) temp++;
						}
						else
						{
							for(i = 0; i < 8; i++)
								if(g_rf_comm.buffer[2] & (0x01 << i)) temp++; 
							for(i = 0; i < (g_robot.num - 9); i++)
								if((g_rf_comm.buffer[1] & 0x0f) & (0x01 << i)) temp++;
						}
					}
					break;
				}
			}


			if( (temp != -1) && (((g_rf_comm.buffer[3+4*temp] & 0x8) >> 3) == 1) )
			{
				stop_mode_packet(data);

				if(packet_flag == 1)
				{   
					nRF24L01_dev.buf.buf[0]=data[0];
					nRF24L01_dev.buf.buf[1]=data[1];
					nRF24L01_dev.buf.buf[2]=data[2]; 
					nRF24L01_dev.buf.buf[3]=data[3];
					nRF24L01_dev.buf.buf[4]=data[4];
					nRF24L01_dev.buf.buf[5]=data[5]; 
					memset(&nRF24L01_dev.buf.buf[6], 0, sizeof(unsigned char) * (PACKET_LEN_UP - 6));
										  
					nRF24L01_dev.send_packet( &nRF24L01_dev );

					g_do_set_receive_mode_flag = 1;
				}
			}
		}
		else    //bit3 为0 则上传红外 平常就为该模式
		{
			/*--------------参见协议0.2比赛时分帧返回包格式----------------------*/
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
				memset(&nRF24L01_dev.buf.buf[15], 0, sizeof(unsigned char) * (PACKET_LEN_UP - 15));			
				nRF24L01_dev.send_packet( &nRF24L01_dev );
			 }
		}
	}
	
	g_do_set_receive_mode_flag = 1;
}
