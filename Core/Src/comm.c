/* coding: utf-8 */
#include <string.h>
#include <stdint.h>
#include "main.h"
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

#define GAME_TELEMETRY_ENABLE 1
#define GAME_TELEMETRY_HEADER0 0xAA
#define GAME_TELEMETRY_HEADER1 0x55
#define GAME_TELEMETRY_TYPE    0x10
#define GAME_TELEMETRY_SLOT_COUNT 13u
#define GAME_TELEMETRY_SLOT_MS 20u
#define GAME_TELEMETRY_FRAME_LEN 24u

static const uint8_t game_crc8_table[448] =
{
	0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83,
	0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
	0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e,
	0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
	0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0,
	0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
	0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d,
	0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
	0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5,
	0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
	0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58,
	0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
	0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6,
	0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
	0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b,
	0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
	0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f,
	0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
	0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92,
	0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
	0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c,
	0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
	0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1,
	0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
	0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49,
	0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
	0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4,
	0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
	0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a,
	0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
	0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7,
	0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
	0xd8, 0x86, 0x64, 0x3a, 0xb9, 0xe7, 0x05, 0x5b,
	0x1a, 0x44, 0xa6, 0xf8, 0x7b, 0x25, 0xc7, 0x99,
	0x45, 0x1b, 0xf9, 0xa7, 0x24, 0x7a, 0x98, 0xc6,
	0x87, 0xd9, 0x3b, 0x65, 0xe6, 0xb8, 0x5a, 0x04,
	0xfb, 0xa5, 0x47, 0x19, 0x9a, 0xc4, 0x26, 0x78,
	0x39, 0x67, 0x85, 0xdb, 0x58, 0x06, 0xe4, 0xba,
	0x66, 0x38, 0xda, 0x84, 0x07, 0x59, 0xbb, 0xe5,
	0xa4, 0xfa, 0x18, 0x46, 0xc5, 0x9b, 0x79, 0x27,
	0x12, 0x4c, 0xae, 0xf0, 0x73, 0x2d, 0xcf, 0x91,
	0xd0, 0x8e, 0x6c, 0x32, 0xb1, 0xef, 0x0d, 0x53,
	0x8f, 0xd1, 0x33, 0x6d, 0xee, 0xb0, 0x52, 0x0c,
	0x4d, 0x13, 0xf1, 0xaf, 0x2c, 0x72, 0x90, 0xce,
	0x31, 0x6f, 0x8d, 0xd3, 0x50, 0x0e, 0xec, 0xb2,
	0xf3, 0xad, 0x4f, 0x11, 0x92, 0xcc, 0x2e, 0x70,
	0xac, 0xf2, 0x10, 0x4e, 0xcd, 0x93, 0x71, 0x2f,
	0x6e, 0x30, 0xd2, 0x8c, 0x0f, 0x51, 0xb3, 0xed,
	0x54, 0x0a, 0xe8, 0xb6, 0x35, 0x6b, 0x89, 0xd7,
	0x96, 0xc8, 0x2a, 0x74, 0xf7, 0xa9, 0x4b, 0x15,
	0xc9, 0x97, 0x75, 0x2b, 0xa8, 0xf6, 0x14, 0x4a,
	0x0b, 0x55, 0xb7, 0xe9, 0x6a, 0x34, 0xd6, 0x88,
	0x77, 0x29, 0xcb, 0x95, 0x16, 0x48, 0xaa, 0xf4,
	0xb5, 0xeb, 0x09, 0x57, 0xd4, 0x8a, 0x68, 0x36,
	0xea, 0xb4, 0x56, 0x08, 0x8b, 0xd5, 0x37, 0x69,
	0x28, 0x76, 0x94, 0xca, 0x49, 0x17, 0xf5, 0xab
};



/*******************************************************************************
* @brief 计算CRC校验
* @author OpenAI codex	
*******************************************************************************/
static uint8_t game_crc8_calc(const uint8_t *buf, int len)
{
	uint8_t fcs = 0;
	for(int i = 0; i < len; ++i)
	{
		fcs = game_crc8_table[fcs ^ buf[i]];
	}
	return fcs;
}


/*******************************************************************************
* @brief 比赛状态实时回包
* @author OpenAI codex	
*******************************************************************************/
static void build_game_telemetry_packet(uint8_t *out, uint8_t robot_id)
{
	memset(out, 0, PACKET_LEN_UP);

	out[0] = GAME_TELEMETRY_HEADER0;
	out[1] = GAME_TELEMETRY_HEADER1;
	out[2] = GAME_TELEMETRY_TYPE;
	out[3] = robot_id;

	out[4] = (g_robot.is_ball_detected ? 0x01 : 0x00);

	{
		uint8_t bat_v = get_bat_v();
		uint8_t cap_v = (uint8_t)get_cap_v();
		if(bat_v == 0xFF) bat_v = 0xFE;
		if(cap_v == 0xFF) cap_v = 0xFE;
		out[5] = bat_v;
		out[6] = cap_v;
	}

	for(int i = 0; i < 4; ++i)
	{
		int32_t w = (int32_t)(g_robot.wheels[i].cur_speed / 10);
		if(w > 32767) w = 32767;
		if(w < -32768) w = -32768;
		const int offset = 7 + i * 2;
		out[offset] = (uint8_t)(w & 0xFF);
		out[offset + 1] = (uint8_t)((w >> 8) & 0xFF);
	}

	/* bytes [15..22] reserved */
	out[GAME_TELEMETRY_FRAME_LEN - 1] = game_crc8_calc(out, (int)GAME_TELEMETRY_FRAME_LEN - 1);
}

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
				nRF24L01_dev.buf.buf[23]=data[23];	
				nRF24L01_dev.send_packet( &nRF24L01_dev );
			}
			 //else
			 //BEEP_OFF();
	}
	
	g_do_set_receive_mode_flag = 1;
}

void do_game_telemetry_up(void)
{
#if GAME_TELEMETRY_ENABLE
	static uint32_t last_slot = 0xFFFFFFFFu;
	const uint32_t tick = HAL_GetTick();
	const uint32_t slot = (tick / GAME_TELEMETRY_SLOT_MS) % GAME_TELEMETRY_SLOT_COUNT;

	if(slot == last_slot)
	{
		return;
	}
	last_slot = slot;

	if(type != PACKET_Normal)
	{
		return;
	}

	const uint8_t robot_id = (uint8_t)((g_robot.num - 1) & 0x0f);
	if(slot != (uint32_t)(robot_id % GAME_TELEMETRY_SLOT_COUNT))
	{
		return;
	}

	nRF24L01_dev.buf.pos = PACKET_LEN_UP;
	build_game_telemetry_packet((uint8_t*)nRF24L01_dev.buf.buf, robot_id);
	nRF24L01_dev.send_packet(&nRF24L01_dev);
	g_do_set_receive_mode_flag = 1;
#endif
}
