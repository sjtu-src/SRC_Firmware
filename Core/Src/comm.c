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
extern nRF24L01 nRF24L01_dev;

/*******************************************************************************
* @brief 无线通信部分初始化
* @note 通信模块初始化函数,初始化中首先将通信模块初始化为接受模式.				
*******************************************************************************/
int init_comm(void)
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
	
	return 0;
}
