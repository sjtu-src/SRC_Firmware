#include "NRF24L01.h"
#include "main.h"
#include "spi.c"
#include "typedef.h"
#include "gpio.h"
#include "nrf24l01_drv.h"
#include "cfg.h"
#include "robot.h"

#define CE_LOW HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, GPIO_PIN_RESET)
#define CE_HIGH HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, GPIO_PIN_SET)
#define CS_LOW HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_RESET)
#define CS_HIGH HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_SET)

int receive_flag;
nRF24L01 nRF24L01_dev;

/******************************************************************************
 * @brief nrf24L01 SPI通信初始化
 * @author Xuanting Liu
 ******************************************************************************/
void init_nrf24l01()
{
    CE_LOW;
    CS_HIGH;
    __HAL_SPI_ENABLE(&HAL_NRF24L01_SPI);
}

/******************************************************************************
 * @brief 从nrf24L01中获取一包数据
 * @author Shouxian Chen
 ******************************************************************************/
int get_nRF24L01_packet(nRF24L01 *dev)
{
	u8 len;
	dev->buf.pos=0;

	/* check if IRQ pin is low, I enabled only the RX IRQ, so if pin is low, there is a packet */
	if(GET_NRF_IRQ_IO())
	{
		/* IRQ pin is high, no packet */
		return 0;
	}
		
	len = L01_ReadRXPayload(dev->buf.buf);
	if(len > 32)
	{	

		/* packet length error */
		dev->packet_error++;
		return -1;
	}
	else 
	{
		dev->buf.pos = len;
		return len;
	}
}

/******************************************************************************
 * @brief 向2401模块发送一包数据，使其发送。
 * @note 函数中先将2401设置成为发送模式，再向2401发送数据使其发射数据
 * @author Shouxian Chen
 ******************************************************************************/
int send_nRF24L01_packet(nRF24L01 *dev)
{
	L01_CE_LOW();
	
	L01_SetTRMode(TX_MODE);
    L01_FlushTX();
	L01_WriteTXPayload_NoAck(dev->buf.buf, dev->buf.pos); 
	L01_CE_HIGH();	//CE = 1, start send 

	wait_us(20);
	L01_CE_LOW();

	clr_receive_flag(); //set wait tx flag, this flag will clean after 3ms
	
	return dev->buf.pos;
}

/******************************************************************************
 * @brief nrf24L01模块初始化
 * @author Shouxian Chen
 ******************************************************************************/
void nrf24l01_init(void)
{
	u8 addr[5] = {NRF24L01_ADDR1_0, NRF24L01_ADDR1_1, NRF24L01_ADDR1_2, NRF24L01_ADDR1_3,
	NRF24L01_ADDR1_4};
	u8 i;

	receive_flag = -1;

    L01_CE_LOW( );

	L01_ClearIRQ( IRQ_ALL );

	/* power up, crc 2-bit, mask MAX_RT_IRQ & TX_IRQ */
    //L01_WriteSingleReg( L01REG_CONFIG, 0x0e);
    L01_WriteSingleReg( L01REG_CONFIG, 0x3f);
	wait_100us(50);	

    /* Fixed packet length */
    L01_WriteSingleReg( L01REG_RX_PW_P0, PACKET_LEN );

	/* disable auto ack */							 
    L01_WriteSingleReg( L01REG_EN_AA, 0);  

	/* Enable pipe 0 receive */
    L01_WriteSingleReg( L01REG_EN_RXADDR, (1 << ERX_P0) );

	/* Address width : 5Byte */
    L01_WriteSingleReg( L01REG_SETUP_AW, AW_5BYTES );     

	/* no re-tx */
    L01_WriteSingleReg( L01REG_RETR, 0); 

	/* set rf freq */
    L01_WriteSingleReg( L01REG_RF_CH, g_robot.frq & 0x7f );        //Initial channel

	/* set rf 250kps, 0db Power*/
    L01_WriteSingleReg( L01REG_RF_SETUP, 0x26 );

	/* set tx and rx address */
	L01_SetRXAddr( 0, &addr[0], 5 ); 	//Set RX address

	for(i = 0; i < 5; i++)
	{
		addr[i] = addr[i] + 0x11;
	}
    L01_SetTXAddr( &addr[0], 5 ); 		//Set TX address
    
}

/******************************************************************************
 * @brief start nRF24L01 to RX mode
 * @author Shouxian Chen
 ******************************************************************************/
void start_nRF24L01_RX(void)
{
	if(receive_flag == 1) return ;
	
	L01_CE_LOW();
	
	L01_SetTRMode(RX_MODE);
	L01_FlushRX(); 
    L01_FlushTX();
    L01_ClearIRQ(IRQ_ALL);
	
	L01_CE_HIGH();	//CE = 1, start RX
	
	receive_flag = 1;
}

/******************************************************************************
 * @brief 将receive_flag置为1
 * @author Shouxian Chen
 ******************************************************************************/
void set_receive_flag()
{
	receive_flag = 1;
}