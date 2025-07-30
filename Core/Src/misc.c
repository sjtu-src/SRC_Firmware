/* coding: utf-8 */
#include "misc.h"
#include "gpio.h"
#include "cfg.h"
#include "robot.h"
#include "math.h"
#include "main.h"
#include "tim.h"
#include "spi.h"

/*******************************************************************************
* @brief 初始化吸球电机
* @author Xuanting Liu
*******************************************************************************/
void init_dribbler(void)
{
	HAL_GPIO_WritePin(MOTOR_BRK5_GPIO_Port, MOTOR_BRK5_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOTOR_DIR5_GPIO_Port, MOTOR_DIR5_Pin, GPIO_PIN_RESET);
	__HAL_TIM_ENABLE(&htim11);
}

/*******************************************************************************
* @brief 读取拨码开关
* @param freq 通信频率
* @param num 车号
* @param mode 模式
* @author Xuanting Liu
*******************************************************************************/
void read_dip_sw(u8 *freq, u8 *num, u8 *mode)
{
	u16 dat = 0;
	u8 i;
	
	DIS_INT();

	/* load dip sw bit */
	DIP_SW_LD_LOW();
	wait_us(5);
	DIP_SW_CLK_LOW();
	DIP_SW_LD_HI();

	/* load data */
	for(i = 0; i < 16; i++)
	{
		dat = dat >> 1;
		if(GET_DIP_SW_D_IO()) dat = dat | 0x8000;
		
		DIP_SW_CLK_HI();
		wait_us(1);
		DIP_SW_CLK_LOW();		
		wait_us(1);
	}

	EN_INT();
	
	/* calualte the num and freq from dip set */
	/* BECARE: the dip sw bit 4 is num and freq point LSB, and bit 1 is MSB */
	*mode = (u8)((dat >> 13) & 0x7);
	*freq = (u8)((dat >> 4) & 0xf);
	*num = (u8)((dat >> 0) & 0xf) + 1; //??????????????1

	/* change rf frq channel to 24l01 freq */
	switch(*freq)
	{
		case 0:
			*freq = 95;
			break;
		case 1:
			*freq = 73;
			break;
		case 2:
			*freq = 75;
			break;
		case 3:
			*freq = 77;
			break;
		case 4:
			*freq = 79;
			break;
		case 5:
			*freq = 81;
			break;
		case 6:
			*freq = 85;
			break;
		case 7:
			*freq = 89;
			break;
		case 8:
			*freq = 101;
			break;
		case 9:
			*freq = 103;
			break;
		case 10:
			*freq = 93;
			break;
		case 11:
			*freq = 105;
			break;
		case 12:
			*freq = 109;
			break;
		case 13:
			*freq = 113;
			break;
		case 14:
			*freq = 117;
			break;
		case 15:
			*freq = 121;
			break;
		default:
			*freq = 93;
			break;
	}
}

/*******************************************************************************
* @brief 开启红外PWM
* @author Xuanting Liu
*******************************************************************************/
void start_ir_pwm(void)
{
	__HAL_TIM_ENABLE(&htim10);
}

/*******************************************************************************
* @brief nrf24l01 spi read and write one byte.
* @return 16bit adc/16
*******************************************************************************/
u8 nrf_spi_wr_rd(uint8_t input) 
{
    uint8_t tx_data = input;
    uint8_t rx_data;
    
    HAL_SPI_TransmitReceive(&HAL_NRF24L01_SPI, &tx_data, &rx_data, 1, HAL_MAX_DELAY);
    return rx_data;
}

/*******************************************************************************
* @brief shoot功能关闭
* @author Xuanting Liu
*******************************************************************************/
void shoot_off(void)
{
	HAL_TIM_PWM_Stop(&htim9, TIM_CHANNEL_1);
	__HAL_TIM_DISABLE(&htim9);
}

/*******************************************************************************
* @brief chip功能关闭
* @author Xuanting Liu
*******************************************************************************/
void chip_off(void)
{
	HAL_TIM_PWM_Stop(&htim9, TIM_CHANNEL_2);
	__HAL_TIM_DISABLE(&htim9);
}

/*******************************************************************************
* @brief motor encoder speed to RPM 速度单位转换将输入的码盘速度[count/s]转换为国际速度单位[rpm]
* @param speed 格/s  count/s
* @return rpm 转/分钟 电机最高3000rpm
*******************************************************************************/
long N2RPM(long speed )
{
	return (speed * 60) / ( 4 * ENCODER_COUNTS_PER_TURN_SET );
}
