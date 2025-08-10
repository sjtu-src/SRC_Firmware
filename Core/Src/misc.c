/* coding: utf-8 */
#include "misc.h"
#include "gpio.h"
#include "cfg.h"
#include "robot.h"
#include "math.h"
#include "main.h"
#include "tim.h"
#include "spi.h"
#include "adc.h"
#include "cmsis_os.h"
 

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
* @brief 开启shoot平射功能
* @author Xuanting Liu
*******************************************************************************/
void shoot_on(u32 value)
{

	if(value > MAX_SHOT_STRENGTH) value = MAX_SHOT_STRENGTH;

	value = MAX_SHOT_STRENGTH + 3 - value;

	if(value == 0) value = 1;
	
	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim9, TIM_CHANNEL_2);
	__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, value);
	__HAL_TIM_ENABLE(&htim9);
}


/*******************************************************************************
* @brief 开启chip挑射功能
* @author Xuanting Liu
*******************************************************************************/
void chip_on(u32 value)
{
	if(value > MAX_SHOT_STRENGTH) value = MAX_SHOT_STRENGTH;

	value = MAX_SHOT_STRENGTH - value;

	if(value == 0) value = 1;

	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim9, TIM_CHANNEL_1);
	__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, value);
	__HAL_TIM_ENABLE(&htim9);
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

/*******************************************************************************
* @brief APA102七彩灯颜色和亮度设置
* @param r 红色通道
* @param g 绿色通道
* @param b 蓝色通道
* @param bright 亮度
* @author Xuanting Liu
*******************************************************************************/
void set_heart_led(u8 r, u8 g, u8 b, u8 bright)
{
	u8 val;
	
	/* APA102 start frame, 4 byte 0 */
	heart_led_spi_wr(0x0);       
    heart_led_spi_wr(0x0);
    heart_led_spi_wr(0x0);
    heart_led_spi_wr(0x0);

	val = 0xe0 | (bright & 0x1f);
	heart_led_spi_wr(val);
	heart_led_spi_wr(b);
	heart_led_spi_wr(g);
	heart_led_spi_wr(r);
	
	/* APA102 end frame, 4 byte 0xff */
	heart_led_spi_wr(0xff);       
    heart_led_spi_wr(0xff);
    heart_led_spi_wr(0xff);
    heart_led_spi_wr(0xff);
}

/*******************************************************************************
* @brief APA102七彩灯数据传输
* @author Xuanting Liu
*******************************************************************************/
void heart_led_spi_wr(u8 val)
{
	u8 i;

	for(i = 0; i < 8; i++)
	{
		RGB_LED_CLK_LOW();
		
		if(val & 0x80) RGB_LED_DAT_HI();
		else RGB_LED_DAT_LOW();

		wait_us(1);	
		RGB_LED_CLK_HI();
		wait_us(1);	

		val = val << 1;
	}

	RGB_LED_CLK_HI();
}

/*******************************************************************************
* @brief 彩色LED点亮
* @author Xuanting Liu 
* @note 正常比赛模式-白色；CRAY模式-绿色；自检模式-蓝色
*******************************************************************************/
void heart_beat(void)
{
	LED_POWER_ON_HIGH();
	switch(g_robot.mode)
	{
		case NORMAL_MODE:
			set_heart_led(100, 100, 100, 0x1f); //白色
			break;
		case CRAY_MODE:
			set_heart_led(0, 100, 0, 0x1f); //绿色
			break;
		case SELFTEST_MODE:
			set_heart_led(0, 0, 100, 0x1f); //蓝色
			break;
		default:
			LED_POWER_ON_LOW();
			break;
	}
}

/*******************************************************************************
* @brief 获取电容adc值
* @return 16bit adc/16 取12bit的高8位
* @author Xuanting Liu
*******************************************************************************/
u8 get_cap_v(void)
{
    u16 val;

    // 配置ADC通道8（PB0）
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = ADC_CHANNEL_8;  // 选择通道8
    sConfig.Rank = 1;                 // 设置为第1个通道
    sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES; // 设置采样时间
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    // 启动ADC1
    HAL_ADC_Start(&hadc1);

    // 等待ADC转换完成
    HAL_ADC_PollForConversion(&hadc1, 100);

    // 获取ADC值
    val = HAL_ADC_GetValue(&hadc1);

	// 停止ADC1
	HAL_ADC_Stop(&hadc1);

    // 将16位ADC值右移4位，取高8位
    val = val >> 4;

    return (uint8_t)(val & 0xFF);
}

/*******************************************************************************
* @brief 检测电池电压是否过低
* @return 若低于报警电压，返回1;若低于强制停止的电压，返回2;电压正常返回0
* @author Xuanting Liu
*******************************************************************************/
int is_power_low(void)
{
  	/* compare battery power AD value with low power threashold */
	static int powerarr[5] = {0};
 	static int num = 0;
	static int avepower;
	static int limpower = 5;
	static u8 is_fifo_full = 0;
	int i,u,d;
	u8 bat_v;
	int ret = 0;
  	
   	bat_v = get_bat_v(); 

	//Beep_Show_8bit(bat_v);
   	
   	powerarr[num++] = bat_v;
	if(num == 5) 
	{
		num = 0;
		is_fifo_full = 1;
	}

	u = 0;
	d = 0;
	
	if(is_fifo_full)
	{
		for(i = 1; i < 5; i++)
		{
			if(powerarr[i] > powerarr[u]) u = i;
			if(powerarr[i] < powerarr[d]) d = i;			
		}
        
		if(powerarr[u] > (avepower + limpower)) powerarr[u] = avepower;
		if(powerarr[d] < (avepower - limpower)) powerarr[d] = avepower;
        
        avepower = powerarr[0];
		
        for(i = 1; i < 5; i++)
		{
			avepower = avepower + powerarr[i];
		}

		avepower = avepower / 5;
		bat_v = (u8)avepower;

		if(avepower < (int)WARNING_POWER_D)
		{
			ret = 1;
  			if(avepower <= (int)FORCESTOP_POWER_D)
  			{
				ret = 2;
  			}
    	}
	}
	else 
	{
		avepower = (powerarr[0] + powerarr[1] + powerarr[2] + powerarr[3]) / 4;
	}

	DIS_INT();
	g_robot.bat_v = bat_v;
	g_robot.bat_v_f = bat_v  * BAT_V_ADC_GAIN;
	EN_INT();
	
    return ret;
}

/*******************************************************************************
* @brief 获取电池电压adc值
* @author Xuanting Liu
*******************************************************************************/
u8 get_bat_v(void)
{
    u16 val;

    // 配置ADC通道9（PB1）
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = ADC_CHANNEL_9;  // 选择通道9
    sConfig.Rank = 1;                 // 设置为第1个通道
    sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES; // 设置采样时间
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    // 启动ADC1
    HAL_ADC_Start(&hadc1);

    // 等待ADC转换完成
    HAL_ADC_PollForConversion(&hadc1, 100);

    // 获取ADC值
    val = HAL_ADC_GetValue(&hadc1);

	// 停止ADC1
	HAL_ADC_Stop(&hadc1);

    // 将16位ADC值右移4位，取高8位
    val = val >> 4;

    return (uint8_t)(val & 0xFF);
}

/*******************************************************************************
* @brief 将轮子的速度单位从(m/s)转换到码盘的(count/s)
* @param vel 速度值 单位m/s
* @return 编码器速度 格/s  count/s 
* @author Xuanting Liu
*******************************************************************************/
long V2N(float vel)
{
	return vel * g_robot.kv2n;
}


/*******************************************************************************
* @brief 将轮子的速度单位从码盘的(count/s)转换到轮子的(m/s)
* @param n 编码器速度 格/s  count/s 4倍频后的速度
* @return 速度值 单位m/s g_robot.kv2n=74037 电机最大转速3000rpm=50rps=50*1024*4=204800/74037=2.76m/s
* @author Xuanting Liu
*******************************************************************************/
float N2V(long n)
{
	return (float)n / g_robot.kv2n;
}