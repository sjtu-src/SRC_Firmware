/* coding: utf-8 */
#include "misc.h"
#include "gpio.h"
#include "cfg.h"
#include "robot.h"
#include "math.h"
#include "main.h"
#include "tim.h"
#include "spi.h"
#include "board_monitor.h"
#include "action.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "oled.h"

extern int test_drib_speed;
extern int test_drib_stage;

int frequency;

/*******************************************************************************
* @brief OLED显示初始画面
* @author Xuanting Liu
*******************************************************************************/
void OLED_Display_Init(void)
{
	OLED_Init();

	// 显示SRC Logo
	OLED_ShowImage(0, 4, 128, 43, SRC_LOGO);

	OLED_DrawLine(0, 51, 127, 51);

	// 显示频率
	OLED_ShowString(0, 55, "Frq.",  OLED_6X8);
    OLED_ShowNum(24, 55, g_robot.dip_frq, 1, OLED_6X8);

	// 显示车号
	OLED_ShowString(39, 55, "No.",  OLED_6X8);
    OLED_ShowNum(57, 55, g_robot.num-1, 2, OLED_6X8);

	// 显示MODE
	switch(g_robot.mode)
	{
		case NORMAL_MODE:
			OLED_ShowString(78, 55, "Normal",  OLED_6X8);
			break;
		case SELFTEST_MODE:
			OLED_ShowString(78, 55, "SelfTest",  OLED_6X8);
			break;
		case TEST_DRIBBLING_MODE:
			OLED_ShowString(78, 55, "TestDrib",  OLED_6X8);
			break;
		case CRAY_MODE:
			OLED_ShowString(78, 55, "Cray",  OLED_6X8);
			break;
		case CONTINUOUS_KICKING_MODE:
			OLED_ShowString(78, 55, "kicking",  OLED_6X8);
			break;
		default:
			OLED_ShowString(78, 55, "Invalid",  OLED_6X8);
			break;
	}

	OLED_Update();
}


/*******************************************************************************
* @brief Update shutdown text only when the state or 400 ms dots phase changes.
* @author COEX, Xuanting Liu
*******************************************************************************/
static void shutdown_display(void)
{
	static uint8_t last_state = 0xffU, last_phase = 0xffU;
	static char *dots[] = { "   ", ".  ", ".. ", "..." };
	board_status_t status;
	uint8_t phase = (HAL_GetTick() / 400U) % 4U;
	board_monitor_get(&status);
	if(status.shutdown_state == last_state &&
		(status.shutdown_state != BOARD_SHUTDOWN_DISCHARGING || phase == last_phase)) return;
	last_state = status.shutdown_state;
	last_phase = phase;
	OLED_Clear();
	if(status.shutdown_state == BOARD_SHUTDOWN_FAILED)
	{
		OLED_ShowString(0, 8, "Discharge Fail", OLED_6X8);
		OLED_ShowString(0, 26, "Press the button 3s", OLED_6X8);
		OLED_ShowString(0, 38, "to shut down", OLED_6X8);
	}
	else if(status.shutdown_state == BOARD_SHUTDOWN_KILL)
		OLED_ShowString(0, 24, "Powering off", OLED_6X8);
	else
	{
		OLED_ShowString(0, 24, "Discharging", OLED_6X8);
		OLED_ShowString(66, 24, dots[phase], OLED_6X8);
	}
	OLED_Update();
}

/*******************************************************************************
* @brief 机器人状态显示
* @author Xuanting Liu
*******************************************************************************/
void Robot_State_Display(void)
{
	int i;
	if(board_is_shutdown())
	{
		shutdown_display();
		return;
	}
	OLED_ClearArea(0, 0, 128, 51);

	switch(g_robot.mode)
	{
		case NORMAL_MODE:
		case SELFTEST_MODE:
		case CRAY_MODE:
		{
			// 显示电池电压和容电压
			OLED_Printf(0, 0, OLED_6X8, "BAT:%.1fV", g_robot.bat_v_f);
			OLED_Printf(66, 0, OLED_6X8, "CAP:%.1fV", g_robot.cap_v_f);

			// 显示各轮轮速
			for(i=0; i<4; i++)
			{
				OLED_Printf(0, 11+i*10, OLED_6X8, "wheel[%d]:%d", i, g_robot.wheels[i].cur_speed);
				//OLED_Printf(0, 11+i*10, OLED_6X8, "wheel[%d]:%d", i, g_robot.wheels[i].cur_position);
			}
			break;
		}
		case TEST_DRIBBLING_MODE:
		{
			if(test_drib_stage == 0)
				OLED_Printf(19, 28, OLED_6X8, "Get me a ball!");
			else if(test_drib_stage == 1)
				OLED_Printf(13, 28, OLED_6X8, "Rotate speed:%d", test_drib_speed);
			break;
		}
		case CONTINUOUS_KICKING_MODE:
		{
			OLED_Printf(19, 28, OLED_6X8, "Kicked: %d", g_robot.continuous_kick_cnt);
			if(g_robot.continuous_kick_mode == 0)
				OLED_Printf(13, 38, OLED_6X8, "I am kicking!");
			else
				OLED_Printf(13, 38, OLED_6X8, "I am resting!");
			break;
		}
	}

	OLED_Update();
}
 

/*******************************************************************************
* @brief 初始化吸球电机
* @author Xuanting Liu
*******************************************************************************/
void init_dribbler(void)
{
	HAL_GPIO_WritePin(MOTOR_BRK5_GPIO_Port, MOTOR_BRK5_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOTOR_DIR5_GPIO_Port, MOTOR_DIR5_Pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 0);
}

/* Match wheel startup: release BRAKEZ, load zero, start PWM, then connect AF. */
void start_dribbler(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if(!board_actuators_disabled())
    {
        init_dribbler();
        if(HAL_TIM_GenerateEvent(&htim11, TIM_EVENTSOURCE_UPDATE) != HAL_OK)
            Error_Handler();
        __HAL_TIM_CLEAR_FLAG(&htim11, TIM_FLAG_UPDATE);
        if(HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1) != HAL_OK)
            Error_Handler();
        HAL_TIM_MspPostInit(&htim11);
    }
    __set_PRIMASK(primask);
}

/*******************************************************************************
* @brief 读取拨码开关
* @param freq 通信频率
* @param num 车号
* @param mode 模式
* @author Xuanting Liu
*******************************************************************************/
void read_dip_sw(u8 *dip_freq, u8 *freq, u8 *num, u8 *mode)
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

	*dip_freq = *freq;
	frequency = *freq;

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
	HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
}

/*******************************************************************************
* @brief 初始化功率板；有效测量到达后由监控任务控制升压
* @author COEX, Xuanting Liu
*******************************************************************************/
void init_Power_Board(void)
{
	board_monitor_init();
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
* @brief Arm one high-active pulse; PWM2/HIGH returns low at the OPM rollover.
* @author COEX, Xuanting Liu
*******************************************************************************/
static int kick_pulse(uint32_t channel, uint32_t ticks)
{
	uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim9) + 1U;
	if(ticks == 0 || period < 2U) return 0;
	if(ticks >= period) ticks = period - 1U;
	HAL_TIM_PWM_Stop(&htim9, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim9, TIM_CHANNEL_2);
	HAL_GPIO_WritePin(CHG_EN_GPIO_Port, CHG_EN_Pin, GPIO_PIN_RESET);
	__HAL_TIM_SET_COMPARE(&htim9, channel, period - ticks);
	HAL_TIM_GenerateEvent(&htim9, TIM_EVENTSOURCE_UPDATE);
	__HAL_TIM_CLEAR_FLAG(&htim9, TIM_FLAG_UPDATE);
	if(HAL_TIM_PWM_Start(&htim9, channel) != HAL_OK)
	{
		shooter_off();
		return 0;
	}
	return 1;
}

/*******************************************************************************
* @brief Normal kick remains subject to battery, capacitor and shutdown interlocks.
* @author COEX, Xuanting Liu
*******************************************************************************/
static void start_kick(uint32_t channel, u32 value)
{
	if(value == 0) return;
	if(value > MAX_SHOT_STRENGTH) value = MAX_SHOT_STRENGTH;
	taskENTER_CRITICAL();
	if(board_kicker_ready() && !(TIM9->CR1 & TIM_CR1_CEN))
		kick_pulse(channel, value * 5U);
	taskEXIT_CRITICAL();
}

/*******************************************************************************
* @brief Dedicated shutdown pulse; caller requires a fresh valid capacitor measurement.
* @author COEX, Xuanting Liu
*******************************************************************************/
int discharge_on(uint8_t device, uint32_t duration_us)
{
	int result = 0;
	if(device > 1U || duration_us == 0U) return 0;
	taskENTER_CRITICAL();
	if(board_is_discharging() && KICKER_CALIBRATION_CONFIRMED &&
		!(TIM9->CR1 & TIM_CR1_CEN) &&
		(TIM9->CCER & (TIM_CCER_CC1P | TIM_CCER_CC2P)) == 0)
		result = kick_pulse(device ? TIM_CHANNEL_2 : TIM_CHANNEL_1,
			(duration_us + KICKER_TIMER_TICK_US / 2U) / KICKER_TIMER_TICK_US);
	taskEXIT_CRITICAL();
	return result;
}

/*******************************************************************************
* @brief 开启shoot平射功能
* @author COEX, Xuanting Liu
*******************************************************************************/
void shoot_on(u32 value)
{
	start_kick(TIM_CHANNEL_1, value);
}


/*******************************************************************************
* @brief 开启chip挑射功能
* @author COEX, Xuanting Liu
*******************************************************************************/
void chip_on(u32 value)
{
	start_kick(TIM_CHANNEL_2, value);
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
* @brief 将功率板电容电压编码为兼容旧无线协议的单字节数值
* @author COEX, Xuanting Liu
*******************************************************************************/
u8 get_cap_v(void)
{
	board_status_t status;
	float value;
	board_monitor_get(&status);
	if(!status.kicker_valid) return 0;
	value = status.cap_v / CAP_V_ADC_GAIN;
	if(value > 254.0f) value = 254.0f;
	return (u8)(value + 0.5f);
}

/*******************************************************************************
* @brief 检测电池电压是否过低
* @return 若低于报警电压，返回1;若低于强制停止的电压，返回2;电压正常返回0
* @author COEX, Xuanting Liu
*******************************************************************************/
int is_power_low(void)
{
	board_status_t status;
	board_monitor_get(&status);
	g_robot.bat_v = get_bat_v();
	if(!status.battery_valid || status.bat_v <= FORCESTOP_POWER_A) return 2;
	if(status.bat_v < WARNING_POWER_A) return 1;
	return 0;
}

/*******************************************************************************
* @brief 将INA226电池电压编码为兼容旧无线协议的单字节数值
* @author COEX, Xuanting Liu
*******************************************************************************/
u8 get_bat_v(void)
{
	board_status_t status;
	float value;
	board_monitor_get(&status);
	if(!status.battery_valid) return 0;
	value = status.bat_v / BAT_V_ADC_GAIN;
	if(value > 254.0f) value = 254.0f;
	return (u8)(value + 0.5f);
}

/*******************************************************************************
* @brief 将轮子的速度单位从(m/s)转换到码盘的(count/s)，这个函数存在误差，最好不要用
* @param vel 速度值 单位m/s
* @return 编码器速度 格/s  count/s 
* @author Xuanting Liu
*******************************************************************************/
long V2N(float vel)
{
	return vel * g_robot.kv2n;
}


/*******************************************************************************
* @brief 将轮子的速度单位从码盘的(count/s)转换到轮子的(m/s)，这个函数存在误差，最好不要用
* @param n 编码器速度 格/s  count/s 4倍频后的速度
* @return 速度值 单位m/s g_robot.kv2n=74037 电机最大转速3000rpm=50rps=50*1024*4=204800/74037=2.76m/s
* @author Xuanting Liu
*******************************************************************************/
float N2V(long n)
{
	return (float)n / g_robot.kv2n;
}

/*******************************************************************************
* @brief 通信成功使灯闪烁
* @author Xuanting Liu
*******************************************************************************/
void Communication_Success(void)
{
	static int comm_count = 0;

	comm_count++;

	if(comm_count < 15)
		COMM_LED_ON();
	else 
		COMM_LED_OFF();

	if(comm_count > 30)	comm_count=0;
}

/*******************************************************************************
* @brief 红外信号积分滞回滤波，抑制瞬时跳变
* @param raw_infra 原始红外状态
* @return 滤波后的稳定红外状态(0/1)
* @author Xuanting Liu
*******************************************************************************/
int Infra_Filter_Update(int raw_infra)
{
	enum
	{
		INFRA_SCORE_MAX = 150,
		INFRA_UP_STEP = 1,
		INFRA_DOWN_STEP = 20,
		INFRA_TH_ON = 80,
		INFRA_TH_OFF = 100
	};

	static int infra_score = 0;
	static int infra_state = 0;

	raw_infra = (raw_infra != 0) ? 1 : 0;

	if(raw_infra)
	{
		infra_score += INFRA_UP_STEP;
		if(infra_score > INFRA_SCORE_MAX) infra_score = INFRA_SCORE_MAX;
	}
	else
	{
		infra_score -= INFRA_DOWN_STEP;
		if(infra_score < 0) infra_score = 0;
	}

	if((infra_state == 0) && (infra_score >= INFRA_TH_ON))
	{
		infra_state = 1;
	}
	else if((infra_state == 1) && (infra_score <= INFRA_TH_OFF))
	{
		infra_state = 0;
	}

	return infra_state;
}
