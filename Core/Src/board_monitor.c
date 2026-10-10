/* coding: utf-8 */
#include "board_monitor.h"
#include "main.h"
#include "spi.h"
#include "i2c.h"
#include "tim.h"
#include "cfg.h"
#include "gpio.h"
#include "misc.h"
#include "motor.h"
#include "action.h"
#include "robot.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

#define KICKER_SAMPLES 16U
#define BOARD_IO_TIMEOUT 3U
#define BOARD_DATA_TIMEOUT 100U
#define INA226_ADDRESS (0x40U << 1)

static board_status_t board_status;
static volatile uint8_t shutdown_requested;
static volatile uint8_t battery_alert;
static uint8_t kicker_tx[KICKER_SAMPLES * 4U];
static uint8_t kicker_rx[KICKER_SAMPLES * 4U];

/*******************************************************************************
* @brief Read one INA226 register, MSB first; I2C3 belongs to the monitor task.
* @author COEX, Xuanting Liu
*******************************************************************************/
static int ina226_read(uint8_t reg, uint16_t *value)
{
	uint8_t data[2];
	if(HAL_I2C_Mem_Read(&hi2c3, INA226_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT,
		data, sizeof(data), BOARD_IO_TIMEOUT) != HAL_OK) return 0;
	*value = ((uint16_t)data[0] << 8) | data[1];
	return 1;
}

/*******************************************************************************
* @brief Write one INA226 register without changing the CubeMX bus settings.
* @author COEX, Xuanting Liu
*******************************************************************************/
static int ina226_write(uint8_t reg, uint16_t value)
{
	uint8_t data[2];
	data[0] = (uint8_t)(value >> 8);
	data[1] = (uint8_t)value;
	return HAL_I2C_Mem_Write(&hi2c3, INA226_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT,
		data, sizeof(data), BOARD_IO_TIMEOUT) == HAL_OK;
}

/*******************************************************************************
* @brief Configure continuous bus/shunt conversions, as in TIGERs pwr_ina226.
* @note ALERT is active-low conversion-ready; reading register 6 acknowledges it.
* @author COEX, Xuanting Liu
*******************************************************************************/
static int ina226_init(void)
{
	uint16_t manufacturer, device;
	if(!ina226_read(0xfe, &manufacturer) || manufacturer != 0x5449U) return 0;
	if(!ina226_read(0xff, &device) || (device & 0xfff0U) != 0x2260U) return 0;
	return ina226_write(0x00, 0x42dfU) && ina226_write(0x06, 0x0400U);
}

/*******************************************************************************
* @brief Read ADC122S051 channels; discard two settling samples after each switch.
* @author COEX, Xuanting Liu
*******************************************************************************/
static int kicker_read(board_status_t *status)
{
	uint32_t sum[2] = {0, 0};
	uint32_t count[2] = {0, 0};
	uint32_t channel, i;
	HAL_StatusTypeDef result;
	HAL_GPIO_WritePin(KICK_CS_GPIO_Port, KICK_CS_Pin, GPIO_PIN_RESET);
	result = HAL_SPI_TransmitReceive(&hspi2, kicker_tx, kicker_rx,
		sizeof(kicker_tx), BOARD_IO_TIMEOUT);
	HAL_GPIO_WritePin(KICK_CS_GPIO_Port, KICK_CS_Pin, GPIO_PIN_SET);
	if(result != HAL_OK) return 0;
	for(channel = 0; channel < 2; channel++)
	{
		for(i = 2; i < KICKER_SAMPLES; i++)
		{
			uint32_t offset = (channel * KICKER_SAMPLES + i) * 2;
			uint16_t sample = ((uint16_t)kicker_rx[offset] << 8) | kicker_rx[offset + 1];
			if(sample < 4096U)
			{
				sum[channel] += sample;
				count[channel]++;
			}
		}
	}
	if(count[0] < 4 || count[1] < 4) return 0;
	status->temperature = (6600.0f * sum[0] / (count[0] * 4096.0f) - 800.0f) / 39.0f;
	status->cap_v = KICKER_CAP_FULL_SCALE_V * sum[1] / (count[1] * 4096.0f);
	/* An all-zero SPI reply is not a usable temperature measurement. */
	if(sum[0] == 0 || status->temperature < -20.0f || status->temperature > 150.0f) return 0;
	status->kicker_tick = HAL_GetTick();
	return 1;
}

/*******************************************************************************
* @brief Initialise outputs only; bus transactions start after the scheduler.
* @author COEX, Xuanting Liu
*******************************************************************************/
void board_monitor_init(void)
{
	uint32_t i;
	memset(&board_status, 0, sizeof(board_status));
	memset(kicker_tx, 0, sizeof(kicker_tx));
	for(i = KICKER_SAMPLES; i < 2 * KICKER_SAMPLES; i++) kicker_tx[i * 2] = 1U << 3;
	HAL_GPIO_WritePin(CHG_EN_GPIO_Port, CHG_EN_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(KICK_CS_GPIO_Port, KICK_CS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(KILL_GPIO_Port, KILL_Pin, GPIO_PIN_SET);
}

/*******************************************************************************
* @brief Latch shutdown before stopping loads so other tasks cannot restart them.
* @author COEX, Xuanting Liu
*******************************************************************************/
static void board_shutdown(void)
{
	taskENTER_CRITICAL();
	shutdown_requested = 1;
	HAL_GPIO_WritePin(CHG_EN_GPIO_Port, CHG_EN_Pin, GPIO_PIN_RESET);
	init_motor();
	do_dribbler(0);
	shooter_off();
	BEEP_OFF();
	taskEXIT_CRITICAL();
}

/*******************************************************************************
* @brief Service PDN even during startup; debounce for 50 ms and pulse KILL low.
* @author COEX, Xuanting Liu
*******************************************************************************/
static void update_shutdown(void)
{
	static uint8_t pdn_low;
	static uint32_t pdn_tick, kill_tick;
	uint32_t now = HAL_GetTick();
	if(!shutdown_requested)
	{
		if(HAL_GPIO_ReadPin(PDN_GPIO_Port, PDN_Pin) == GPIO_PIN_RESET)
		{
			if(!pdn_low)
			{
				pdn_low = 1;
				pdn_tick = now;
			}
			else if(now - pdn_tick >= 50U)
			{
				board_shutdown();
				kill_tick = now;
			}
		}
		else pdn_low = 0;
	}
	if(shutdown_requested)
	{
		/* Match TIGERs: release KILL between pulses if USB keeps the MCU alive. */
		HAL_GPIO_WritePin(KILL_GPIO_Port, KILL_Pin,
			((now - kill_tick) % 200U < 100U) ? GPIO_PIN_RESET : GPIO_PIN_SET);
	}
}

/*******************************************************************************
* @brief Publish power measurements and supervise charging at nominal 100 Hz.
* @author COEX, Xuanting Liu
*******************************************************************************/
void board_monitor_update(void)
{
	static uint32_t retry_tick;
	static uint8_t attempted;
	static uint8_t hot;
	static uint32_t charge_tick, inhibit_tick;
	static float charge_start_v;
	board_status_t status = board_status;
	uint16_t shunt, bus, mask;
	uint8_t charging;
	uint32_t now;
	update_shutdown();
	status.shutdown = shutdown_requested;
	if(status.shutdown)
	{
		status.charge_enabled = 0;
	}
	else
	{
		status.kicker_valid = kicker_read(&status);
		now = HAL_GetTick();
		if(!status.ina226_ready)
		{
			if(!attempted || now - retry_tick >= 1000U)
			{
				attempted = 1;
				retry_tick = now;
				status.ina226_ready = ina226_init();
			}
		}
		else if(battery_alert || now - status.battery_tick >= 10U)
		{
			/* Poll as a fallback too: boot with ALERT low must not lose a sample. */
			battery_alert = 0;
			if(ina226_read(0x06, &mask))
			{
				if((mask & 0x0008U) && ina226_read(0x01, &shunt) && ina226_read(0x02, &bus))
				{
					status.shunt_mv = (int16_t)shunt * 0.0025f;
					status.bat_v = bus * 0.00125f;
					status.current_valid = INA226_SHUNT_MOHM > 0.0f;
					status.current_a = status.current_valid ? status.shunt_mv / INA226_SHUNT_MOHM : 0.0f;
					status.battery_tick = HAL_GetTick();
					status.battery_valid = 1;
				}
			}
			else
			{
				status.ina226_ready = 0;
				status.battery_valid = 0;
			}
		}
		now = HAL_GetTick();
		if(now - status.battery_tick > BOARD_DATA_TIMEOUT) status.battery_valid = 0;
		if(status.temperature >= KICKER_MAX_TEMP_C) hot = 1;
		else if(status.temperature <= KICKER_MAX_TEMP_C - 5.0f) hot = 0;
		status.kick_done = HAL_GPIO_ReadPin(KICK_ERR_GPIO_Port, KICK_ERR_Pin) == GPIO_PIN_RESET;
		charging = HAL_GPIO_ReadPin(CHG_EN_GPIO_Port, CHG_EN_Pin) == GPIO_PIN_SET;
		status.charge_enabled = charging;
		if(!charging || status.cap_v >= charge_start_v + 5.0f)
		{
			charge_tick = now;
			charge_start_v = status.cap_v;
		}
		/* DONE is latched by the LT3751. Only interpret it while charging;
		 * allow a later CHARGE rising edge to clear the latch after cooldown. */
		if(charging && status.kicker_valid &&
			((status.kick_done && status.cap_v < TARGET_V_CAP - 5.0f) || now - charge_tick >= 3000U))
		{
			status.charge_fault = 1;
			inhibit_tick = now;
		}
		if(charging && status.kick_done) status.charge_enabled = 0;
		if(status.charge_fault && now - inhibit_tick >= 30000U) status.charge_fault = 0;
		if(!KICKER_CALIBRATION_CONFIRMED || !status.kicker_valid || !status.battery_valid ||
			hot || status.charge_fault || (TIM9->CR1 & TIM_CR1_CEN) ||
			status.bat_v <= FORCESTOP_POWER_A || status.cap_v >= TARGET_V_CAP)
			status.charge_enabled = 0;
		else if(status.cap_v < TARGET_V_CAP - 5.0f)
			status.charge_enabled = 1;
		/* A command task can start a kick between sampling and publishing. */
		taskENTER_CRITICAL();
		if(TIM9->CR1 & TIM_CR1_CEN) status.charge_enabled = 0;
		HAL_GPIO_WritePin(CHG_EN_GPIO_Port, CHG_EN_Pin,
			status.charge_enabled ? GPIO_PIN_SET : GPIO_PIN_RESET);
		taskEXIT_CRITICAL();
	}
	taskENTER_CRITICAL();
	board_status = status;
	g_robot.cap_v_f = status.kicker_valid ? status.cap_v : 0.0f;
	g_robot.bat_v_f = status.battery_valid ? status.bat_v : 0.0f;
	taskEXIT_CRITICAL();
}

/*******************************************************************************
* @brief Return a coherent snapshot; stale readings cannot authorise a kick.
* @author COEX, Xuanting Liu
*******************************************************************************/
void board_monitor_get(board_status_t *status)
{
	uint32_t now;
	taskENTER_CRITICAL();
	*status = board_status;
	taskEXIT_CRITICAL();
	now = HAL_GetTick();
	status->shutdown = shutdown_requested;
	if(now - status->kicker_tick > BOARD_DATA_TIMEOUT) status->kicker_valid = 0;
	if(now - status->battery_tick > BOARD_DATA_TIMEOUT) status->battery_valid = 0;
	if(!status->battery_valid) status->current_valid = 0;
}

/*******************************************************************************
* @brief Remember ALERT only; no I2C transactions in an interrupt.
* @author COEX, Xuanting Liu
*******************************************************************************/
void board_monitor_exti(uint16_t pin)
{
	if(pin == SMB_ALERT_Pin) battery_alert = 1;
	/* PDN level is debounced in the task, including a low level at boot. */
}

/*******************************************************************************
* @brief Gate all actuator command paths once power-off has been requested.
* @author COEX, Xuanting Liu
*******************************************************************************/
int board_is_shutdown(void)
{
	return shutdown_requested != 0;
}

/*******************************************************************************
* @brief Require fresh, calibrated measurements and a safe one-pulse polarity.
* @author COEX, Xuanting Liu
*******************************************************************************/
int board_kicker_ready(void)
{
	board_status_t status;
	board_monitor_get(&status);
	return !status.shutdown && KICKER_CALIBRATION_CONFIRMED &&
		status.kicker_valid && status.battery_valid && status.bat_v > FORCESTOP_POWER_A &&
		!status.charge_fault && status.temperature < KICKER_MAX_TEMP_C &&
		status.cap_v >= TARGET_V_CAP - 10.0f &&
		(TIM9->CCER & (TIM_CCER_CC1P | TIM_CCER_CC2P)) == 0;
}
