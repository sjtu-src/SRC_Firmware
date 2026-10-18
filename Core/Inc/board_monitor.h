/* coding: utf-8 */
#ifndef __BOARD_MONITOR_H__
#define __BOARD_MONITOR_H__

#include "stdint.h"

/* Watch/OLED states; ordinary actuator commands stay blocked in all shutdown states.
 * Author: COEX, Xuanting Liu
 */
enum
{
	BOARD_SHUTDOWN_NONE,
	BOARD_SHUTDOWN_DISCHARGING,
	BOARD_SHUTDOWN_FAILED,
	BOARD_SHUTDOWN_KILL
};
enum
{
	BOARD_DISCHARGE_OK,
	BOARD_DISCHARGE_COILS,
	BOARD_DISCHARGE_SENSOR,
	BOARD_DISCHARGE_TIMEOUT,
	BOARD_DISCHARGE_OUTPUT
};

typedef struct
{
	float cap_v;
	float temperature;
	float bat_v;
	float shunt_mv;
	float current_a;
	uint32_t kicker_tick;
	uint32_t battery_tick;
	uint8_t kicker_valid;
	uint8_t battery_valid;
	uint8_t current_valid;
	uint8_t charge_enabled;
	uint8_t kick_done;
	uint8_t charge_fault;
	uint8_t ina226_ready;
	uint8_t shutdown;
	uint8_t quiet;
	uint8_t shutdown_state;
	uint8_t discharge_fault;
	uint8_t discharge_failed_coils;
	uint32_t discharge_pulses;
} board_status_t;

/* Read-only debugger evidence. First failure is retained until reset;
 * current validity fields continue updating. This is not a recovery mechanism.
 */
typedef struct
{
    uint32_t first_failure_ms;
    uint32_t first_error;
    uint32_t first_hal_status;
    uint32_t failures;
    uint32_t battery_age_ms;
    float bat_v;
    uint16_t manufacturer_id;
    uint16_t device_id;
    uint8_t first_register;
    uint8_t first_write;
    uint8_t scl_high;
    uint8_t sda_high;
    uint8_t ina226_ready;
    uint8_t battery_valid;
    uint8_t quiet;
} board_battery_diag_t;
extern volatile board_battery_diag_t g_battery_diag;

void board_monitor_init(void);
void board_monitor_update(void);
void board_monitor_get(board_status_t *status);
void board_monitor_exti(uint16_t pin);
int board_is_shutdown(void);
int board_actuators_disabled(void);
int board_is_discharging(void);
int board_kicker_ready(void);

#endif
