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

/* Multiple inhibit reasons may be present simultaneously. */
enum
{
    BOARD_CHG_CAL = 1U << 0,
    BOARD_CHG_ADC = 1U << 1,
    BOARD_CHG_BAT_DATA = 1U << 2,
    BOARD_CHG_BAT_LOW = 1U << 3,
    BOARD_CHG_HOT = 1U << 4,
    BOARD_CHG_NO_BOARD = 1U << 5,
    BOARD_CHG_PROGRESS = 1U << 6,
    BOARD_CHG_KICK = 1U << 7,
    BOARD_CHG_COOLDOWN = 1U << 8,
    BOARD_CHG_QUIET = 1U << 9,
    BOARD_CHG_SHUTDOWN = 1U << 10,
    BOARD_CHG_FULL = 1U << 11
};

typedef struct
{
    uint16_t charge_inhibit;
    uint16_t charge_fault_reason;
    uint32_t charge_retry_ms;
    uint32_t charge_cooldown_ms;
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
	uint8_t kicker_installed;
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

void board_charge_gpio_init(void);
void board_monitor_init(void);
void board_monitor_update(void);
void board_monitor_get(board_status_t *status);
void board_monitor_exti(uint16_t pin);
/* Called in the kick critical section after a successful timer start. */
void board_notify_kick(void);
int board_is_shutdown(void);
int board_actuators_disabled(void);
int board_is_discharging(void);
int board_kicker_ready(void);
int board_kicker_installed(void);

#endif
