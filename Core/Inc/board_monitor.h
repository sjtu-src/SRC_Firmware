/* coding: utf-8 */
#ifndef __BOARD_MONITOR_H__
#define __BOARD_MONITOR_H__

#include "stdint.h"

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
} board_status_t;

void board_monitor_init(void);
void board_monitor_update(void);
void board_monitor_get(board_status_t *status);
void board_monitor_exti(uint16_t pin);
int board_is_shutdown(void);
int board_kicker_ready(void);

#endif
