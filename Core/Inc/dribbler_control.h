/* coding: utf-8 */
#ifndef __DRIBBLER_CONTROL_H__
#define __DRIBBLER_CONTROL_H__

#include "stdint.h"

typedef struct
{
	float rpm;
	float target_rpm;
	float duty;
	uint8_t stalled;
} dribbler_status_t;

void dribbler_fg_edge(void);
void dribbler_set_target(int command);
void dribbler_update(void);
void dribbler_get(dribbler_status_t *status);

#endif
