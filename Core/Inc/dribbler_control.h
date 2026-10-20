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

/* Open-loop mode: target_rpm, stalled and no_fg_stops stay zero.
 * Watch this object after self-test; history survives the normal stop command.
 * Counters describe software activity, not a measurement of the PB9 waveform.
 */
typedef struct
{
    uint32_t requests, run_requests, rejected_requests, updates;
    uint32_t no_fg_stops, nonzero_outputs, fg_edges;
    uint32_t last_request_ms, last_nonzero_ms;
    int32_t command;
    float target_rpm, rpm, duty;
    uint8_t stalled, disabled;
    uint32_t pwm_ccr, pwm_arr, pwm_cr1, pwm_ccer;
    uint32_t last_nonzero_ccr, last_nonzero_cr1, last_nonzero_ccer;
} dribbler_diag_t;

extern volatile dribbler_diag_t g_dribbler_diag;

void dribbler_fg_edge(void);
void dribbler_set_target(int command);
void dribbler_update(void);
void dribbler_get(dribbler_status_t *status);

#endif
