/* coding: utf-8 */
#include "dribbler_control.h"
#include "board_monitor.h"
#include "cfg.h"
#include "main.h"
#include "tim.h"
#include "action.h"
#include "robot.h"
#include "FreeRTOS.h"
#include "task.h"

static volatile uint32_t fg_count;
static int dribbler_command;
static float integral;
static uint32_t last_motion_tick;
static dribbler_status_t dribbler_status;

/*******************************************************************************
* @brief FG1 rising-edge ISR: count only, no HAL calls or floating-point work.
* @author COEX, Xuanting Liu
*******************************************************************************/
void dribbler_fg_edge(void)
{
	fg_count++;
}

/*******************************************************************************
* @brief All nonzero legal commands use the same configured speed; zero stops.
* @author COEX, Xuanting Liu
*******************************************************************************/
void dribbler_set_target(int command)
{
	int direction;
	if(command < -3 || command > 3) command = 0;
	direction = (command > 0) - (command < 0);
	taskENTER_CRITICAL();
	if(board_actuators_disabled()) direction = 0;
	if(direction != dribbler_command || direction == 0)
	{
		dribbler_command = direction;
		integral = 0.0f;
		dribbler_status.duty = 0.0f;
		dribbler_status.target_rpm = direction ? DRIBBLER_TARGET_RPM : 0.0f;
		last_motion_tick = HAL_GetTick();
		/* A missing-feedback latch is cleared only by an explicit stop. */
		if(direction == 0) dribbler_status.stalled = 0;
		g_robot.dribbler = 0;
		set_dribbler(0, direction > 0);
	}
	taskEXIT_CRITICAL();
}

/*******************************************************************************
* @brief 100 ms FG1 speed window and bounded PI speed control for MOTOR5.
* @note A4931 FG1 has three rising edges per electrical revolution.
* @author COEX, Xuanting Liu
*******************************************************************************/
void dribbler_update(void)
{
	static uint32_t last_count, last_tick;
	uint32_t now = HAL_GetTick();
	uint32_t elapsed = now - last_tick;
	uint32_t count, edges;
	float error, candidate, duty, step;
	uint8_t dir;
	if(elapsed < DRIBBLER_SAMPLE_MS) return;
	last_tick = now;
	count = fg_count;
	edges = count - last_count;
	last_count = count;
	taskENTER_CRITICAL();
	dribbler_status.rpm = (float)edges * 60000.0f /
		(3.0f * DRIBBLER_POLE_PAIRS * elapsed);
	if(edges) last_motion_tick = now;
	if(dribbler_command && now - last_motion_tick >= DRIBBLER_FG_TIMEOUT_MS)
		dribbler_status.stalled = 1;
	if(!dribbler_command || dribbler_status.stalled || board_actuators_disabled())
	{
		integral = 0.0f;
		dribbler_status.duty = 0.0f;
		g_robot.dribbler = 0;
		set_dribbler(0, dribbler_command > 0);
		taskEXIT_CRITICAL();
		return;
	}
	error = dribbler_status.target_rpm - dribbler_status.rpm;
	candidate = integral + DRIBBLER_SPEED_KI * error * elapsed * 0.001f;
	if(candidate < 0.0f) candidate = 0.0f;
	if(candidate > DRIBBLER_MAX_DUTY) candidate = DRIBBLER_MAX_DUTY;
	duty = DRIBBLER_SPEED_KP * error + candidate;
	/* Conditional integration avoids accumulating error at either output limit. */
	if((duty >= 0.0f && duty <= DRIBBLER_MAX_DUTY) ||
		(duty > DRIBBLER_MAX_DUTY && error < 0.0f) || (duty < 0.0f && error > 0.0f))
		integral = candidate;
	duty = DRIBBLER_SPEED_KP * error + integral;
	if(duty < 0.0f) duty = 0.0f;
	if(duty > DRIBBLER_MAX_DUTY) duty = DRIBBLER_MAX_DUTY;
	step = DRIBBLER_DUTY_RAMP_PER_S * elapsed * 0.001f;
	if(duty > dribbler_status.duty + step) duty = dribbler_status.duty + step;
	dribbler_status.duty = duty;
	dir = dribbler_command > 0;
	#if (DIRB_MOTOR_VERSION == OLD_DIRB_MOTOR)
		dir = !dir;
	#endif
	g_robot.dribbler = (uint16_t)(duty * (__HAL_TIM_GET_AUTORELOAD(&htim11) + 1U));
	set_dribbler(g_robot.dribbler, dir);
	taskEXIT_CRITICAL();
}

/*******************************************************************************
* @brief Return measured RPM, target RPM, output duty and missing-feedback fault.
* @author COEX, Xuanting Liu
*******************************************************************************/
void dribbler_get(dribbler_status_t *status)
{
	taskENTER_CRITICAL();
	*status = dribbler_status;
	taskEXIT_CRITICAL();
}
