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
static dribbler_status_t dribbler_status;
volatile dribbler_diag_t g_dribbler_diag;

/* Caller holds the task critical section. FG is telemetry only. */
static void dribbler_publish_diag(void)
{
    g_dribbler_diag.command = dribbler_command;
    g_dribbler_diag.target_rpm = dribbler_status.target_rpm;
    g_dribbler_diag.rpm = dribbler_status.rpm;
    g_dribbler_diag.duty = dribbler_status.duty;
    g_dribbler_diag.stalled = dribbler_status.stalled;
    g_dribbler_diag.disabled = board_actuators_disabled() != 0;
    g_dribbler_diag.fg_edges = fg_count;
    g_dribbler_diag.pwm_ccr = __HAL_TIM_GET_COMPARE(&htim11, TIM_CHANNEL_1);
    g_dribbler_diag.pwm_arr = __HAL_TIM_GET_AUTORELOAD(&htim11);
    g_dribbler_diag.pwm_cr1 = htim11.Instance->CR1;
    g_dribbler_diag.pwm_ccer = htim11.Instance->CCER;
}

void dribbler_fg_edge(void)
{
    fg_count++;
}

/* Ported from SRC-tsak do_dribbler(): motor-type gain and direction.
 * Caller holds the task critical section.
 */
static void dribbler_apply_command(void)
{
    uint8_t dir;
    float duty = dribbler_command ? DRIBBLER_FIXED_DUTY : 0.0f;
    if(board_actuators_disabled())
    {
        dribbler_command = 0;
        duty = 0.0f;
    }
    if(duty > DRIBBLER_MAX_DUTY) duty = DRIBBLER_MAX_DUTY;
    if(duty < 0.0f) duty = 0.0f;
    /* The reference treats zero like a positive command for DIR selection. */
#if (DIRB_MOTOR_VERSION == OLD_DRIBBLE_MOTOR)
    dir = dribbler_command < 0;
#elif (DIRB_MOTOR_VERSION == NEW_DRIBBLE_MOTOR)
    dir = dribbler_command >= 0;
#else
#error "Invalid dribbler motor type"
#endif
    dribbler_status.target_rpm = 0.0f;
    dribbler_status.stalled = 0;
    dribbler_status.duty = duty;
    g_robot.dribbler = (uint16_t)(duty * (__HAL_TIM_GET_AUTORELOAD(&htim11) + 1U));
    set_dribbler(g_robot.dribbler, dir);
    dribbler_publish_diag();
    if(g_dribbler_diag.pwm_ccr)
    {
        g_dribbler_diag.nonzero_outputs++;
        g_dribbler_diag.last_nonzero_ms = HAL_GetTick();
        g_dribbler_diag.last_nonzero_ccr = g_dribbler_diag.pwm_ccr;
        g_dribbler_diag.last_nonzero_cr1 = g_dribbler_diag.pwm_cr1;
        g_dribbler_diag.last_nonzero_ccer = g_dribbler_diag.pwm_ccer;
    }
}

/* Nonzero legal commands select fixed duty; zero/invalid commands stop. */
void dribbler_set_target(int command)
{
    if(command < -3 || command > 3) command = 0;
    taskENTER_CRITICAL();
    g_dribbler_diag.requests++;
    g_dribbler_diag.last_request_ms = HAL_GetTick();
    if(command) g_dribbler_diag.run_requests++;
    if(command && board_actuators_disabled())
        g_dribbler_diag.rejected_requests++;
    dribbler_command = (command > 0) - (command < 0);
    dribbler_apply_command();
    taskEXIT_CRITICAL();
}

/* FG1 has three rising edges per electrical revolution. RPM remains available
 * for diagnostics, but never changes PWM and never triggers a no-FG stop.
 */
void dribbler_update(void)
{
    static uint32_t last_count, last_tick;
    uint32_t now = HAL_GetTick();
    uint32_t elapsed = now - last_tick;
    uint32_t count, edges;
    taskENTER_CRITICAL();
    if(board_actuators_disabled()) dribbler_apply_command();
    if(elapsed >= DRIBBLER_SAMPLE_MS)
    {
        last_tick = now;
        count = fg_count;
        edges = count - last_count;
        last_count = count;
        dribbler_status.rpm = (float)edges * 60000.0f /
            (3.0f * DRIBBLER_POLE_PAIRS * elapsed);
        g_dribbler_diag.updates++;
        dribbler_publish_diag();
    }
    taskEXIT_CRITICAL();
}

void dribbler_get(dribbler_status_t *status)
{
    taskENTER_CRITICAL();
    *status = dribbler_status;
    taskEXIT_CRITICAL();
}
