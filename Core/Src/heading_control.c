/* coding: utf-8 */
#include "heading_control.h"
#include "cfg.h"
#include "imu_attitude.h"
#include "pid.h"
#include <string.h>

#define RAD_TO_DEG 57.29577951308232f

static float clamp_float(float value, float minimum, float maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static float wrap_angle(float angle)
{
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

void HeadingControl_Reset(heading_control_t *control)
{
    memset(control, 0, sizeof(*control));
    control->status = HEADING_CONTROL_DISABLED;
}

/*******************************************************************************
* @brief Add bounded yaw tracking feedback to the operator rotation command.
* @note The operator rotation command is always retained as feedforward. The
*       target yaw follows simultaneous translation and rotation commands.
*******************************************************************************/
float HeadingControl_Update(heading_control_t *control, int speed_x, int speed_y,
                            float rotation_command, int pid_type,
                            float measured_yaw_deg, float measured_yaw_rate_dps,
                            uint8_t attitude_flags, uint32_t attitude_tick,
                            uint32_t now_tick)
{
    uint32_t elapsed_ms = now_tick - control->update_tick;
    float dt, target_rate, heading_rate, rate_error;
    float proposed_integral, correction, unsaturated_output;
    int moving = speed_x != 0 || speed_y != 0 || rotation_command != 0.0f;

    control->input_command = rotation_command;
    control->output_command = rotation_command;
    control->correction_command = 0.0f;

#if !HEADING_CONTROL_ENABLE
    (void)pid_type;
    (void)measured_yaw_deg;
    (void)measured_yaw_rate_dps;
    (void)attitude_flags;
    (void)attitude_tick;
    (void)now_tick;
    control->status = HEADING_CONTROL_DISABLED;
    return rotation_command;
#else
    if (pid_type != SPEED_PID)
    {
        HeadingControl_Reset(control);
        control->status = HEADING_CONTROL_UNSUPPORTED_MODE;
        return rotation_command;
    }
    if ((attitude_flags & IMU_VALID) == 0U)
    {
        HeadingControl_Reset(control);
        control->status = HEADING_CONTROL_NO_ATTITUDE;
        return rotation_command;
    }
    if (now_tick - attitude_tick > HEADING_ATTITUDE_TIMEOUT_MS)
    {
        HeadingControl_Reset(control);
        control->status = HEADING_CONTROL_STALE_ATTITUDE;
        return rotation_command;
    }
    measured_yaw_deg *= HEADING_SENSOR_YAW_SIGN;
    measured_yaw_rate_dps *= HEADING_SENSOR_YAW_SIGN;

    if (!moving)
    {
        HeadingControl_Reset(control);
        control->target_yaw_deg = measured_yaw_deg;
        return 0;
    }
    if (!control->initialized || elapsed_ms == 0U ||
        elapsed_ms > HEADING_UPDATE_TIMEOUT_MS)
    {
        control->target_yaw_deg = measured_yaw_deg;
        control->rate_integral = 0.0f;
        control->initialized = 1U;
        elapsed_ms = 0U;
    }
    control->update_tick = now_tick;
    control->status = HEADING_CONTROL_ACTIVE;
    dt = (float)elapsed_ms * 0.001f;

    target_rate = (float)rotation_command * HEADING_COMMAND_RAD_S * RAD_TO_DEG;
    control->target_yaw_rate_dps = target_rate;
    control->target_yaw_deg = wrap_angle(control->target_yaw_deg + target_rate * dt);
    control->yaw_error_deg = wrap_angle(control->target_yaw_deg - measured_yaw_deg);
    control->yaw_error_deg = clamp_float(control->yaw_error_deg,
                                         -HEADING_MAX_TRACKING_ERROR_DEG,
                                         HEADING_MAX_TRACKING_ERROR_DEG);
    /* Do not let an unreachable reference continue running away from the robot. */
    control->target_yaw_deg = wrap_angle(measured_yaw_deg + control->yaw_error_deg);

    /* A zero rotation command means hold the captured yaw. The gyro term reacts
     * to wheel-startup asymmetry before a large angle error develops, while the
     * yaw integral learns persistent direction-dependent chassis imbalance.
     */
    if (rotation_command == 0.0f)
    {
        control->rate_integral = 0.0f;
        control->yaw_rate_error_dps = -measured_yaw_rate_dps;
        proposed_integral = clamp_float(
            control->yaw_integral + control->yaw_error_deg * dt,
            -HEADING_YAW_INTEGRAL_LIMIT, HEADING_YAW_INTEGRAL_LIMIT);
        correction = HEADING_HOLD_KP * control->yaw_error_deg
                   + HEADING_HOLD_KI * proposed_integral
                   - HEADING_HOLD_KD * measured_yaw_rate_dps;
        correction = clamp_float(correction, -HEADING_MAX_CORRECTION_COMMAND,
                                 HEADING_MAX_CORRECTION_COMMAND);
        if (correction > -HEADING_MAX_CORRECTION_COMMAND &&
            correction < HEADING_MAX_CORRECTION_COMMAND)
        {
            control->yaw_integral = proposed_integral;
        }
        correction = HEADING_HOLD_KP * control->yaw_error_deg
                   + HEADING_HOLD_KI * control->yaw_integral
                   - HEADING_HOLD_KD * measured_yaw_rate_dps;
        control->correction_command = clamp_float(
            correction, -HEADING_MAX_CORRECTION_COMMAND,
            HEADING_MAX_CORRECTION_COMMAND);
        control->output_command = control->correction_command;
        return control->output_command;
    }

    control->yaw_integral = 0.0f;

    heading_rate = target_rate + HEADING_ANGLE_KP * control->yaw_error_deg;
    rate_error = heading_rate - measured_yaw_rate_dps;
    control->yaw_rate_error_dps = rate_error;
    proposed_integral = clamp_float(control->rate_integral + rate_error * dt,
                                    -HEADING_RATE_INTEGRAL_LIMIT,
                                    HEADING_RATE_INTEGRAL_LIMIT);
    correction = HEADING_RATE_KP * rate_error + HEADING_RATE_KI * proposed_integral;
    correction = clamp_float(correction, -HEADING_MAX_CORRECTION_COMMAND,
                             HEADING_MAX_CORRECTION_COMMAND);
    unsaturated_output = (float)rotation_command + correction;

    /* Integrate only when neither the correction nor final command pushes farther
     * into saturation. This prevents a delayed reversal after a hard command.
     */
    if (correction > -HEADING_MAX_CORRECTION_COMMAND &&
        correction < HEADING_MAX_CORRECTION_COMMAND &&
        unsaturated_output > -HEADING_MAX_ROTATION_COMMAND &&
        unsaturated_output < HEADING_MAX_ROTATION_COMMAND)
    {
        control->rate_integral = proposed_integral;
    }
    correction = HEADING_RATE_KP * rate_error +
                 HEADING_RATE_KI * control->rate_integral;
    correction = clamp_float(correction, -HEADING_MAX_CORRECTION_COMMAND,
                             HEADING_MAX_CORRECTION_COMMAND);
    control->correction_command = correction;
    control->output_command = clamp_float(rotation_command + correction,
                                          -HEADING_MAX_ROTATION_COMMAND,
                                          HEADING_MAX_ROTATION_COMMAND);
    return control->output_command;
#endif
}
