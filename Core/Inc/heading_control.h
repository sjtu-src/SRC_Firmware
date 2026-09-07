#ifndef HEADING_CONTROL_H
#define HEADING_CONTROL_H

#include <stdint.h>

typedef enum
{
    HEADING_CONTROL_DISABLED = 0,
    HEADING_CONTROL_ACTIVE,
    HEADING_CONTROL_NO_ATTITUDE,
    HEADING_CONTROL_STALE_ATTITUDE,
    HEADING_CONTROL_UNSUPPORTED_MODE
} heading_control_status_t;

typedef struct
{
    float target_yaw_deg;
    float target_yaw_rate_dps;
    float yaw_error_deg;
    float yaw_rate_error_dps;
    float yaw_integral;
    float rate_integral;
    float correction_command;
    float input_command;
    float output_command;
    uint32_t update_tick;
    uint8_t initialized;
    heading_control_status_t status;
} heading_control_t;

void HeadingControl_Reset(heading_control_t *control);
float HeadingControl_Update(heading_control_t *control, int speed_x, int speed_y,
                            float rotation_command, int pid_type,
                            float measured_yaw_deg, float measured_yaw_rate_dps,
                            uint8_t attitude_flags, uint32_t attitude_tick,
                            uint32_t now_tick);

#endif
