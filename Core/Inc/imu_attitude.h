#ifndef IMU_ATTITUDE_H
#define IMU_ATTITUDE_H

#include <stdint.h>
#include "attitude_protocol.h"

#define IMU_VALID          0x01U
#define IMU_CALIBRATING    0x02U
#define IMU_READ_ERROR     0x04U
#define IMU_ACCEL_REJECTED 0x08U

/* Right-handed sensor axes. q rotates sensor vectors into the Z-up world.
 * Euler convention: Rz(yaw) * Ry(pitch) * Rx(roll), angles in degrees.
 * Yaw is relative to completion of stationary calibration, not magnetic north.
 */
typedef struct
{
    float accel_g[3];
    float gyro_dps[3]; /* Raw MPU6050 measurement. */
    float gyro_bias_dps[3];
    float gyro_corrected_dps[3];
    float temperature_c;
    float quaternion[4]; /* w, x, y, z */
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    uint32_t sample_tick;
    uint8_t flags;
    uint8_t sequence;
    uint8_t calibration_reason; /* Last reset reason; see attitude_protocol.h. */
    uint8_t calibration_progress; /* 0..100; not attitude validity. */
} imu_attitude_t;

typedef struct
{
    imu_attitude_t attitude;
    float bias[3];
    float mean[3];
    float m2[3];
    float accel_sum[3];
    unsigned int calibration_count;
    uint32_t calibration_start;
    uint32_t previous_tick;
    uint8_t calibrated;
    uint8_t have_sample;
} imu_filter_t;

void ImuFilter_Reset(imu_filter_t *filter);
void ImuFilter_Update(imu_filter_t *filter, const float accel[3],
                      const float gyro[3], float temperature, uint32_t tick);
void ImuFilter_ReadError(imu_filter_t *filter);

#endif
