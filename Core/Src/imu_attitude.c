/* coding: utf-8 */
#include "imu_attitude.h"
#include <math.h>
#include <string.h>

#define DEG_TO_RAD 0.017453292519943295f
#define RAD_TO_DEG 57.29577951308232f
#define CALIBRATION_SAMPLES 200U
#define CALIBRATION_TIME_MS 2000U

static float clamp_unit(float x)
{
    return x < -1.0f ? -1.0f : (x > 1.0f ? 1.0f : x);
}

static void reset_calibration(imu_filter_t *f)
{
    f->calibration_count = 0U;
    memset(f->mean, 0, sizeof(f->mean));
    memset(f->m2, 0, sizeof(f->m2));
    memset(f->accel_sum, 0, sizeof(f->accel_sum));
}

void ImuFilter_Reset(imu_filter_t *f)
{
    memset(f, 0, sizeof(*f));
    f->attitude.quaternion[0] = 1.0f;
    f->attitude.flags = IMU_CALIBRATING;
}

void ImuFilter_ReadError(imu_filter_t *f)
{
    /* A lost integration interval invalidates heading; require stationary re-zero. */
    uint8_t sequence = f->attitude.sequence;
    ImuFilter_Reset(f);
    f->attitude.sequence = sequence;
    f->attitude.flags = IMU_READ_ERROR;
}

static void update_euler(imu_attitude_t *a)
{
    const float w = a->quaternion[0], x = a->quaternion[1];
    const float y = a->quaternion[2], z = a->quaternion[3];
    a->roll_deg = atan2f(2.0f * (w*x + y*z), 1.0f - 2.0f * (x*x + y*y)) * RAD_TO_DEG;
    a->pitch_deg = asinf(clamp_unit(2.0f * (w*y - z*x))) * RAD_TO_DEG;
    a->yaw_deg = atan2f(2.0f * (w*z + x*y), 1.0f - 2.0f * (y*y + z*z)) * RAD_TO_DEG;
}

/*******************************************************************************
* @brief Fuse acceleration and angular velocity using a normalized quaternion.
* @note Nominal 100 Hz; integrate using measured milliseconds, including wrap.
*       Calibrate gyro bias only while stationary. Reject large linear acceleration.
*******************************************************************************/
void ImuFilter_Update(imu_filter_t *f, const float accel[3],
                      const float gyro[3], float temperature, uint32_t tick)
{
    imu_attitude_t *a = &f->attitude;
    uint32_t elapsed = tick - f->previous_tick;
    float norm = sqrtf(accel[0]*accel[0] + accel[1]*accel[1] + accel[2]*accel[2]);
    float rate[3], q[4], dt, length;
    unsigned int i;

    if (f->have_sample && elapsed == 0U) return;
    if (f->have_sample && elapsed > 100U) ImuFilter_ReadError(f);
    f->previous_tick = tick;
    f->have_sample = 1U;
    a->sample_tick = tick;
    a->sequence++;
    memcpy(a->accel_g, accel, sizeof(a->accel_g));
    memcpy(a->gyro_dps, gyro, sizeof(a->gyro_dps));
    a->temperature_c = temperature;

    if (!f->calibrated)
    {
        a->flags = IMU_CALIBRATING;
        if (norm < 0.90f || norm > 1.10f ||
            fabsf(gyro[0]) > 5.0f || fabsf(gyro[1]) > 5.0f || fabsf(gyro[2]) > 5.0f)
        {
            reset_calibration(f);
            return;
        }
        if (f->calibration_count == 0U) f->calibration_start = tick;
        f->calibration_count++;
        for (i = 0U; i < 3U; i++)
        {
            float delta = gyro[i] - f->mean[i];
            f->mean[i] += delta / (float)f->calibration_count;
            f->m2[i] += delta * (gyro[i] - f->mean[i]);
            f->accel_sum[i] += accel[i];
        }
        if (f->calibration_count < CALIBRATION_SAMPLES ||
            tick - f->calibration_start < CALIBRATION_TIME_MS) return;
        for (i = 0U; i < 3U; i++)
        {
            if (f->m2[i] / (float)(f->calibration_count - 1U) > 0.25f)
            {
                reset_calibration(f);
                return;
            }
            f->bias[i] = f->mean[i];
        }
        {
            const float *g = f->accel_sum;
            float roll = atan2f(g[1], g[2]);
            float pitch = atan2f(-g[0], sqrtf(g[1]*g[1] + g[2]*g[2]));
            float cr = cosf(roll * 0.5f), sr = sinf(roll * 0.5f);
            float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
            a->quaternion[0] = cr * cp;
            a->quaternion[1] = sr * cp;
            a->quaternion[2] = cr * sp;
            a->quaternion[3] = -sr * sp;
        }
        f->calibrated = 1U;
        a->flags = IMU_VALID;
        update_euler(a);
        return;
    }

    a->flags = IMU_VALID;
    memcpy(q, a->quaternion, sizeof(q));
    for (i = 0U; i < 3U; i++) rate[i] = (gyro[i] - f->bias[i]) * DEG_TO_RAD;
    if (norm >= 0.85f && norm <= 1.15f)
    {
        /* Measured gravity cross predicted gravity: proportional tilt correction.
         * No integral yaw correction is possible without an external reference.
         */
        float ax = accel[0]/norm, ay = accel[1]/norm, az = accel[2]/norm;
        float vx = 2.0f * (q[1]*q[3] - q[0]*q[2]);
        float vy = 2.0f * (q[0]*q[1] + q[2]*q[3]);
        float vz = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
        rate[0] += 2.0f * (ay*vz - az*vy);
        rate[1] += 2.0f * (az*vx - ax*vz);
        rate[2] += 2.0f * (ax*vy - ay*vx);
    }
    else a->flags |= IMU_ACCEL_REJECTED;

    dt = (float)elapsed * 0.0005f; /* half of the measured time step */
    a->quaternion[0] += (-q[1]*rate[0] - q[2]*rate[1] - q[3]*rate[2]) * dt;
    a->quaternion[1] += ( q[0]*rate[0] + q[2]*rate[2] - q[3]*rate[1]) * dt;
    a->quaternion[2] += ( q[0]*rate[1] - q[1]*rate[2] + q[3]*rate[0]) * dt;
    a->quaternion[3] += ( q[0]*rate[2] + q[1]*rate[1] - q[2]*rate[0]) * dt;
    length = sqrtf(a->quaternion[0]*a->quaternion[0] + a->quaternion[1]*a->quaternion[1] +
                   a->quaternion[2]*a->quaternion[2] + a->quaternion[3]*a->quaternion[3]);
    for (i = 0U; i < 4U; i++) a->quaternion[i] /= length;
    update_euler(a);
}
