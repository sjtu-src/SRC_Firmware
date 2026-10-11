#ifndef ATTITUDE_PROTOCOL_H
#define ATTITUDE_PROTOCOL_H

#include <stdint.h>

/* AA 55 10, 24 bytes. Byte 23 remains CRC8 over bytes 0..22.
 * [15..16] roll, [17..18] pitch, [19..20] relative yaw: signed LE, 0.01 deg.
 * [21] version 0xA0 | flags (valid=1, calibrating=2, read error=4, accel rejected=8).
 * [22] fresh IMU sample counter, modulo 256. Legacy firmware writes zeros.
 */
#define ATTITUDE_VERSION 0xA0U
#define ATTITUDE_OFFSET 15U

/* Local calibration diagnostics for Keil Watch. Byte 4 on the wire remains
 * infrared only (0 or 1); intermediate receiver compatibility is unverified.
 * Author: COEX, Xuanting Liu
 */
#define IMU_CAL_NONE 0U
#define IMU_CAL_ACCEL 1U
#define IMU_CAL_GYRO 2U
#define IMU_CAL_VARIANCE 3U
#define IMU_CAL_GAP 4U
#define IMU_CAL_IO 5U


static inline void attitude_encode_angle(uint8_t *out, float degrees)
{
    int32_t value;
    if (degrees > 180.0f) degrees = 180.0f;
    if (degrees < -180.0f) degrees = -180.0f;
    value = (int32_t)(degrees * 100.0f + (degrees < 0.0f ? -0.5f : 0.5f));
    out[0] = (uint8_t)((uint32_t)value & 0xFFU);
    out[1] = (uint8_t)(((uint32_t)value >> 8) & 0xFFU);
}

static inline int32_t attitude_decode_angle(const uint8_t *in)
{
    uint32_t raw = (uint32_t)in[0] | ((uint32_t)in[1] << 8);
    return raw >= 32768U ? (int32_t)raw - 65536 : (int32_t)raw;
}

#endif
