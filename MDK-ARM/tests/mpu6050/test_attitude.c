#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "imu_attitude.h"
#include "attitude_protocol.h"

static const float level[3] = {0, 0, 1};
static const float zero[3] = {0, 0, 0};
static void calibrate(imu_filter_t *f, const float a[3], const float g[3], uint32_t start)
{
    unsigned int i;
    ImuFilter_Reset(f);
    for (i=0; i<=210; i++) ImuFilter_Update(f, a, g, 25.0f, start+i*10);
    assert(f->attitude.flags & IMU_VALID);
}
int main(void)
{
    imu_filter_t f;
    float bias[3]={.4f,-.2f,.6f}, turn[3]={.4f,-.2f,90.6f};
    float tilted[3]={0,.5f,.8660254f}, tilted_y[3]={-.5f,0,.8660254f};
    float moving[3]={0,0,10}, accel[3]={2,0,1};
    unsigned int i;
    uint8_t bytes[2];
    calibrate(&f, level, bias, 0);
    assert(fabsf(f.attitude.gyro_bias_dps[2]-.6f)<.001f);
    assert(fabsf(f.attitude.gyro_corrected_dps[2])<.001f);
    for(i=211;i<=1210;i++) ImuFilter_Update(&f,level,bias,25,i*10);
    assert(fabsf(f.attitude.yaw_deg)<.01f);
    for(i=1211;i<=1310;i++) ImuFilter_Update(&f,level,turn,25,i*10);
    assert(fabsf(f.attitude.gyro_corrected_dps[2]-90)<.001f);
    assert(fabsf(f.attitude.yaw_deg-90)<.1f);
    for(i=1311;i<=1510;i++) ImuFilter_Update(&f,level,turn,25,i*10);
    assert(fabsf(f.attitude.yaw_deg+90)<.1f);
    calibrate(&f,tilted,zero,0);
    assert(fabsf(f.attitude.roll_deg-30)<.01f && fabsf(f.attitude.pitch_deg)<.01f);
    calibrate(&f,tilted_y,zero,0);
    assert(fabsf(f.attitude.pitch_deg-30)<.01f);
    /* Gravity correction converges to a new tilt without inventing yaw. */
    for(i=211;i<=1210;i++) ImuFilter_Update(&f,level,zero,25,i*10);
    assert(fabsf(f.attitude.pitch_deg)<.01f);
    ImuFilter_Update(&f,accel,zero,25,12110);
    assert(f.attitude.flags & IMU_ACCEL_REJECTED);
    ImuFilter_Update(&f,level,zero,25,13000);
    assert(f.attitude.flags == IMU_CALIBRATING);
    ImuFilter_ReadError(&f);
    assert(f.attitude.flags == IMU_READ_ERROR);
    ImuFilter_Reset(&f);
    for(i=0;i<400;i++) ImuFilter_Update(&f,level,moving,25,i*10);
    assert(!f.calibrated);
    /* Millisecond wrap must not invalidate an otherwise fresh sample. */
    calibrate(&f,level,zero,0xfffff800U);
    assert(f.calibrated);
    ImuFilter_Update(&f,level,zero,25,0xfffff800U+2110U);
    assert(f.attitude.flags & IMU_VALID);
    attitude_encode_angle(bytes,-179.99f);
    assert(bytes[0]==0xb1 && bytes[1]==0xb9 && attitude_decode_angle(bytes)==-17999);
    attitude_encode_angle(bytes,90);
    assert(bytes[0]==0x28 && bytes[1]==0x23 && attitude_decode_angle(bytes)==9000);
    puts("PASS: calibration, bias, yaw rotation/wrap, tilt, acceleration rejection, stale/error, tick wrap, LE angles");
    return 0;
}
