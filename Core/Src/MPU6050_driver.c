/* coding: utf-8 */
#include "MPU6050_driver.h"
#include "i2c.h"
#include "cmsis_os.h"

/* AD0 is grounded on SRC_CtrlBoard. HAL expects the shifted address. */
#define MPU6050_I2C_TIMEOUT_MS  20U
#define MPU6050_PWR_MGMT_2      0x6CU

volatile MPU6050_InitStatus_t g_mpu6050_status = MPU6050_NOT_TESTED;

/*******************************************************************************
* @brief Read one register using the existing I2C2 peripheral.
*******************************************************************************/
static HAL_StatusTypeDef MPU6050_ReadByte(uint8_t reg, uint8_t *value)
{
    return HAL_I2C_Mem_Read(&hi2c2, SlaveAddress, reg,
                            I2C_MEMADD_SIZE_8BIT, value, 1U,
                            MPU6050_I2C_TIMEOUT_MS);
}

/*******************************************************************************
* @brief Write one register with a finite timeout, including absent-device cases.
*******************************************************************************/
static HAL_StatusTypeDef MPU6050_WriteByte(uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(&hi2c2, SlaveAddress, reg,
                             I2C_MEMADD_SIZE_8BIT, &value, 1U,
                             MPU6050_I2C_TIMEOUT_MS);
}

/*******************************************************************************
* @brief Identify, reset, wake and verify the MPU6050 in task context.
* @note Call once during startup, before other tasks access I2C2.
*       No calibration, motion sampling or heading correction is enabled here.
* @return 1 on verified initialization; 0 on communication or register failure.
*******************************************************************************/
int InitMPU6050(void)
{
    static const uint8_t settings[][2] = {
        {PWR_MGMT_1, MPU6050_CLOCK_PLL_XGYRO},
        {MPU6050_PWR_MGMT_2, 0x00U},
        {SMPLRT_DIV, 0x09U},
        {CONFIG, MPU6050_EXT_SYNC_DISABLED | MPU6050_DLPF_BW_20},
        {GYRO_CONFIG, MPU6050_GYRO_FS_500},
        {ACCEL_CONFIG, MPU6050_ACCEL_FS_4},
        {INT_ENABLE, 0x00U}
    };
    uint8_t value = 0U;
    uint32_t i;

    g_mpu6050_status = MPU6050_BUS_ERROR;
    if (MPU6050_ReadByte(WHO_AM_I, &value) != HAL_OK)
        return 0;
    if (value != MPU6050_NAME)
    {
        g_mpu6050_status = MPU6050_ID_ERROR;
        return 0;
    }

    if (MPU6050_WriteByte(PWR_MGMT_1, 0x80U) != HAL_OK)
        return 0;
    osDelay(100U);

    for (i = 0U; i < sizeof(settings) / sizeof(settings[0]); i++)
    {
        if (MPU6050_WriteByte(settings[i][0], settings[i][1]) != HAL_OK)
            return 0;
    }
    /* Allow the gyro and PLL to settle before checking the active configuration. */
    osDelay(100U);
    for (i = 0U; i < sizeof(settings) / sizeof(settings[0]); i++)
    {
        if (MPU6050_ReadByte(settings[i][0], &value) != HAL_OK)
            return 0;
        if (value != settings[i][1])
        {
            g_mpu6050_status = MPU6050_CONFIG_ERROR;
            return 0;
        }
    }
    g_mpu6050_status = MPU6050_INIT_OK;
    return 1;
}

/*******************************************************************************
* @brief Read one coherent acceleration/temperature/gyro sample from I2C2.
* @note MPU6050 uses big-endian sensor registers; output is g, deg/s and Celsius.
*******************************************************************************/
int MPU6050_ReadMotion(float accel[3], float gyro[3], float *temperature)
{
    uint8_t ready = 0U, data[14];
    HAL_StatusTypeDef result;
    unsigned int i;
    if (!I2C2_Lock(10U)) return 0;
    result = MPU6050_ReadByte(0x3AU, &ready);
    if (result == HAL_OK && (ready & 1U) != 0U)
        result = HAL_I2C_Mem_Read(&hi2c2, SlaveAddress, ACCEL_XOUT_H,
                                  I2C_MEMADD_SIZE_8BIT, data, sizeof(data),
                                  MPU6050_I2C_TIMEOUT_MS);
    I2C2_Unlock();
    if (result != HAL_OK) return -1;
    if ((ready & 1U) == 0U) return 0;
    for (i = 0U; i < 3U; i++)
    {
        int16_t raw_accel = (int16_t)(((uint16_t)data[2U*i] << 8) | data[2U*i+1U]);
        int16_t raw_gyro = (int16_t)(((uint16_t)data[8U+2U*i] << 8) | data[9U+2U*i]);
        accel[i] = (float)raw_accel / 8192.0f;
        gyro[i] = (float)raw_gyro / 65.5f;
    }
    *temperature = (float)(int16_t)(((uint16_t)data[6] << 8) | data[7]) / 340.0f + 36.53f;
    return 1;
}
