"""Run the production MPU6050 initialization against a fault-injecting HAL stub.

Requires Python 3 and a host GCC on PATH. No board or Python packages required.
"""
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[3]

HAL = r'''
#include <stdint.h>
typedef int I2C_HandleTypeDef;
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
extern I2C_HandleTypeDef hi2c2;
int I2C2_Lock(uint32_t);
void I2C2_Unlock(void);
#define I2C_MEMADD_SIZE_8BIT 1
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *, uint16_t, uint16_t,
                                  uint16_t, uint8_t *, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *, uint16_t, uint16_t,
                                   uint16_t, uint8_t *, uint16_t, uint32_t);
'''

HARNESS = r'''
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "i2c.h"
#include "MPU6050_driver.h"
I2C_HandleTypeDef hi2c2;
static int lock_available=1, locked=0;
int I2C2_Lock(uint32_t timeout) { (void)timeout; locked=lock_available; return locked; }
void I2C2_Unlock(void) { assert(locked); locked=0; }
static uint8_t regs[256];
static int calls, fail_at, corrupt_reg, delay_ms;
static HAL_StatusTypeDef fault;

static HAL_StatusTypeDef access_bus(I2C_HandleTypeDef *bus, uint16_t addr,
    uint16_t size, uint16_t count, uint32_t timeout)
{
    assert(bus == &hi2c2 && addr == 0xD0 && size == 1 && (count == 1 || count == 14));
    assert(timeout > 0 && timeout <= 25);
    return ++calls == fail_at ? fault : HAL_OK;
}
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *bus, uint16_t addr,
    uint16_t reg, uint16_t size, uint8_t *value, uint16_t count, uint32_t timeout)
{
    HAL_StatusTypeDef result = access_bus(bus, addr, size, count, timeout);
    if (result == HAL_OK) {
        memcpy(value,regs+reg,count);
        if (reg == corrupt_reg) *value ^= 1;
    }
    return result;
}
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *bus, uint16_t addr,
    uint16_t reg, uint16_t size, uint8_t *value, uint16_t count, uint32_t timeout)
{
    HAL_StatusTypeDef result = access_bus(bus, addr, size, count, timeout);
    if (result == HAL_OK) regs[reg] = *value;
    return result;
}
void osDelay(uint32_t ms) { delay_ms += ms; }
static void setup(void)
{
    memset(regs, 0, sizeof(regs));
    regs[0x75] = 0x68;
    calls = fail_at = delay_ms = 0;
    corrupt_reg = -1;
    lock_available=1; locked=0;
}
int main(void)
{
    const int config_regs[] = {0x6B, 0x6C, 0x19, 0x1A, 0x1B, 0x1C, 0x38};
    int total, n, kind;
    setup();
    assert(InitMPU6050() == 1 && g_mpu6050_status == MPU6050_INIT_OK);
    assert(regs[0x19] == 9);
    assert(regs[0x6B] == 1 && regs[0x6C] == 0 && regs[0x38] == 0);
    assert(regs[0x1B] == 8 && regs[0x1C] == 8 && delay_ms >= 200);
    total = calls;
    /* Fail every transaction in turn, including the first probe and final read. */
    for (kind = HAL_ERROR; kind <= HAL_TIMEOUT; kind++)
        for (n = 1; n <= total; n++)
        {
            setup(); fail_at = n; fault = (HAL_StatusTypeDef)kind;
            assert(InitMPU6050() == 0 && g_mpu6050_status == MPU6050_BUS_ERROR);
            assert(calls == n);
        }
    setup(); regs[0x75] = 0x69;
    assert(InitMPU6050() == 0 && g_mpu6050_status == MPU6050_ID_ERROR);
    assert(calls == 1 && delay_ms == 0);
    for (n = 0; n < 7; n++)
    {
        setup(); corrupt_reg = config_regs[n];
        assert(InitMPU6050() == 0 && g_mpu6050_status == MPU6050_CONFIG_ERROR);
    }
    setup();
    assert(InitMPU6050() == 1 && g_mpu6050_status == MPU6050_INIT_OK);
    printf("PASS: success, %d bus fault cases, wrong ID, 7 readback faults, recovery\n",
           3 * total);
    {
        float a[3],g[3],temp;
        setup(); regs[0x3a]=1;
        regs[0x3b]=0x20; regs[0x3d]=0xf0;
        regs[0x41]=1; regs[0x42]=0x54;
        regs[0x43]=2; regs[0x44]=0x8f;
        regs[0x45]=0xfa; regs[0x46]=0xe2;
        regs[0x47]=0x7f; regs[0x48]=0xee;
        assert(MPU6050_ReadMotion(a,g,&temp)==1 && calls==2 && !locked);
        assert(fabsf(a[0]-1)<.001f && fabsf(a[1]+.5f)<.001f);
        assert(fabsf(g[0]-10)<.001f && fabsf(g[1]+20)<.001f && fabsf(g[2]-500)<.001f);
        assert(fabsf(temp-37.53f)<.001f);
        setup();
        assert(MPU6050_ReadMotion(a,g,&temp)==0 && calls==1 && !locked);
        setup(); lock_available=0;
        assert(MPU6050_ReadMotion(a,g,&temp)==0 && calls==0 && !locked);
        for(n=1;n<=2;n++) {
            setup(); regs[0x3a]=1; fail_at=n; fault=HAL_TIMEOUT;
            assert(MPU6050_ReadMotion(a,g,&temp)==-1 && calls==n && !locked);
        }
        puts("PASS: coherent burst, signed sensor units, temperature, no-data, occupied bus, timeout unlock");
    }
    return 0;
}
'''

def main():
    tmp = Path(__file__).resolve().parent / 'build'
    tmp.mkdir(exist_ok=True)
    (tmp / 'i2c.h').write_text(HAL)
    (tmp / 'cmsis_os.h').write_text('#include <stdint.h>\nvoid osDelay(uint32_t);\n')
    (tmp / 'test.c').write_text(HARNESS)
    exe = tmp / 'test.exe'
    subprocess.run(['gcc', '-std=c99', '-Wall', '-Wextra', '-Werror',
                    '-I', str(tmp), '-I', str(ROOT / 'Core/Inc'),
                    str(ROOT / 'Core/Src/MPU6050_driver.c'), str(tmp / 'test.c'),
                    '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)


if __name__ == '__main__':
    main()
