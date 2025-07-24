
#ifndef __I2C_H__
#define __I2C_H__

#include "stm32f4xx.h"
#include "typedef.h"
#include "tim.h"

/* IO simluation I2C bus, operation define */
#define SCL_H       HAL_GPIO_WritePin(SCL_GPIO_Port, SCL_Pin, GPIO_PIN_SET); //PB10

#define SCL_L       HAL_GPIO_WritePin(SCL_GPIO_Port, SCL_Pin, GPIO_PIN_RESET);
    
#define SDA_H       HAL_GPIO_WritePin(SDA_GPIO_Port, SDA_Pin, GPIO_PIN_SET);	//PB11

#define SDA_L       HAL_GPIO_WritePin(SDA_GPIO_Port, SDA_Pin, GPIO_PIN_RESET);

#define SDA_RD(val) (val = GPIOB->IDR & GPIO_PIN_11)

#define i2c_delay()		wait_us(10)

/* i2c eeprom parameter define */
#define PAGE_WR_SIZE				8
#define HIGH_SZ_PAGE_WR_SIZE		32
#define EEPROM_DEV_ADDR				0xa0

void init_i2c(void);
u8 i2c_m_send_dat(u8 dev_addr, u8 *buf, u8 cnt);
u8 i2c_m_read_dat(u8 dev_addr, u8 *buf, u8 cnt);
u8 m_probe_i2c_dev(u8 dev_addr);
void low_size_eeprom_rd(void *dat_buf, u16 addr, u32 count);
void low_size_eeprom_wr(void *dat_buf, u16 addr, u32 count);
u8 i2c_m_rd_byte(u8 dev_addr, u8 reg_addr, u8 *dat);
u8 i2c_m_wr_byte(u8 dev_addr, u8 reg_addr, u8 dat);

void high_size_eeprom_rd(void *dat_buf, u16 addr, u32 count);
void high_size_eeprom_wr(void *dat_buf, u16 addr, u32 count);



#endif
