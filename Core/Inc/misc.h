#ifndef __MISC_H__
#define __MISC_H__

#include "typedef.h"

void init_dribbler(void);
void read_dip_sw(u8 *freq, u8 *num, u8 *mode);
void start_ir_pwm(void);
u8 nrf_spi_wr_rd(u8 input);
void chip_off(void);
void shoot_off(void);
long N2RPM(long speed);
void set_heart_led(u8 r, u8 g, u8 b, u8 bright);
void heart_led_spi_wr(u8 val);
void heart_beat(void);
void shoot_on(u32 value);
void chip_on(u32 value);
u8 get_cap_v(void);
int is_power_low(void);
u8 get_bat_v(void);
long V2N(float vel);
float N2V(long n);
void Communication_Success(void);

#endif
 
