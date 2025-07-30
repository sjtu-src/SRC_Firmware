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

#endif
 
