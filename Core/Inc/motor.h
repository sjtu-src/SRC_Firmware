#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "typedef.h"
#include "cfg.h"

void init_motor(void);
void set_motor_dir(u8 m1, u8 m2, u8 m3, u8 m4);
void set_motor_break(u8 m1, u8 m2, u8 m3, u8 m4);
void set_motor_pwm(u16 m1, u16 m2, u16 m3, u16 m4);
void do_update_motor(void);
void update_encoder(int *speed);
void start_encoder(void);
void start_motor(void);


#endif
