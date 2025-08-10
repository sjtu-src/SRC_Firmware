#ifndef __ACTION_H__
#define __ACTION_H__

#include "typedef.h"
#include "packet.h"

#define SHOOT 0
#define CHIP 1

typedef struct _shooter_
{
	int strength;
	int count_down;
} shooter_t;


int init_shooter(void);
int shooter_off(void);
int set_test_shooter(void);
void do_dribbler( int dribbler );
void set_dribbler(u16 value, u8 dir);
int set_shooter(int channel, int value);
void do_shoot( int shoot, int chip );
void do_chip( int shoot, int chip );
void update_shooter(void);
void do_acc_handle_move(int speed_x,int speed_y,int speed_rot);
void do_move(int speed_x,int speed_y,int speed_rot);

static float last_speed_x = 0;
static float last_speed_y = 0;
static float last_speed_rot = 0;

#endif
