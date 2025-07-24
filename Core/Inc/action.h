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

static float last_speed_x = 0;
static float last_speed_y = 0;
static float last_speed_rot = 0;

#endif
