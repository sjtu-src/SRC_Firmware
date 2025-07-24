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

// 插值用结构体，2024.4.22wyq
typedef struct
{
	int speed[3];
	int idx;
	int idx_t;
	int idx_prim;
	int acc;
	int t[2];
	int t_idx;
}v_que;

/*// 卡尔曼滤波器，wyq 2024.4.26
typedef struct
{
	int x_k;
	int m_k;
	int p_error; //过程噪声协方差
	int m_error; //观测误差协方差
	int e_error; //估计误差协方差
	int kar; //卡尔曼增益
	
}karman;

typedef struct
{
	int idx;
	int v_real[3];
	int acc;
}reque;*/

void que_init(v_que*k);
int v_output(v_que*k, int v);
/*//karman滤波器实现函数 wyq 2024.4.26
void karman_init(karman*k, int x_guess, int m_guess, int m_error, int e_error, int p_error);
int karman_value(karman*k, int m_value, int acc); 
void karman_update(karman*k, int x);

void reque_build(reque*q);
int change_que(reque*q, int x);*/

int on_robot_command( packet_robot_t *packet );
int do_robot_command( packet_robot_t *packet );
void EXTI0_IRQHandler(void);
int init_shooter(void);
int do_dribbler( int dribbler );
int do_move( int speed_x, int speed_y, int speed_rot );
void suddenly_stop(void);
int do_shoot( int shoot, int chip );
int do_chip(int shoot, int chip );
int update_shooter(void);
int set_test_shooter(void);

static float last_speed_x = 0;
static float last_speed_y = 0;
static float last_speed_rot = 0;

#endif
