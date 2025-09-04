#ifndef __ROBOT_H__
#define __ROBOT_H__

#include "cfg.h"
#include "pid.h"
#include "typedef.h"
#include "tim.h"


/***控制器几种工作模式选择***/
typedef enum _mode
{
	NORMAL_MODE = 0,     //正常比赛模式 与通讯机通讯
    SELFTEST_MODE = 3,   //自检模式
	TEST_DRIBBLING_MODE = 2, //自主吸球模式
	CRAY_MODE = 7 //CRAY模式
}mode_t;


typedef struct _wheel_
{
	pid_t	pid;
	volatile float speed;     //小车车体合成线速度值 单位[m/s]
	volatile int set;         //pid速度设置值 单位count/s
	volatile int cur_speed;   //当前速度 单位[count/s]
} wheel_t;

typedef struct _error_
{
	u64 time; /* 0:6v motor; 1:12v motor */
	int name;
} err_t;

typedef struct _robot_
{
	int num; 	/* g_robot num load from circuit configuration switch 机器号为拨码盘+1*/
	mode_t mode;	/* g_robot mode */
	u8 frq;	/* 2.4G rf freq */
	
	wheel_t wheels[ CHANNEL_NUM ]; //各个轮子的pid参数
	u16 dribbler;
	
	float kv2n;                    // 74037 m/s和count/s 转换系数 单位count/m  用于电机旋转速度转换为线速度
	int shoot;
	int chip;
	unsigned char is_ball_detected;
	int is_cap_low;
	int is_pow_low;
	int speed_x;
	int speed_y;

	vu8 cap_v;                //电容电压 单位[bit]
	vu8 bat_v;

	volatile float cap_v_f;  //充电电容电压 单位[v]
	volatile float bat_v_f;  //电池电压

	float sin_angle[ CHANNEL_NUM]; //车轮与正方向轴线的角度正弦
	float cos_angle[ CHANNEL_NUM];
	
	float cos_mod_angle[ CHANNEL_NUM]; //后轮修正角度

	err_t error[ERROR_NUM];
	
	u16 firmware_version;    //软件版本

} robot_t;

void SRC_Robot_Init(void);
void init_robot(void);
void Debug_Here(void);
void Beep_Show_8bit(u8 val);
void Beep_Show_32bit(u32 val);
void inc_receive_mode_flag(void);
void do_robot_run(void);
int do_power_monitor(void);
void on_robot_command(void);

extern robot_t g_robot;

#endif

