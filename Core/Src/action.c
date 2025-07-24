#include <math.h>
#include <stdlib.h>
#include "math.h"
#include "action.h"
#include "tim.h"
#include "cfg.h"
#include "gpio.h"
#include "robot.h"
#include "misc.h"
#include "motor.h"
#include "MPU6050_driver.h"

shooter_t g_shooter[ SHOOTER_NUM ];

/*******************************************************************************
* @brief 平射挑射功能关闭
* @author Xuanting Liu
*******************************************************************************/
int shooter_off(void)
{
	u8 i;
	
	shoot_off();
	chip_off();
	for(i = 0; i < SHOOTER_NUM; i++)
	{
		g_shooter[i].count_down = 0;
	}

	return 0;
}

/*******************************************************************************
* @brief 初始化平射挑射
* @author Xuanting Liu
*******************************************************************************/
int init_shooter(void)
{
	shooter_off();
	return 0;
}
