#include "misc.h"
#include "gpio.h"
#include "cfg.h"
#include "stm32f4xx.h"
#include "robot.h"
#include "math.h"
#include "main.h"
#include "tim.h"

void init_dribbler(void)
{
	HAL_GPIO_WritePin(MOTOR_BRK5_GPIO_Port, MOTOR_BRK5_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOTOR_DIR5_GPIO_Port, MOTOR_DIR5_Pin, GPIO_PIN_RESET);
	__HAL_TIM_ENABLE(&htim11);
}
