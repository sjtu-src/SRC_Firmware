/* USER CODE BEGIN Header */
/* coding: utf-8 */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "gpio.h"
#include "robot.h"
#include "tim.h"
#include "motor.h"
#include "comm.h"
#include "action.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
SemaphoreHandle_t xMotorTickSem = NULL;

timer_t power_mon_timer;
timer_t heart_led_timer;
timer_t rf_comm_tim;           //发射机�?�信超时时间
timer_t identify_cpuid_tim;   //cpuid认证超时时间 设置�?????10S
timer_t shoot_interval_timer;

extern char g_do_set_receive_mode_flag;

/* USER CODE END Variables */
osThreadId MotorUpdateHandle;
osThreadId StatesUpdateHandle;
osThreadId IdleHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Update_Motor(void const * argument);
void RobotTask(void const * argument);
void Do_Default(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of MotorUpdate */
  osThreadDef(MotorUpdate, Update_Motor, osPriorityHigh, 0, 128);
  MotorUpdateHandle = osThreadCreate(osThread(MotorUpdate), NULL);

  /* definition and creation of StatesUpdate */
  osThreadDef(StatesUpdate, RobotTask, osPriorityNormal, 0, 2048);
  StatesUpdateHandle = osThreadCreate(osThread(StatesUpdate), NULL);

  /* definition and creation of Idle */
  osThreadDef(Idle, Do_Default, osPriorityIdle, 0, 128);
  IdleHandle = osThreadCreate(osThread(Idle), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_Update_Motor */
/**
  * @brief  Function implementing the MotorUpdate thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Update_Motor */
void Update_Motor(void const * argument)
{
  /* USER CODE BEGIN Update_Motor */
  xMotorTickSem = xSemaphoreCreateBinary();
  /* Infinite loop */
  for(;;)
  {
    if(xSemaphoreTake(xMotorTickSem, pdMS_TO_TICKS(2000)) == pdTRUE) 
    {
        do_update_motor();     
        //Debug_Here();  // 调试断点
    }
  }
  /* USER CODE END Update_Motor */
}

/* USER CODE BEGIN Header_RobotTask */
/**
* @brief Function implementing the StatesUpdate thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RobotTask */
void RobotTask(void const * argument)
{
  /* USER CODE BEGIN RobotTask */
	int i;
	
	i = g_robot.mode + 1;		

	while(i != 0)
	{
			BEEP_ON();
			osDelay(100);
			BEEP_OFF();
			osDelay(100);
			i--;
	}

	/* initial timer */
	power_mon_timer = get_one_timer(POWER_MON_TIME);
	heart_led_timer = get_one_timer(HEARTBEAT_TIME);
	rf_comm_tim = get_one_timer(COMM_TIMEOUT_TIME); //无线通信模式超时时间
	shoot_interval_timer = get_one_timer(1);
	identify_cpuid_tim = get_one_timer(IDENTIFY_CPUID_TIMEOUT_TIME); //cpuid认证超时时间设置10s 10s认证不成功则停止机器

  //Beep_Show_32bit(power_mon_timer);

	/* start motor */
	start_motor();
  HAL_TIM_Base_Start_IT(&htim12);  // 启动TIM12中断
  /* Infinite loop */
  for(;;)
  {
    do_robot_run();
    //osDelay(1);
  }
  /* USER CODE END RobotTask */
}

/* USER CODE BEGIN Header_Do_Default */
/**
* @brief Function implementing the Idle thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Do_Default */
void Do_Default(void const * argument)
{
  /* USER CODE BEGIN Do_Default */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Do_Default */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
