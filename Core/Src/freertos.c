/* USER CODE BEGIN Header */
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
#include "gpio.h"
#include "robot.h"
#include "tim.h"
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
/* USER CODE END Variables */
osThreadId MotorUpdateHandle;
osThreadId CommunicationHandle;
osThreadId StatesUpdateHandle;
osThreadId IdleHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Update_Motor(void const * argument);
void Do_Comm(void const * argument);
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

  /* definition and creation of Communication */
  osThreadDef(Communication, Do_Comm, osPriorityAboveNormal, 0, 256);
  CommunicationHandle = osThreadCreate(osThread(Communication), NULL);

  /* definition and creation of StatesUpdate */
  osThreadDef(StatesUpdate, RobotTask, osPriorityNormal, 0, 128);
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
          // uint32_t start_tick = DWT->CYCCNT;  // 性能监测

          // // 1. 获取目标�? (MODE0:通信队列, MODE3:自检生成)
          // update_motor_targets(wheels); 

          // // 2. 读取编码器�?�（原子操作�?
          // for(int i=0; i<4; i++) {
          //   int32_t delta = atomic_exchange(&encoder_delta[i], 0);
          //   wheels[i].actual_rpm = (delta * 60000) / (PPR * CONTROL_PERIOD_MS);
          // }

          // // 3. PI计算 & 输出
          // for(int i=0; i<4; i++) {
          //   float out = compute_pi(&wheels[i]);
          //   pwm_set_duty(i, (uint32_t)(out * PWM_SCALE_FACTOR));
          // }

          // // 4. 实时性监�? (超时警告)
          // uint32_t exec_time = (DWT->CYCCNT - start_tick) / CPU_FREQ_MHZ;
          // if(exec_time > 900) {  // 超过900us警告
          //   log_warning("MotorCTRL Overrun: %dus", exec_time);
          // }
          Debug_Here();  // 调试断点
        }
  }
  /* USER CODE END Update_Motor */
}

/* USER CODE BEGIN Header_Do_Comm */
/**
* @brief Function implementing the Communication thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Do_Comm */
void Do_Comm(void const * argument)
{
  /* USER CODE BEGIN Do_Comm */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Do_Comm */
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
  HAL_TIM_Base_Start_IT(&htim12);  // 启动TIM12中断
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
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
