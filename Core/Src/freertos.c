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
#include "oled.h"
#include "misc.h"
#include "MPU6050_driver.h"
#include "i2c.h"
#include "board_monitor.h"
#include "dribbler_control.h"
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

/* Release communication and display only after startup owns the bus and buzzer. */
static volatile uint8_t startup_complete = 0U;

timer_t power_mon_timer;
timer_t heart_led_timer;
timer_t rf_comm_tim;           //发射机�?�信超时时间
timer_t identify_cpuid_tim;   //cpuid认证超时时间 设置�?10S
timer_t shoot_interval_timer;

extern char g_do_set_receive_mode_flag;
extern char g_set_receive_mode_flag;
extern packet_robot_t src_robot_packet;

/* USER CODE END Variables */
osThreadId MotorUpdateHandle;
osThreadId CommunicationHandle;
osThreadId StatesUpdateHandle;
osThreadId DisplayStateHandle;
osThreadId IdleHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void Update_Imu(void const *argument);
static void Update_Board(void const *argument);

/* USER CODE END FunctionPrototypes */

void Update_Motor(void const * argument);
void Do_Comm(void const * argument);
void RobotTask(void const * argument);
void Do_Display(void const * argument);
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
  I2C2_CreateMutex();
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
  osThreadDef(Communication, Do_Comm, osPriorityAboveNormal, 0, 512);
  CommunicationHandle = osThreadCreate(osThread(Communication), NULL);

  /* definition and creation of StatesUpdate */
  osThreadDef(StatesUpdate, RobotTask, osPriorityNormal, 0, 1024);
  StatesUpdateHandle = osThreadCreate(osThread(StatesUpdate), NULL);

  /* definition and creation of DisplayState */
  osThreadDef(DisplayState, Do_Display, osPriorityLow, 0, 512);
  DisplayStateHandle = osThreadCreate(osThread(DisplayState), NULL);

  /* definition and creation of Idle */
  osThreadDef(Idle, Do_Default, osPriorityIdle, 0, 128);
  IdleHandle = osThreadCreate(osThread(Idle), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  osThreadDef(ImuUpdate, Update_Imu, osPriorityAboveNormal, 0, 512);
  if (osThreadCreate(osThread(ImuUpdate), NULL) == NULL) Error_Handler();
  osThreadDef(BoardUpdate, Update_Board, osPriorityAboveNormal, 0, 384);
  if (osThreadCreate(osThread(BoardUpdate), NULL) == NULL) Error_Handler();
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
  if (xMotorTickSem == NULL) Error_Handler();
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
  while (startup_complete == 0U)
  {
    osDelay(10U);
  }
  /* Infinite loop */
  for(;;)
  {
    if(g_robot.mode == NORMAL_MODE || g_robot.mode == CRAY_MODE)
    {
        do_communication();
        do_game_telemetry_up();

        if(check_timer(rf_comm_tim)) 
          {
            g_do_set_receive_mode_flag = 1; 
            
            for(int i = 0; i < CHANNEL_NUM; i++)
            {
              g_robot.wheels[i].set = 0;
              g_robot.wheels[i].cur_position = 0;
              g_robot.wheels[i].pid.set = 0;
            }

            do_dribbler(0);
            HeadingControl_Reset(&g_robot.heading_control);
            do_move(0,0,0,g_robot.PID_type);
            do_shoot(0,0);
            do_chip(0,0);
            //Very important: clear the packet when communication timeout.
            src_robot_packet.shoot = 0;
            src_robot_packet.chip = 0;
            src_robot_packet.dribbler = 0;
            src_robot_packet.speed_x = 0;
            src_robot_packet.speed_y = 0;
            src_robot_packet.speed_rot = 0;

            start_nRF24L01_RX();	
            rf_comm_tim = get_one_timer(COMM_TIMEOUT_TIME);
            identify_cpuid_tim = get_one_timer(IDENTIFY_CPUID_TIMEOUT_TIME);
          }

				if(g_do_set_receive_mode_flag)	//发出数据包后置为1，等待数据发出去后将模式修改为接收模式			
          {
            if(g_set_receive_mode_flag >= 3)
            {				
              start_nRF24L01_RX();
            
              g_set_receive_mode_flag = 0;
              g_do_set_receive_mode_flag = 0; 	//置位可接收标志位
            }
          }
    }
    osDelay(5);
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

    /* Separate the gyro result from the preceding mode beep sequence. */
    BEEP_OFF();
    osDelay(1000U);
    if (InitMPU6050() != 0)
    {
        /* Success: two short beeps, 150 ms each, separated by 150 ms. */
        for (i = 0; i < 2; i++)
        {
            BEEP_ON();
            osDelay(150U);
            BEEP_OFF();
            osDelay(150U);
        }
    }
    else
    {
        /* Failure: one long beep. Continue startup without gyro assistance. */
        BEEP_ON();
        osDelay(1000U);
        BEEP_OFF();
    }
    /* Keep runtime power/infrared indications separate from the gyro result. */
    osDelay(500U);

	/* initial timer */
	power_mon_timer = get_one_timer(POWER_MON_TIME);
	heart_led_timer = get_one_timer(HEARTBEAT_TIME);
	rf_comm_tim = get_one_timer(COMM_TIMEOUT_TIME); //无线通信模式超时时间
	shoot_interval_timer = get_one_timer(1);
	identify_cpuid_tim = get_one_timer(IDENTIFY_CPUID_TIMEOUT_TIME); //cpuid认证超时时间设置10s 10s认证不成功则停止机器
  do_power_monitor();

	/* start motor */
	if(!board_actuators_disabled()) start_motor();
  HAL_TIM_Base_Start_IT(&htim12);  // 启动TIM12中断
  startup_complete = 1U;
  /* Infinite loop */
  for(;;)
  {
    do_robot_run();
    osDelay(1);
  }
  /* USER CODE END RobotTask */
}

/* USER CODE BEGIN Header_Do_Display */
/**
* @brief Function implementing the DisplayState thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Do_Display */
void Do_Display(void const * argument)
{
  /* USER CODE BEGIN Do_Display */
  uint32_t display_tick = HAL_GetTick();
  /* Shutdown may interrupt startup; do not hide it behind the logo delay.
   * Author: COEX, Xuanting Liu
   */
  while(HAL_GetTick() - display_tick < 2000U && !board_is_shutdown()) osDelay(10U);
  while (startup_complete == 0U && !board_is_shutdown())
  {
    osDelay(10U);
  }
  /* The startup MPU6050 check has released I2C2. */
  OLED_Display_Init();
  /* Keep the startup logo visible before the first status refresh.
   * I2C2 is unlocked while this task sleeps. Author: COEX, Xuanting Liu
   */
  display_tick = HAL_GetTick();
  while(HAL_GetTick() - display_tick < 2000U && !board_is_shutdown()) osDelay(10U);
  /* Infinite loop */
  for(;;)
  {
    Robot_State_Display();
    osDelay(100U);
  }
  /* USER CODE END Do_Display */
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
/*******************************************************************************
* @brief Sample the IMU at nominal 100 Hz; publish a consistent robot snapshot.
* @note Leave the board stationary for at least two seconds after startup.
*******************************************************************************/
static void Update_Imu(void const *argument)
{
    static imu_filter_t filter;
    float accel[3], gyro[3], temperature;
    TickType_t wake;
    uint32_t last_fresh;
    (void)argument;
    while (startup_complete == 0U) osDelay(10U);
    ImuFilter_Reset(&filter);
    last_fresh = HAL_GetTick();
    wake = xTaskGetTickCount();
    for (;;)
    {
        int result = g_mpu6050_status == MPU6050_INIT_OK ?
                     MPU6050_ReadMotion(accel, gyro, &temperature) : -1;
        uint32_t now = HAL_GetTick();
        if (result > 0)
        {
            ImuFilter_Update(&filter, accel, gyro, temperature, now);
            last_fresh = now;
        }
        else if (result < 0 || now - last_fresh > 100U)
        {
            ImuFilter_ReadError(&filter);
        }
        taskENTER_CRITICAL();
        g_robot.attitude = filter.attitude;
        taskEXIT_CRITICAL();
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(10U));
    }
}

/*******************************************************************************
* @brief Power and FG1 service runs during startup too; never blocks in an ISR.
* @author COEX, Xuanting Liu
*******************************************************************************/
static void Update_Board(void const *argument)
{
	TickType_t wake = xTaskGetTickCount();
	(void)argument;
	for(;;)
	{
		board_monitor_update();
		dribbler_update();
		vTaskDelayUntil(&wake, pdMS_TO_TICKS(10U));
	}
}

/* USER CODE END Application */
