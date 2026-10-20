#ifndef __CFG_H__
#define __CFG_H__

#include "nrf24l01_drv.h"

#define PI 3.14159265358979f
#define TARGET_V_CAP 180.0f

// 红外类型
#define NEW_INFRA 1
#define OLD_INFRA 0

#define NEW_MOTOR 1
#define OLD_MOTOR 0

#define OPTICAL_ENCODER 1
#define MAGNETIC_ENCODER 0

#define LEBO_ROBOT 0
#define NEUMANN_S01 1
#define NEUMANN_S02 2
#define NEUMANN_OPTICAL 3
#define TT_0 4

#define OLD_DRIBBLE_MOTOR 0
#define NEW_DRIBBLE_MOTOR 1

#define OLD_DIR 0
#define NEW_DIR 1

//机器人型号
#define ROBOT_VERSION TT_0
// 吸球电机型号
// #define DIRB_MOTOR_VERSION OLD_DRIBBLE_MOTOR

#if (ROBOT_VERSION == LEBO_ROBOT)
    #define INFRA_TYPE OLD_INFRA 
    #define MOTOR_TYPE OLD_MOTOR
    #define ENCODER_TYPE OPTICAL_ENCODER
    #define ENCODER_COUNTS_PER_TURN_SET 1000    //电机一圈编码器线数
    #define DIRB_MOTOR_VERSION OLD_DRIBBLE_MOTOR
    #define DRIB_DIR  OLD_DIR
#elif (ROBOT_VERSION == NEUMANN_S01)
    #define INFRA_TYPE NEW_INFRA
    #define MOTOR_TYPE OLD_MOTOR
    #define ENCODER_TYPE MAGNETIC_ENCODER
    #define ENCODER_COUNTS_PER_TURN_SET 500    //电机一圈编码器线数
    #define DIRB_MOTOR_VERSION OLD_DRIBBLE_MOTOR
    #define DRIB_DIR  OLD_DIR
#elif (ROBOT_VERSION == NEUMANN_S02)
    #define INFRA_TYPE NEW_INFRA
    #define MOTOR_TYPE NEW_MOTOR
    #define ENCODER_TYPE MAGNETIC_ENCODER
    #define ENCODER_COUNTS_PER_TURN_SET 500    //电机一圈编码器线数
    #define DIRB_MOTOR_VERSION NEW_DRIBBLE_MOTOR
    #define DRIB_DIR  OLD_DIR
#elif (ROBOT_VERSION == NEUMANN_OPTICAL)
    #define INFRA_TYPE NEW_INFRA
    #define MOTOR_TYPE OLD_MOTOR
    #define ENCODER_TYPE OPTICAL_ENCODER
    #define ENCODER_COUNTS_PER_TURN_SET 1000    //电机一圈编码器线数
    #define DIRB_MOTOR_VERSION OLD_DRIBBLE_MOTOR
    #define DRIB_DIR  OLD_DIR
#elif (ROBOT_VERSION == TT_0)
    #define INFRA_TYPE NEW_INFRA
    #define MOTOR_TYPE OLD_MOTOR
    #define ENCODER_TYPE OPTICAL_ENCODER
    #define ENCODER_COUNTS_PER_TURN_SET 1000    //电机一圈编码器线数
    #define DIRB_MOTOR_VERSION NEW_DRIBBLE_MOTOR
    #define DRIB_DIR  NEW_DIR
#else
    #error "Please select robot version"
#endif

/* Final scale applied to every wheel speed command. Keep heading reference
 * conversion tied to this value so command feedforward uses the same scale.
 */
#if (MOTOR_TYPE == NEW_MOTOR)
    #define MOTION_COMMAND_GAIN 8.0f
#else
    #define MOTION_COMMAND_GAIN 3.0f
#endif

/* IMU heading tracking parameters. One rotation command unit first represents
 * 0.025 rad/s and is then scaled by MOTION_COMMAND_GAIN in do_move().
 */
#define HEADING_CONTROL_ENABLE             1
#define HEADING_COMMAND_BASE_RAD_S         0.025f
#define HEADING_COMMAND_RAD_S              (HEADING_COMMAND_BASE_RAD_S * MOTION_COMMAND_GAIN)
#define HEADING_SENSOR_YAW_SIGN           -1.0f
#define HEADING_ANGLE_KP                   2.0f
#define HEADING_RATE_KP                    0.08f
#define HEADING_RATE_KI                    0.01f
#define HEADING_RATE_INTEGRAL_LIMIT        200.0f
#define HEADING_HOLD_KP                    0.16f
#define HEADING_HOLD_KI                    0.04f
#define HEADING_HOLD_KD                    0.08f
#define HEADING_YAW_INTEGRAL_LIMIT         200.0f
#define HEADING_MAX_CORRECTION_COMMAND     32.0f
#define HEADING_MAX_ROTATION_COMMAND       511.0f
#define HEADING_MAX_TRACKING_ERROR_DEG     30.0f
#define HEADING_ATTITUDE_TIMEOUT_MS        100U
#define HEADING_UPDATE_TIMEOUT_MS          100U


/* software version*/
#define  software_verison  0x0300;   // V3.0

/* SYSTEM CLK */
#define FREQ_CPU_CLK		SystemCoreClock
#define FREQ_PCLK_2			(FREQ_CPU_CLK / 2)
#define FREQ_PCLK_1			(FREQ_CPU_CLK / 4)	
#define SYS_TICK_FREQ		1000	//in Hz

/* system config */
#define ENABLE_TORQUE_LIMIT		1

/* system parameter */
#define TORQUE_OUTPUT_RATIO		0.8f
#define SHOOTER_INTERVAL 2000	//两次射门间隔时间

/* g_robot parameter define */
#define CHANNEL_NUM			4		
#define SHOOTER_NUM			2
#define ERROR_NUM			500

/* robot parameter in eeprom define */
#define PARAM_LEN			32

/* shoot and chip parameter */
#define SHOOT_TIM_CLK_FRQ		10000	//10KHz Timer clk

/* motor encoder parameter define */
#define ENCODER_TIM_CLK_FREQ		1000000

/* motor parameter define */
#define MOTOR_PWM_FREQ		25000		//25KHz PWM
#define MOTOR_PWM_PERIOD	(168000000 / MOTOR_PWM_FREQ)

/* dribble motor dir define */
#define NEW_DRIBBLE_MOTOR_DIR		1

/* Electrical & Mechanical parameter */
/* motor Electrical parameter */
#define MOTOR_KI  25.5f /* measured in mNm/A */
#define MOTOR_KV  374.0f /* measured in rpm/V */
#define MOTOR_R   1.2f /* measured in ohm */
// #define MOTOR_KI  25.1f /* measured in mNm/A */
// #define MOTOR_KV  380.0f /* measured in rpm/V */
// #define MOTOR_R   0.464f /* measured in ohm */

#define LIU_WANG_CONST 3.043f // 老车电机与轮子传动比-2024国赛版，变量名为纪念lxt与wyx2025.11.18的工作而起
#define WHEEL_DIAMETER_SMALL 0.0554f //小轮直径(m) 2025.11.19测量结果
#define WHEEL_DIAMETER_BIG 0.0579f //大轮直径(m) 2025.11.19测量结果

/* motor pid parameter */
#define SPEED_PID 1
#define POSITION_PID 0

#define POSITION_PID_KP  	30.0f
#define POSITION_PID_KI  	0.8f
#define POSITION_PID_KD		10.0f

// #define POSITION_PID_KP  	0.0f
// #define POSITION_PID_KI  	0.0f
// #define POSITION_PID_KD		0.0f

#if (MOTOR_TYPE == OLD_MOTOR)
#define MOTOR_PID_KP  	0.1825f
#define MOTOR_PID_KI  	0.0475f
#define MOTOR_PID_KD	0.0f

#define MOTOR_PID_KP1  	0.09f    //PID0
#define MOTOR_PID_KI1  	0.0005f
#define MOTOR_PID_KD1	  0.00f

#define MOTOR_PID_KP2  	0.09f
#define MOTOR_PID_KI2  	0.0005f
#define MOTOR_PID_KD2	  0.00f

#define MOTOR_PID_KP3  	0.09f
#define MOTOR_PID_KI3  	0.0005f
#define MOTOR_PID_KD3	  0.00f

#define MOTOR_PID_KP4 	0.09f
#define MOTOR_PID_KI4 	0.0005f
#define MOTOR_PID_KD4	  0.00f

#elif (MOTOR_TYPE == NEW_MOTOR)
#define MOTOR_PID_KP  	0.1825f
#define MOTOR_PID_KI  	0.0475f
#define MOTOR_PID_KD	0.0f

#define MOTOR_PID_KP1  	0.09f    //PID0
#define MOTOR_PID_KI1  	0.0005f
#define MOTOR_PID_KD1	  0.00f

#define MOTOR_PID_KP2  	0.09f
#define MOTOR_PID_KI2  	0.0005f
#define MOTOR_PID_KD2	  0.00f

#define MOTOR_PID_KP3  	0.09f
#define MOTOR_PID_KI3  	0.0005f
#define MOTOR_PID_KD3	  0.00f

#define MOTOR_PID_KP4 	0.09f
#define MOTOR_PID_KI4 	0.0005f
#define MOTOR_PID_KD4	  0.00f

#endif

#define GYRO_PID_KP   0.2f
#define GYRO_PID_KI	 0.0575f
#define GYRO_PID_KD	 0.0f

#define MOTOR_TORQUE_LIMIT	275.0f
// #define MOTOR_TORQUE_LIMIT	97.1f

/* Set 4 while using the original pack, 6 for the new pack. COEX, Xuanting Liu */
#ifndef BATTERY_CELLS
#define BATTERY_CELLS 6U
#endif
#define BATTERY_V (3.7f * BATTERY_CELLS)

#define MAX_SHOT_STRENGTH 127
#define MAX_ACC 10	/* 限定车子的加速度 */

#define WHEEL_CENTER_OFFSET 0.080f  /* 轮子距车中心距离(m) 2024.4.16中心重新测定结果*/

#define STOP_THRESHOLD 60 

//王奕轩新底板54°
#define D_WHEEL_ANGLE_FRONT   53    /*前轮与轴线角度(度) 前轮轮子轴线与小车前后轴线角度*/
#define D_WHEEL_ANGLE_BACK    142   /*后轮与轴线角度(度) 后轮轮子轴线与小车前后轴线角度*/

#define FRONT_MODIFY_2024 2.33 // 后轮偏转角
#define BACK_MODIFY_2024 4.08

// 以下三个参数目前已被弃用，为远古版本车辆参数
// 新参数请见上文
#define D_WHEEL_REDUCTION_RATIO_X 3		/*减速比*/     // X.YZ
#define D_WHEEL_REDUCTION_RATIO_YZ 18	/*减速比*/   //X.YZ      
#define WHEEL_RADIUS 0.028f            //轮子半径 单位[m]

#define SHOOT_DELAY 15

/* TIGERs power board and MOTOR5 parameters. Author: COEX, Xuanting Liu */
#define KICKER_CAP_FULL_SCALE_V 251.42f
#define KICKER_MAX_TEMP_C 60.0f
#define KICKER_CALIBRATION_CONFIRMED 1
#define INA226_SHUNT_MOHM 2.0f
#define DRIBBLER_POLE_PAIRS 1U
#define DRIBBLER_SAMPLE_MS 100U /* FG telemetry only. */
/* SRC-tsak do_dribbler(): all legal nonzero gears use the motor-type gain. */
#if (DIRB_MOTOR_VERSION == OLD_DRIBBLE_MOTOR)
#define DRIBBLER_FIXED_DUTY 0.50f
#elif (DIRB_MOTOR_VERSION == NEW_DRIBBLE_MOTOR)
#define DRIBBLER_FIXED_DUTY 0.55f
#else
#error "Invalid dribbler motor type"
#endif
#define DRIBBLER_MAX_DUTY 0.15f

/* Legacy radio voltage encoding only; no longer MCU ADC calibration. */
#define BAT_V_ADC_GAIN			(3.3f / 256.0f / (3300.0f / (22000.0f + 3300.0f)))  //电池电压电阻分压系数

// #define CAP_V_ADC_GAIN			(3.3f / 256.0f / (5.1f / (249.0f + 5.1f)))    //充电电容电阻分压系数
#define CAP_V_ADC_GAIN			(3.3f / 256.0f / (12.4f / (1800.0f + 12.4f)) * 1.225f / 1.5f)    //充电电容电阻分压系数

/* battery low power protection time */
#define LOW_POWER_TIME		(5 * SYS_TICK_FREQ) //5s

/* do timer action time define */
#define PID_COUNTER_OVERFLOW 2
#define SECOND_COUNTER_OVERFLOW 	SYS_TICK_FREQ
#define HEARTBEAT_TIME 	SYS_TICK_FREQ
#define COMM_TIMEOUT_TIME 500
#define IDENTIFY_CPUID_TIMEOUT_TIME 10000
#define INFRA_COUNTER_ON_OVERFLOW 6
#define INFRA_COUNTER_OFF_OVERFLOW 2
#define POWER_MON_TIME 1000
#define GYRO_MON_TIME 15
#define DUTY_CHANGE_OVERFLOW 3000
#define SHOOT_COUNTER_OVERFLOW 40
#define MOVE_CONTROL_OVRFLOW 7
#define COMM_COUNTER_LONG_OVERFLOW 3000
#define DO_RECIEVE_OVERFLOW 7
#define DO_FILTER_COUNT 10000

/*  Function selection */
#define ENABLE_BEEP
#define ENABLE_LED
#define ENABLE_SHOOTER
#define ENABLE_HEARTBEAT
#define ENABLE_POWERMON
//#define ENABLE_INFRA_BEEP
#define ENABLE_BLE_FUN
#define ENABLE_JOYSTICK_CONTROL

/* NRF24L01 parameter define */
#define NRF24L01_SPI			SPI3
#define HAL_NRF24L01_SPI        hspi3
#define NRF24L01_ADDR_COUNT    AW_5BYTES	//5 byte

#define NRF24L01_ADDR1_4     0x10
#define NRF24L01_ADDR1_3     0x71
#define NRF24L01_ADDR1_2     0x45
#define NRF24L01_ADDR1_1     0x98
#define NRF24L01_ADDR1_0     0x00

#define PACKET_LEN    25   /* This Packet_len must accord with Transfer's Packet_len */
#define PACKET_LEN_UP PACKET_LEN	/* 2401向上通讯字节数 */

/* IR BALL PWM define */
#define IR_BALL_PWM_FREQ	38000	//38KHz
#define IR_BALL_PWM_DUTY	20		//20%

/* Low power limit define */
/* Per-cell limits; no automatic cell-count detection. COEX, Xuanting Liu */
#define WARNING_POWER_A (3.5f * BATTERY_CELLS)
#define FORCESTOP_POWER_A (3.3f * BATTERY_CELLS)
#define SHUTDOWN_POWER_A (3.0f * BATTERY_CELLS)
#define BATTERY_QUIET_MS 5000U
#define BATTERY_SHUTDOWN_MS 1000U

/* Dedicated TIGERs-style discharge parameters. COEX, Xuanting Liu */
#define SHUTDOWN_PDN_DEBOUNCE_MS 50U
#define SHUTDOWN_FORCE_HOLD_MS 3000U
#define KICKER_TIMER_TICK_US 20U
#define KICKER_DISCHARGE_TARGET_V 2.0f
#define KICKER_DISCHARGE_COOLDOWN_MS 80U
#define KICKER_DISCHARGE_PROGRESS_MS 2000U
#define KICKER_DISCHARGE_MIN_DROP_V 0.25f
#define KICKER_DISCHARGE_SENSOR_TIMEOUT_MS 500U
#define KICKER_DISCHARGE_TIMEOUT_MS 60000U

#define WARNING_POWER_D 	(WARNING_POWER_A * 3.3f / (22 + 3.3f) / 3.3f * 256)
#define FORCESTOP_POWER_D 	(FORCESTOP_POWER_A * 3.3f / (22 + 3.3f) / 3.3f * 256)

#define BLUETOOTH_MAX_NAME 10   //蓝牙名称最大长度
#define BLUETOOTH_MAX_CODE 6   //蓝牙密码最大长度

#define MPU6050_GYRO_USED  0

/* This board, Z-up/down measurements on 2026-10-10. COEX, Xuanting Liu
 * Corrected Z = (measured Z - bias) * scale, before stationary calibration.
 * Two-face estimate only; X/Y remain unchanged. Recalibrate for another MPU.
 * Disable this correction with bias=0.0f and scale=1.0f.
 */
#define MPU6050_ACCEL_Z_BIAS_G (-0.208984375f)
#define MPU6050_ACCEL_Z_SCALE 0.982489806f


//射门力度调整曲线参数
#define SHOOT_POWER_CURVE_A 13
#define SHOOT_POWER_CURVE_B 70

//吸球电机转速放大参数 1~1.8
#define DRIBBLER_POWER_AMP 1.8


#endif

