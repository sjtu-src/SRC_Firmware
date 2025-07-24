#ifndef __CFG_H__
#define __CFG_H__

#define PI 3.14159265358979f


/* software version*/
#define  software_verison  0x0200;   // V2.0

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
#define MOTOR_PWM_PERIOD	(SystemCoreClock / MOTOR_PWM_FREQ)

/* dribble motor dir define */
#define NEW_DRIBBLE_MOTOR_DIR		1

/* Electrical & Mechanical parameter */
/* motor Electrical parameter */
#define MOTOR_KI  25.5f /* measured in mNm/A */
#define MOTOR_KV  374.0f /* measured in rpm/V */
#define MOTOR_R   1.2f /* measured in ohm */

/* motor pid parameter */
//#define MOTOR_PID_KP  	210
//#define MOTOR_PID_KI  	150
//#define MOTOR_PID_KD	0

//#define MOTOR_PID_KP  	50
//#define MOTOR_PID_KI  	30
//#define MOTOR_PID_KD	5

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


#define GYRO_PID_KP   0.2f
#define GYRO_PID_KI	 0.0575f
#define GYRO_PID_KD	 0.0f

#define MOTOR_TORQUE_LIMIT	175.0f

#define BATTERY_V   13.0f /* measured in V, maybe detected while running */

#define MAX_SHOT_STRENGTH 127
//#define MAX_ACC 20	/* 限定车子的加速度 */
#define MAX_ACC 50	/* 限定车子的加速度 */

#define WHEEL_CENTER_OFFSET 0.080f  /* 轮子距车中心距离(m) 2024.4.16中心重新测定结果*/

#define STOP_THRESHOLD 60 

#define D_WHEEL_ANGLE_FRONT   47    /*55		 前轮与轴线角度(度) 前轮轮子轴线与小车前后轴线角度*/

/**
* @brief 王奕轩新底板54°
* /
#define D_WHEEL_ANGLE_BACK_2014 128	 /* 后轮与轴线角度(度) */
#define D_WHEEL_ANGLE_BACK_2013 135

#define FRONT_MODIFY_2024 2.33 // 后轮偏转角
#define BACK_MODIFY_2024 4.08

#define D_WHEEL_REDUCTION_RATIO_X 3		/*减速比*/     // X.YZ
#define D_WHEEL_REDUCTION_RATIO_YZ 18	/*减速比*/   //X.YZ      
#define WHEEL_RADIUS 0.028f            //轮子半径 单位[m]

//#define ENCODER_COUNTS_PER_TURN_SET 256
//#define ENCODER_COUNTS_PER_TURN_SET 512    //电机一圈编码器线数
#define ENCODER_COUNTS_PER_TURN_SET 1024    //电机一圈编码器线数

#define SHOOT_DELAY 15

/* ADC parameter */
#define BAT_V_ADC_GAIN			(3.3f / 256.0f / (3300.0f / (22000.0f + 3300.0f)))  //电池电压电阻分压系数

// #define CAP_V_ADC_GAIN			(3.3f / 256.0f / (5.1f / (249.0f + 5.1f)))    //充电电容电阻分压系数
#define CAP_V_ADC_GAIN			(3.3f / 256.0f / (10.0f / (1800.0f + 10.0f)))    //充电电容电阻分压系数

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
#define WARNING_POWER_A 13.2f       //电池电压阈值1 单位v 蜂鸣器响
#define FORCESTOP_POWER_A 13.8f     //电池电压阈值2  停止机器人

#define WARNING_POWER_D 	(WARNING_POWER_A * 3.3f / (22 + 3.3f) / 3.3f * 256)
#define FORCESTOP_POWER_D 	(FORCESTOP_POWER_A * 3.3f / (22 + 3.3f) / 3.3f * 256)

#define BLUETOOTH_MAX_NAME 10   //蓝牙名称最大长度
#define BLUETOOTH_MAX_CODE 6   //蓝牙密码最大长度

#define MPU6050_GYRO_USED  1

//射门力度调整曲线参数
#define SHOOT_POWER_CURVE_A 13
#define SHOOT_POWER_CURVE_B 70

//吸球电机转速放大参数 1~1.8
#define DRIBBLER_POWER_AMP 1.8


#endif

