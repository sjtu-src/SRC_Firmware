/* coding: utf-8 */
#include "math.h"
#include "cfg.h"
#include "pid.h"
#include "misc.h"
#include "robot.h"

GYRO_PID_STRUCT gyro_pid = {0};

/*******************************************************************************
* @brief 计算最大输出
* @param max_torque 最大扭矩
* @param speed 当前速度
* @param bat_v 电池电压
* @return 最大输出值
******************************************************************************/
long calc_max_output( float max_torque, long speed, float bat_v )
{
	float Vin = max_torque / MOTOR_KI * MOTOR_R + (float)speed / MOTOR_KV;

	return MOTOR_PWM_PERIOD * Vin / bat_v;
}


/*******************************************************************************
* @brief pid参数初始化
* @note 增量式pid u(k)=u(k-1)+deta_u(k)
* @note deat_u(k)=(kp+ki+kd)*e(k)-(kp+2kd)*e(k-1)+kd*e(k-2)
******************************************************************************/
void pid_init(pid_t *pid, float Kp, float Ki, float Kd )
{
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;

	pid->A = Kp + Ki + Kd;
	pid->B = Kp + 2 * Kd;
	pid->C = Kd;
  
  	pid->set = 0;
	
	pid->limit = MOTOR_PWM_PERIOD;
    pid->torque_limit = MOTOR_TORQUE_LIMIT;

	pid_reinit(pid);
}



/*******************************************************************************
* @brief 陀螺仪闭环控制pid参数计算初始化
* @note 增量式pid u(k)=u(k-1)+deta_u(k)
* @note deat_u(k)=(kp+ki+kd)*e(k)-(kp+2kd)*e(k-1)+kd*e(k-2)
******************************************************************************/
void gyro_pid_init(GYRO_PID_STRUCT *pid, float Kp, float Ki, float Kd )
{
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;

	pid->A = Kp + Ki + Kd;
	pid->B = Kp + 2 * Kd;
	pid->C = Kd;
  	
	pid->limit = 150;     
   
	pid->e1 = 0;
	pid->e2 = 0;
	pid->e3 = 0;
	pid->set_value = 0;   //旋转值设定为0
	pid->feedback_speed = 0;
	pid->out = 0;
}


/*******************************************************************************
* @brief pid参数计算初始化 
******************************************************************************/
void pid_reinit(pid_t *pid)
{
	pid->e1 = 0;
	pid->e2 = 0;
	pid->e3 = 0;
	pid->set = 0;
}

/*******************************************************************************
* @brief pid控制计算 
* @note 增量式pid u(k)=u(k-1)+deta_u(k)
* @note deat_u(k)=(kp+ki+kd)*e(k)-(kp+2kd)*e(k-1)+kd*e(k-2)
*******************************************************************************/
int pid_step(pid_t *pid, int cur_value, float bat_v )
{
	long rpm = 0; 
	long max_output_for_torque = 0;
	int d_out;
  
	/* standard digital PID algorithm */
	pid->e3 = pid->e2;
	pid->e2 = pid->e1;
	pid->e1 = pid->set - cur_value;

	d_out = pid->A * pid->e1 - pid->B * pid->e2 + pid->C * pid->e3;
  
	pid->out = pid->out + d_out;
	
	/* output limit */
	if( pid->out > pid->limit )
		pid->out = pid->limit;
	else if( pid->out < -pid->limit )
		pid->out = -pid->limit;
  
	#ifdef ENABLE_TORQUE_LIMIT
		/* perform torque limit */
		rpm = N2RPM(cur_value);
		if( rpm < 0 ) rpm = -rpm;
		max_output_for_torque = calc_max_output( pid->torque_limit, rpm, bat_v);

		/* torque limit */
		if( pid->out > max_output_for_torque )
			pid->out = max_output_for_torque;
		else if( pid->out < -max_output_for_torque )
			pid->out = -max_output_for_torque;
	#endif

	return pid->out;
}

