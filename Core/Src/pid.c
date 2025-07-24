/********************************************************************************
* 权限:    杭州南江机器人股份有限公司
* 文件名:    pid.c          
* 功能描述:    
           pid控制算法
* 版本      作者             时间          状态
* V1.0      chenshouxian     2015.09.09    创建文件
*****************************************************************************
*****************************************************************************/ 

#include "arm_math.h"
#include "cfg.h"
#include "pid.h"
#include "misc.h"
#include "robot.h"

GYRO_PID_STRUCT gyro_pid = {0};

/*******************************************************************************
* Function Name   : calc_max_output
* Description	  : 计算最大输出
*					
* Input 		  : float max_torque, long speed, float bat_v
* Output		  : None
* Return		  : None.
******************************************************************************/
long calc_max_output( float max_torque, long speed, float bat_v )
{
	float Vin = max_torque / MOTOR_KI * MOTOR_R + (float)speed / MOTOR_KV;

	return MOTOR_PWM_PERIOD * Vin / bat_v;
}


/*******************************************************************************
* Function Name   : pid_init
* Description	  : pid参数计算初始化 增量式pid u(k)=u(k-1)+deta_u(k)
*					deat_u(k)=(kp+ki+kd)*e(k)-(kp+2kd)*e(k-1)+kd*e(k-2)
* Input 		  : pid_t *pid, float Kp, float Ki, float Kd 
* Output		  : None
* Return		  : None.
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

/******************************************************************************
 * pid_set_point: 
 *
 * Parameter:
 * Input: 
 * Output: 
 * Returns: 
 * 
 * modification history
 * --------------------
 *    2015/09/09, Shouxian Chen create this function
 * 
 ******************************************************************************/
void pid_set_point(pid_t *pid, int set_value )
{
	pid->set = set_value;
}

/******************************************************************************
 * pid_set_limit: 
 *
 * Parameter:
 * Input: 
 * Output: 
 * Returns: 
 * 
 * modification history
 * --------------------
 *    2015/09/09, Shouxian Chen create this function
 * 
 ******************************************************************************/
void pid_set_limit(pid_t *pid, int limit )
{
	pid->limit = limit;
}

/*******************************************************************************
* Function Name   : pid_set_torque_limit
* Description	  : pid参数计算初始化 
*		
* Input 		  : pid_t *pid, float limit 
* Output		  : None
* Return		  : None.
******************************************************************************/
void pid_set_torque_limit(pid_t *pid, float limit )
{
	pid->torque_limit = limit;
}


/*******************************************************************************
* Function Name   : pid_step
* Description	  : pid控制计算 增量式pid u(k)=u(k-1)+deta_u(k)
*					deat_u(k)=(kp+ki+kd)*e(k)-(kp+2kd)*e(k-1)+kd*e(k-2)
* Input 		  : pid_t *pid, int cur_value, float bat_v
* Output		  : None
* Return		  : pid控制器输出.
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


/*******************************************************************************
* Function Name   : gyro_pid_init
* Description	  : 陀螺仪闭环控制pid参数计算初始化 增量式pid u(k)=u(k-1)+deta_u(k)
*					deat_u(k)=(kp+ki+kd)*e(k)-(kp+2kd)*e(k-1)+kd*e(k-2)
* Input 		  : pid_t *pid, float Kp, float Ki, float Kd 
* Output		  : None
* Return		  : None.
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
* Function Name   : gyro_control_calcuate
* Description	  : pid控制计算 增量式pid u(k)=u(k-1)+deta_u(k)
*					deat_u(k)=(kp+ki+kd)*e(k)-(kp+2kd)*e(k-1)+kd*e(k-2)
* Input 		  : pid_t *pid, int cur_value, float bat_v
* Output		  : None
* Return		  : pid控制器输出.
*******************************************************************************/
int gyro_control_calcuate(GYRO_PID_STRUCT *pid)
{

	float d_out;
  
	/* standard digital PID algorithm */
	pid->e3 = pid->e2;
	pid->e2 = pid->e1;
	pid->e1 = pid->set_value - pid->feedback_speed;

	d_out = pid->A * pid->e1 - pid->B * pid->e2 + pid->C * pid->e3;
  
	pid->out = pid->out + d_out;
	
	/* output limit */
	if( pid->out > pid->limit )
		pid->out = pid->limit;
	else if( pid->out < -pid->limit )
		pid->out = -pid->limit;
  

	return pid->out;
}

/*******************************************************************************
* Function Name   : pid_get_output
* Description	  : 
* Input 		  : pid_t *pid
* Output		  : None
* Return		  : None.
******************************************************************************/
int pid_get_output(pid_t *pid)
{
	return pid->out;
}

/*******************************************************************************
* Function Name   : pid_set_param
* Description	  : pid参数计算初始化 增量式pid u(k)=u(k-1)+deta_u(k)
*					deat_u(k)=(kp+ki+kd)*e(k)-(kp+2kd)*e(k-1)+kd*e(k-2)
* Input 		  : pid_t *pid, float Kp, float Ki, float Kd 
* Output		  : None
* Return		  : None.
******************************************************************************/
void pid_set_param(pid_t *pid, float Kp, float Ki, float Kd )
{
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;

	pid->A = Kp + Ki + Kd;
	pid->B = Kp + 2 * Kd;
	pid->C = Kd;
}

/*******************************************************************************
* Function Name   : pid_reinit
* Description	  : pid参数计算初始化 
*		
* Input 		  : pid_t *pid
* Output		  : None
* Return		  : None.
******************************************************************************/
void pid_reinit(pid_t *pid)
{
	pid->e1 = 0;
	pid->e2 = 0;
	pid->e3 = 0;
	pid->set = 0;
}



//单神经元自适应pid尝试 wyq 2024.5
void u_que_init(u_que*k, int l)
{
	int i = 0;
	k->idx = 0;
	k->length = l;
	for(i = 0; i < l; i++)
		k->data[i] = 0;
	k->outtmp = 0;
}

void update_que(u_que*k, float x)
{
	k->outtmp = k->data[k->idx];
	k->data[k->idx] = x;
	k->idx = (k->idx+1)%4;
}

void netpid_init(net_pid* k, float alpha1, float alpha2, float limit)
{
	int i = 4;
	k->alpha1 = alpha1;
	k->alpha2 = alpha2;
	
	u_que_init(&k->u1, 4);
	u_que_init(&k->u2, 4);
	u_que_init(&k->u3, 4);
	
	u_que_init(&k->out, 4);
	u_que_init(&k->net, 4);
	
	k->i1 = 0;
	k->i2 = 0;
	k->i3 = 0;
	
	k->limit = limit;
	k->flag = 0;
	
	k->layer1[0] = 0.09;
	k->layer1[1] = 0.0005;
	k->layer1[2] = 0;
	
	k->layer1[3] = -0.09;
	for(i = 4; i<9; i++)
		k->layer1[i] = 0;
	for(i = 0; i<3; i++)
		k->layer2[i] = 1;
	k->pid_out = 0;
}

// 计算pid输出
int pid_output(net_pid* k, float error, int cur_value)
{
	float u1, u2, u3;
	//float o1;
	long rpm = 0; 
	long max_output_for_torque = 0;
	
	k->i3 = k->i2;
	k->i2 = k->i1;
	k->i1 = error;
	
	u1 = k->layer1[0] * k->i1 + k->layer1[3] * k->i2 + k->layer1[6] * k->i3;
	u2 = k->layer1[1] * k->i1 + k->layer1[4] * k->i2 + k->layer1[7] * k->i3;
	u3 = k->layer1[2] * k->i1 + k->layer1[5] * k->i2 + k->layer1[8] * k->i3;
	
	update_que(&k->u1, u1);
	update_que(&k->u2, u2);
	update_que(&k->u3, u3);
	
	k->pid_out += k->layer2[0] * u1 + k->layer2[1] * u2 + k->layer2[2] * u3;
	
	update_que(&k->net, k->pid_out);
	if(k->pid_out > k->limit)
		k->pid_out = k->limit;
	if(k->pid_out < -k->limit)
		k->pid_out = -k->limit;
	update_que(&k->out, k->pid_out);
	//if(k->flag < 4) k->flag++;
	
#ifdef ENABLE_TORQUE_LIMIT
	/* perform torque limit */
	rpm = N2RPM(cur_value);
	if( rpm < 0 ) rpm = -rpm;
	max_output_for_torque = calc_max_output( MOTOR_TORQUE_LIMIT, rpm, 16.0f);

	/* torque limit */
	if( k->pid_out > max_output_for_torque )
		k->pid_out = max_output_for_torque;
	else if( k->pid_out < -max_output_for_torque )
		k->pid_out = -max_output_for_torque;
#endif
	return k->pid_out;
}

// 激活函数导数计算
float calc_f_grad(float limit, float output)
{
	if(output >= 0)
		return output/(2*limit);
	else
		return -output/(2*limit);
}

// 判断梯度下降方向，避免差分出现0，用符号函数激活
int jud_grad(float a, float b)
{
	if(a*b > 0) return 1;
	if(0 == a*b) return 0;
	if(a*b < 0) return -1;
}

// 反向传播更新网络权值 wyq 2024.5
void net_update(net_pid* k, u_que*error)
{
	
	float delta[3] = {0}, tmp, tmp_layer1[9] = {0};
	int i = 0, j = 0, idx[7];
	//if(k->flag < 4) return;
/*idx_last_u1 = k->u1->idx, idx_last_u2 = k->u2->idx, 
idx_last_u3 = k->u3->idx, idx_last_o = k->out->idx, idx_last_e = error->idx, 
idx_last_n = k->net->idx;*/
	
	idx[0] = k->u1.idx, idx[1] = k->u2.idx, idx[2] = k->u3.idx, idx[3] = k->out.idx, 
	idx[4] = error->idx, idx[5] = k->net.idx, idx[6] = (error->idx+4)%6; 
	
	for(i = 0; i < 4; i++)
	{
		tmp = (error->data[idx[4]] * calc_f_grad(k->limit, k->out.data[idx[3]]) * 
			jud_grad(fabs(error->data[idx[4]]) -  fabs(error->data[(idx[4]+5)%6]), 
			fabs(k->out.data[idx[3]]) - fabs(k->out.data[(idx[3]+3)%4])))/4;
		delta[0] += tmp*k->u1.data[idx[0]];
		delta[1] += tmp*k->u2.data[idx[1]];
		delta[2] += tmp*k->u3.data[idx[2]];
		
		tmp_layer1[0] += tmp*k->layer2[0]*error->data[idx[6]];
		tmp_layer1[1] += tmp*k->layer2[1]*error->data[idx[6]];
		tmp_layer1[2] += tmp*k->layer2[2]*error->data[idx[6]];
		
		tmp_layer1[3] += tmp*k->layer2[0]*error->data[(idx[6]+5)%6];
		tmp_layer1[4] += tmp*k->layer2[1]*error->data[(idx[6]+5)%6];
		tmp_layer1[5] += tmp*k->layer2[2]*error->data[(idx[6]+5)%6];
		
		tmp_layer1[6] += tmp*k->layer2[0]*error->data[(idx[6]+4)%6];
		tmp_layer1[7] += tmp*k->layer2[1]*error->data[(idx[6]+4)%6];
		tmp_layer1[8] += tmp*k->layer2[2]*error->data[(idx[6]+4)%6];
		
		for(j = 0; j<6; j++) 
			idx[j] = (idx[j]+3)%4;
		idx[6] = (idx[6]+5)%6;
	}
	
	for(i = 0; i<3; i++) k->layer2[i] -= k->alpha1*delta[i];
	for(j = 0; j<9; j++) k->layer1[j] -= k->alpha2*tmp_layer1[j];
}
