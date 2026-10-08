#include "PID.h"

/*****************************************************************************
*函  数：void PidParameter_init(void)
*功  能：初始化PID结构体里的一些成员值
*参  数：无
*返回值：无 
*备  注: 由于PID参数我们都保存在FLASH中，所以此函数初始化时不用初始化这些参数，
*        但是Flash中的参数有可能因为误操作被擦除，如果Flash读取参数失败，则就
*        初始化为默认参数。
*****************************************************************************/
void PID_Rest(pid_t** pid, uint8_t len)
{
	for(uint8_t i = 0; i < len; i++)
	{
	  	pid[i]->Integral = 0;
	    pid[i]->PreError = 0;
	    pid[i]->Out = 0;
	}
}

/*****************************************************************************
*函  数：void PID_Postion_Cal(PID_TYPE*PID,float target,float measure)
*功  能：位置式PID算法
*参  数：PID: 算法P I D参数的结构体
*        target: 目标值
*        measure: 测量值 
*返回值：无 
*备  注: 角度环和角速度环共用此函数
*****************************************************************************/
void PID_Control(pid_t* pid, uint8_t integral_flag, float dt)
{
	//计算偏差值
	pid->Error = pid->Target - pid->Measure;
	
	//计算微分：偏差变化率
	pid->Differ = (pid->Error - pid->PreError) / dt;
	
	if(integral_flag == 1)    //飞机解锁之后再加入积分,防止积分过调Airplane_Enable == 1&&RC_Control.THROTTLE >= 180
	{
		if((pid->Measure) < (pid->Ilimit) && (pid->Measure) > -(pid->Ilimit))   //积分分离
		{
			pid->Integral += pid->Error * dt;                  //计算积分：偏差累计和
			
			/* 进行积分限幅 */
			if(pid->Integral > pid->Irang)                     
				pid->Integral = pid->Irang;
			if(pid->Integral < -pid->Irang)
			    pid->Integral = -pid->Irang;                    
		}
	}
	else
	{
		pid->Integral = 0;
	}
	
	//结果保存，比例 + 积分 + 微分
	pid->Out =  pid->Ki * pid->Integral + 
	            pid->Kp * pid->Error + pid->Kd * pid->Differ;	
	pid->PreError = pid->Error;
}

void PID_CascadeControl(pid_t *pidangle, pid_t *pidrate, uint8_t integral_flag, float dt)
{
	//角度外环进行 PID 处理
	PID_Control(pidangle, integral_flag, dt);
	
	//外环输出，赋值给内环期望值
	pidrate->Target = pidangle->Out;
	
	//对内环进行 PID 计算
	PID_Control(pidrate, integral_flag, dt);
}

/*****************************************************************************
*函  数：void PidParameter_init(void)
*功  能：初始化PID结构体里的一些成员值
*参  数：无
*返回值：无 
*备  注: 由于PID参数我们都保存在FLASH中，所以此函数初始化时不用初始化这些参数，
*        但是Flash中的参数有可能因为误操作被擦除，如果Flash读取参数失败，则就
*        初始化为默认参数。
*****************************************************************************/
void PidParameter_init(void)
{
	
}
