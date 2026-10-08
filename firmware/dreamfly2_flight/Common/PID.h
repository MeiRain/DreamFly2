#ifndef __PID_H
#define __PID_H

#include "stm32f4xx.h"

// PID算法的数据结构
typedef struct 
{
	float Kp; // 参数
	float Ki;
	float Kd;
	float Target;
	float Measure;
	float Error;	// 比例项，保存本次偏差
	float Integral; // 积分项，保存偏差累计和
	float Differ;	// 微分项，保存偏差变化率
	float PreError;	// 保存上次偏差值
	float Ilimit;	//积分分离
	float Irang;	//积分限幅
	float Out;
} pid_t;

void PID_CascadeControl(pid_t *pidangle, pid_t *pidrate, uint8_t integral_flag, float dt);

#endif
