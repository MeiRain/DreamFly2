#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f4xx.h"

typedef struct
{
	int16_t PWM1;
	int16_t PWM2;	
	int16_t PWM3;
	int16_t PWM4;
} motor_t;

void Motor_Init(void);
void Motor_PWMLimit(int16_t *motorx_pwm);
void Motor_Control(motor_t* motor);

#endif
