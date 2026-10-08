#include "TIM.h"
#include "Motor.h"

/************************代码移植修改区************************************/

#define MOTOR_PWMMIN 0
#define MOTOR_PWMMAX 900

#define MOTOR1_CLK 		RCC_AHB1Periph_GPIOB 	// 端口时钟
#define MOTOR1_PORT 	GPIOB					// 端口
#define MOTOR1_PIN 		GPIO_Pin_6				// 引脚
#define MOTOR1_AF 		GPIO_PinSource6			// 复用引脚

#define MOTOR2_CLK 		RCC_AHB1Periph_GPIOB 	// 端口时钟
#define MOTOR2_PORT 	GPIOB					// 端口
#define MOTOR2_PIN 		GPIO_Pin_7				// 引脚
#define MOTOR2_AF 		GPIO_PinSource7			// 复用引脚

#define MOTOR3_CLK 		RCC_AHB1Periph_GPIOB 	// 端口时钟
#define MOTOR3_PORT 	GPIOB					// 端口
#define MOTOR3_PIN 		GPIO_Pin_0				// 引脚
#define MOTOR3_AF 		GPIO_PinSource0			// 复用引脚

#define MOTOR4_CLK 		RCC_AHB1Periph_GPIOB 	// 端口时钟
#define MOTOR4_PORT 	GPIOB					// 端口
#define MOTOR4_PIN 		GPIO_Pin_1				// 引脚
#define MOTOR4_AF		GPIO_PinSource1			// 复用引脚

/**************************************************************************/

/******************************************************************************************
 *函  数：void Motor_Init(void)
 *功  能：TIM3_PWM 和 TIM4_PWM 输出通道的 GPIO 设置，PWM 初始化
 *参  数：无
 *返回值：无
 *备  注：TIM4_CH1(Motor1) -> PB6
 *        TIM4_CH2(Motor2) -> PB7
 *        TIM3_CH3(Motor3) -> PB0
 *        TIM3_CH4(Motor4) -> PB1
 *		  TIMx_ARR  	决定方波的周期
 *        TIMx_CCRx	决定方波的占空比
 *******************************************************************************************/
void Motor_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(MOTOR1_CLK | MOTOR2_CLK | MOTOR3_CLK | MOTOR4_CLK, ENABLE);

	// 连接 TIM3 的通道到 AF2
	GPIO_PinAFConfig(MOTOR1_PORT, MOTOR1_AF, GPIO_AF_TIM3);
	GPIO_PinAFConfig(MOTOR2_PORT, MOTOR2_AF, GPIO_AF_TIM3);
	GPIO_PinAFConfig(MOTOR3_PORT, MOTOR3_AF, GPIO_AF_TIM4);
	GPIO_PinAFConfig(MOTOR4_PORT, MOTOR4_AF, GPIO_AF_TIM4);

	GPIO_InitStructure.GPIO_Pin = MOTOR1_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	   // 受外设控制的引脚，需要配置为复用模式
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; // 速度100M
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	   // 推完输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	   // 上拉输入
	GPIO_Init(MOTOR1_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = MOTOR2_PIN;
	GPIO_Init(MOTOR2_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = MOTOR3_PIN;
	GPIO_Init(MOTOR3_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = MOTOR4_PIN;
	GPIO_Init(MOTOR4_PORT, &GPIO_InitStructure);
}

void Motor_PWMLimit(int16_t *motor_pwmx)
{
	*motor_pwmx = (*motor_pwmx <= MOTOR_PWMMIN) ? MOTOR_PWMMIN : *motor_pwmx;
	*motor_pwmx = (*motor_pwmx >= MOTOR_PWMMAX) ? MOTOR_PWMMAX : *motor_pwmx;
}

/*****************************************************************************
 *函  数：void Motor_PWM_Config(int16_t motor1_pwm, int16_t motor2_pwm, int16_t motor3_pwm, int16_t motor4_pwm)
 *功  能：电机要输出数值转换成PWM波形输出
 *参  数：MOTOR1 电机1
 *        MOTOR2 电机2
 *        MOTOR3 电机3
 *        MOTOR3 电机4
 *返回值：无
 *备  注：无
 *****************************************************************************/
void Motor_Control(motor_t* motor)
{
	Motor_PWMLimit(&motor->PWM1);
	Motor_PWMLimit(&motor->PWM2);
	Motor_PWMLimit(&motor->PWM3);
	Motor_PWMLimit(&motor->PWM4);

	TIM4->CCR1 = motor->PWM1;
	TIM4->CCR2 = motor->PWM2;
	TIM3->CCR3 = motor->PWM3;
	TIM3->CCR4 = motor->PWM4;
}
