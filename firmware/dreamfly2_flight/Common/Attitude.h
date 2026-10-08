#ifndef __ATTITUDE_H
#define __ATTITUDE_H

#include "stm32f4xx.h"

#define RADTODEG 57.295780f			   // 弧度到角度转换因子 = [(1 rad * 180) / π]
#define DEGTORAD 0.0174533f			   // 角度到弧度转换因子 = [(1 degree * π) / 180]
#define ACC_GRA_SCALEFACTOR 0.0012044f // 加速度计转换重力单位因子 = [(2 * 4 / 65536) * 9.80665]
#define GYR_RAD_SCALEFACTOR 0.0010653f // 陀螺仪弧度制单位因子 = [(π / 180) * (2 * 2000 / 65536)]
#define GYR_DEG_SCALEFACTOR 0.0610351f // 陀螺仪角度制单位因子 = (2 * 2000 / 65536)

typedef struct
{
	float ROL;
	float PIT;
	float YAW;
} angle_t;

void Attitude_Update(int16_t *acc_x, int16_t *acc_y, int16_t *acc_z, 
					 int16_t *gyr_x, int16_t *gyr_y, int16_t *gyr_z, angle_t* angle);
float invSqrt(float x);

#endif
