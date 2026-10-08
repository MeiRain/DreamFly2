#include <math.h>

#include "Attitude.h"

#define KP 0.8f						   // 比例增益控制加速度计，磁力计的收敛速率
#define KI 0.0003f					   // 积分增益控制陀螺偏差的收敛速度，Kp = Ki = 0 就是完全相信陀螺仪
#define HALFT 0.005f				   // 解算周期的一半时间

/*********************************************************************************************************
 *函  数：void IMU_GetAngle(FLOAT_XYZ *Gyr_rad, FLOAT_XYZ *Acc_filt, FLOAT_ANGLE *g_ATT_Angle)
 *功　能：获取姿态角
 *参  数：Gyr_rad 	指向角速度的指针（注意单位必须是弧度）
 *        Acc_filt 	指向加速度的指针
 *        g_ATT_Angle 	指向姿态角的指针
 *返回值：无
 *备  注：求解四元数和欧拉角都在此函数中完成，此函数是通过绕 Z->Y->X 得到的欧拉角描述的余弦矩阵为例
 **********************************************************************************************************/
void Attitude_Update(int16_t *acc_x, int16_t *acc_y, int16_t *acc_z, 
					 int16_t *gyr_x, int16_t *gyr_y, int16_t *gyr_z, angle_t* angle)
{
	static float q0 = 1, q1 = 0, q2 = 0, q3 = 0;  // 四元数初始值，一般设为1、0、0、0
	static float exInt = 0, eyInt = 0, ezInt = 0; // 标度积分误差（初始化为0）
	
	float ax = *acc_x, ay = *acc_y, az = *acc_z; // 读出加速度计数据
	float gx = *gyr_x, gy = *gyr_y, gz = *gyr_z; // 读出陀螺仪数据
	float vx, vy, vz;																		  // 机体坐标系下的重力加速度向量
	float ex, ey, ez;																		  // 理论和实际重力加速度的误差向量
	float norm;																				  // 通用单位化变量

	float q0q0 = q0 * q0;
	float q0q1 = q0 * q1;
	float q0q2 = q0 * q2;
	// float q0q3 = q0 * q3;
	float q1q1 = q1 * q1;
	// float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q3q3 = q3 * q3;

	if (ax * ay * az == 0)
		return;

	gx *= GYR_RAD_SCALEFACTOR;
	gy *= GYR_RAD_SCALEFACTOR;
	gz *= GYR_RAD_SCALEFACTOR;

	// 1.加速度计测量机体坐标系下的实际重力单位向量
	norm = invSqrt(ax * ax + ay * ay + az * az);
	ax = ax * norm;
	ay = ay * norm;
	az = az * norm;

	// 2.提取四元数的等效余弦矩阵中的重力分量（利用四元数将地理重力加速度旋转至机体坐标系）
	vx = 2 * (q1q3 - q0q2);
	vy = 2 * (q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3;

	// 3.向量叉积得出姿态误差
	// 叉积误差是指将带有误差的加计向量转动到与重力向量重合，需要绕什么轴，转多少角度。
	// 叉积在机体三轴上的投影，就是加计和重力之间的角度误差。
	// 如果陀螺按这个叉积误差的轴，转动叉积误差的角度（也就是转动三轴投影的角度）那就能把加计和重力向量的误差消除掉。
	// 如果完全按叉积误差转过去，那就是完全信任加计。如果一点也不转，那就是完全信任陀螺。（具体可看向量叉积的定义）
	// 那么把这个叉积的三轴乘以 Ki，加到陀螺的积分角度上去，就是这个 Ki 互补系数的互补算法了。
	ex = (ay * vz - az * vy);
	ey = (az * vx - ax * vz);
	ez = (ax * vy - ay * vx);

	// 4.对误差进行积分
	exInt = exInt + ex * KI;
	eyInt = eyInt + ey * KI;
	ezInt = ezInt + ez * KI;

	// 5.互补滤波，姿态误差补偿到角速度上，修正角速度积分漂移
	// 这里的 gz 由于没有观测者进行矫正会产生漂移，表现出来的就是积分自增或自减
	gx = gx + KP * ex + exInt;
	gy = gy + KP * ey + eyInt;
	gz = gz + KP * ez + ezInt; 

	// 6.用准确的陀螺仪数据通过一阶龙格库塔法解四元素的微分方程
	q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * HALFT;
	q1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * HALFT;
	q2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * HALFT;
	q3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * HALFT;

	// 7.单位化四元数
	norm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 = q0 * norm;
	q1 = q1 * norm;
	q2 = q2 * norm;
	q3 = q3 * norm;

	// 8.四元数转换成欧拉角（Z->Y->X）
	float G_yaw = (*gyr_z) * GYR_DEG_SCALEFACTOR; // 将Z轴角速度陀螺仪值 转换为Z角度/秒

	if ((G_yaw > 1.0f) || (G_yaw < -1.0f)) // 数据太小可以认为是干扰，不是偏航动作
	{
		angle->YAW += G_yaw * HALFT; // yaw，直接对角速度积分，因此实际结果会逐渐偏移
	}

	angle->PIT = -asin(2.f * (q1q3 - q0q2)) * RADTODEG;								   // pitch（负号要注意）
	angle->ROL = atan2(2.f * q2q3 + 2.f * q0q1, q0q0 - q1q1 - q2q2 + q3q3) * RADTODEG; // roll
}

/**************************实现函数*********************************************************************
函  数：static float invSqrt(float x)
功　能: 快速计算 1/Sqrt(x)
参  数：要计算的值
返回值：结果
备  注：比普通Sqrt()函数要快四倍
*********************************************************************************************************/
float invSqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long *)&y;
	i = 0x5f3759df - (i >> 1);
	y = *(float *)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}
