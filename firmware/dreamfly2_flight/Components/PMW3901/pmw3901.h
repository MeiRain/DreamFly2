#ifndef __PMW3901MB_H
#define __PMW3901MB_H

#include "stm32f4xx.h"

typedef struct 
{
	uint8_t Motion;		
	uint8_t Observation;		
	uint8_t Delta_X_L;
	uint8_t Delta_X_H;
	uint8_t Delta_Y_L;
	uint8_t Delta_Y_H;
	uint8_t Squal;
	uint8_t RawData_Sum;
	uint8_t Maximum_RawData;
	uint8_t Minimum_RawData;
	uint8_t Shutter_Upper;
	uint8_t Shutter_Lower;
} motionburst_t;

typedef struct  
{
	int16_t Delata_X;
	int16_t Delata_Y;
	int16_t POS_X;
	int16_t POX_Y;
	
//	float pixSum[2];		/*累积像素*/
//	float pixComp[2];		/*像素补偿*/
//	float pixValid[2];		/*有效像素*/
//	float pixValidLast[2];	/*上一次有效像素*/
//	
//	float deltaPos[2];		/*2帧之间的位移 单位cm*/
//	float deltaVel[2];		/*速度 单位cm/s*/
//	float posSum[2];		/*累积位移 单位cm*/
//	float velLpf[2];		/*速度低通 单位cm/s*/
//	
//	uint8_t isOpFlowOk;		/*光流状态*/
//	uint8_t isDataValid;	/*数据有效*/
} pmw3901mb_t;

void PMW3901MB_Init(void);
void PMW3901MB_GetRawData(int16_t *delta_x, int16_t *delta_y);

#endif
