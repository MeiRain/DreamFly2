#ifndef __BMP280_H
#define __BMP280_H

#include "stm32f4xx.h"

//BMP温度过采样因子
typedef enum {
	T_OVERSAMPLING_NONE = 0x0,	/*skipped*/
	T_OVERSAMPLING_1X,			/*x1*/
	T_OVERSAMPLING_2X,			/*x2*/
	T_OVERSAMPLING_4X,			/*x4*/
	T_OVERSAMPLING_8X,			/*x8*/
	T_OVERSAMPLING_16X			    /*x16*/
} OSRS_T_e;

//BMP压力过采样因子，枚举类型会从前一个值递增
typedef enum
{
	P_OVERSAMPLING_NONE = 0x0,	/*skipped*/
	P_OVERSAMPLING_1X,			/*x1*/
	P_OVERSAMPLING_2X,			/*x2*/
	P_OVERSAMPLING_4X,			/*x4*/
	P_OVERSAMPLING_8X,			/*x8*/
	P_OVERSAMPLING_16X			/*x16*/
} OSRS_P_e;

//BMP工作模式
typedef enum {
	SLEEP_MODE = 0x0,
	FORCED_MODE = 0x1,	//可以说0x2
	NORMAL_MODE = 0x3
} Mode_e;

//保持时间
typedef enum {
	T_SB1 = 0x0,	    /*0.5ms*/
	T_SB2,			/*62.5ms*/
	T_SB3,			/*125ms*/
	T_SB4,			/*250ms*/
	T_SB5,			/*500ms*/
	T_SB6,			/*1000ms*/
	T_SB7,			/*2000ms*/
	T_SB8			/*4000ms*/
} T_SB_e;

//IIR滤波器时间常数
typedef enum {
	FILTER_NONE = 0x0,	/*filter off*/
	FILTER_MODE1,		/*0.223*ODR*/	/*x2*/
	FILTER_MODE2,		/*0.092*ODR*/	/*x4*/
	FILTER_MODE3,		/*0.042*ODR*/	/*x8*/
	FILTER_MODE4		/*0.021*ODR*/	/*x16*/
} Filter_e;

typedef struct  
{
	/* T1~P9 为补偿系数 */
	uint16_t T1;
	int16_t	T2;
	int16_t	T3;
	uint16_t P1;
	int16_t	P2;
	int16_t	P3;
	int16_t	P4;
	int16_t	P5;
	int16_t	P6;
	int16_t	P7;
	int16_t	P8;
	int16_t	P9;
} bmp280_t;

typedef struct
{
	OSRS_T_e T_Oversampling;
	OSRS_P_e P_Oversampling;
	Mode_e	 Work_Mode;
	
	Filter_e 	Filter_Coefficient;
	T_SB_e 		Time_Standby;
	FunctionalState		SPI_EN;
} bmp280_init_t;

typedef struct
{
	float Presssure;
	float Temperature;
	float Altitude;
} bmp_t;

uint8_t BMP280_Check(void);
void BMP280_Init(void);
double BMP280_GetTemperature(void);
double BMP280_GetPressure(void);
void BMP280_GetRawData(bmp_t *bmp280_raw);

#endif
