#include "I2C.h"
#include "USART.h"
#include <stdio.h>
#include <math.h>
#include "BMP280.h"
#include "BMP280_Register.h"

typedef			long signed int				BMP280_S32_t;	//有符号 32位！
typedef			long unsigned int			BMP280_U32_t;	//无符号 32位！
typedef			long long signed int		BMP280_S64_t;

static bmp280_t g_BMP280;

static BMP280_S32_t g_T_Fine;			//用于计算补偿

#define dig_T1 g_BMP280.T1  
#define dig_T2 g_BMP280.T2  
#define dig_T3 g_BMP280.T3  
#define dig_P1 g_BMP280.P1  
#define dig_P2 g_BMP280.P2  
#define dig_P3 g_BMP280.P3  
#define dig_P4 g_BMP280.P4  
#define dig_P5 g_BMP280.P5  
#define dig_P6 g_BMP280.P6  
#define dig_P7 g_BMP280.P7  
#define dig_P8 g_BMP280.P8  
#define dig_P9 g_BMP280.P9  
 
 /**************************传感器值转定点值*************************************/

#ifdef USE_FIXED_POINT_COMPENSATE
// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC. 
// g_T_Fine carries fine temperature as global value
static BMP280_S32_t BMP280_CompensateTemperature_int32(BMP280_S32_t adc_t)
{
	BMP280_S32_t Var1, Var2, T;
	Var1 = ((((adc_t>>3) - ((BMP280_S32_t)dig_T1<<1))) * ((BMP280_S32_t)dig_T2)) >> 11;
	Var2 = (((((adc_t>>4) - ((BMP280_S32_t)dig_T1)) * ((adc_t>>4) - ((BMP280_S32_t)dig_T1))) >> 12) * 
	((BMP280_S32_t)dig_T3)) >> 14;
	g_T_Fine = Var1 + Var2;
	T = (g_T_Fine * 5 + 128) >> 8;
	return T;
}

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
static BMP280_U32_t BMP280_CompensatePressure_int64(BMP280_S32_t adc_p)
{
	BMP280_S64_t Var1, Var2, P;
	Var1 = ((BMP280_S64_t)g_T_Fine) - 128000;
	Var2 = Var1 * Var1 * (BMP280_S64_t)dig_P6;
	Var2 = Var2 + ((Var1*(BMP280_S64_t)dig_P5)<<17);
	Var2 = Var2 + (((BMP280_S64_t)dig_P4)<<35);
	Var1 = ((Var1 * Var1 * (BMP280_S64_t)dig_P3)>>8) + ((Var1 * (BMP280_S64_t)dig_P2)<<12);
	Var1 = (((((BMP280_S64_t)1)<<47)+Var1))*((BMP280_S64_t)dig_P1)>>33;
	if (Var1 == 0)
	{
	return 0; // avoid exception caused by division by zero
	}
	P = 1048576-adc_p;
	P = (((P<<31)-Var2)*3125)/Var1;
	Var1 = (((BMP280_S64_t)dig_P9) * (P>>13) * (P>>13)) >> 25;
	Var2 = (((BMP280_S64_t)dig_P8) * P) >> 19;
	P = ((P + Var1 + Var2) >> 8) + (((BMP280_S64_t)dig_P7)<<4);
	return (BMP280_U32_t)P;
}

#else
/**************************传感器值转定点值*************************************/
// Returns temperature in DegC, double precision. Output value of “51.23” equals 51.23 DegC.
// g_T_Fine carries fine temperature as global value
static double BMP280_CompensateTemperature_double(BMP280_S32_t adc_t)
{
	double Var1, Var2, T;
	Var1 = (((double)adc_t)/16384.0 - ((double)dig_T1)/1024.0) * ((double)dig_T2);
	Var2 = ((((double)adc_t)/131072.0 - ((double)dig_T1)/8192.0) *
	(((double)adc_t)/131072.0 - ((double) dig_T1)/8192.0)) * ((double)dig_T3);
	g_T_Fine = (BMP280_S32_t)(Var1 + Var2);
	T = (Var1 + Var2) / 5120.0;
	return T;
}

// Returns pressure in Pa as double. Output value of “96386.2” equals 96386.2 Pa = 963.862 hPa
static double BMP280_CompensatePressure_double(BMP280_S32_t adc_p)
{
	double Var1, Var2, P;
	Var1 = ((double)g_T_Fine/2.0) - 64000.0;
	Var2 = Var1 * Var1 * ((double)dig_P6) / 32768.0;
	Var2 = Var2 + Var1 * ((double)dig_P5) * 2.0;
	Var2 = (Var2/4.0)+(((double)dig_P4) * 65536.0);
	Var1 = (((double)dig_P3) * Var1 * Var1 / 524288.0 + ((double)dig_P2) * Var1) / 524288.0;
	Var1 = (1.0 + Var1 / 32768.0)*((double)dig_P1);
	if (Var1 == 0.0)
	{
		return 0; // avoid exception caused by division by zero
	}
	P = 1048576.0 - (double)adc_p;
	P = (P - (Var2 / 4096.0)) * 6250.0 / Var1;
	Var1 = ((double)dig_P9) * P * P / 2147483648.0;
	Var2 = P * ((double)dig_P8) / 32768.0;
	P = P + (Var1 + Var2 + ((double)dig_P7)) / 16.0;
	return P;
}
#endif
 
/*****************************************************************************
 *函  数：uint8_t BMP280_ReadByte(uint8_t reg, uint8_t *buf)
 *功  能：从指定BMP280寄存器读取一个字节数据
 *参  数：reg： 寄存器地址
 *        buf:  读取数据存放的地址
 *返回值：0成功 1失败
 *备  注：BMP280代码移植只需把IIC驱动修改成自己的即可
*****************************************************************************/
static uint8_t BMP280_ReadByte(uint8_t reg_address, uint8_t* pByte)
{
 	if (I2C1_ReadByte(BMP280_ADDR, reg_address, pByte))
		return 1;
	else
		return 0;
}

/*****************************************************************************
 *函  数：uint8_t BMP280_WriteByte(uint8_t reg, uint8_t data)
 *功  能：写一个字节数据到 BMP280 寄存器
 *参  数：reg：寄存器地址
 *        data: 要写入的数据
 *返回值：0成功 1失败
 *备  注：BMP280代码移植只需把IIC驱动修改成自己的即可
*****************************************************************************/
static uint8_t BMP280_WriteByte(uint8_t reg_address, uint8_t Byte)
{
	if (I2C1_WriteByte(BMP280_ADDR, reg_address, Byte))
		return 1;
	else
		return 0;
}

static void BMP280_RegisterInit(bmp280_init_t *bmp280_initstructure)
{
	uint8_t RegValue;
	RegValue = ((bmp280_initstructure->T_Oversampling)<<5)|
			 ((bmp280_initstructure->P_Oversampling)<<2)|
			 ((bmp280_initstructure)->Work_Mode);
	BMP280_WriteByte(CONTROL_MEAS, RegValue);
	
	RegValue = ((bmp280_initstructure->Time_Standby)<<5)|
			 ((bmp280_initstructure->Filter_Coefficient)<<2)|
			 ((bmp280_initstructure->SPI_EN));
	BMP280_WriteByte(CONFIG, RegValue);
}

static uint8_t BMP280_GetStatus(uint8_t status_flag)
{
	uint8_t Flag;
	BMP280_ReadByte(STATUS, &Flag);
	if(Flag & status_flag)
		return SET;
	else 
		return RESET;
}

uint8_t BMP280_Check(void)
{
 	uint8_t DeviceID;
 	BMP280_ReadByte(ID, &DeviceID);
	if (DeviceID == BMP280_DEVID)
		return 1;
	else
		return 0;
}

void BMP280_Init(void)
{
	uint8_t LSB, MSB; 
	
 	//获取温度校准系数  
 	BMP280_ReadByte(DIG_T1_LSB, &LSB);
 	BMP280_ReadByte(DIG_T1_MSB, &MSB);
 	g_BMP280.T1 = MSB << 8 | LSB;
 	BMP280_ReadByte(DIG_T2_LSB, &LSB);
 	BMP280_ReadByte(DIG_T2_MSB, &MSB);
 	g_BMP280.T2 = MSB << 8 | LSB;  
 	BMP280_ReadByte(DIG_T3_LSB, &LSB);
 	BMP280_ReadByte(DIG_T3_MSB, &MSB);
 	g_BMP280.T3 = MSB << 8 | LSB;  
	
 	//获取气压校准系数  
 	BMP280_ReadByte(DIG_P1_LSB, &LSB);
 	BMP280_ReadByte(DIG_P1_MSB, &MSB);    
 	g_BMP280.P1 = MSB << 8 | LSB;  
 	BMP280_ReadByte(DIG_P2_LSB, &LSB);
 	BMP280_ReadByte(DIG_P2_MSB, &MSB);      
 	g_BMP280.P2 = MSB << 8 | LSB;  
 	BMP280_ReadByte(DIG_P3_LSB, &LSB);
 	BMP280_ReadByte(DIG_P3_MSB, &MSB);  
 	g_BMP280.P3 = MSB << 8 | LSB;  
 	BMP280_ReadByte(DIG_P4_LSB, &LSB);
 	BMP280_ReadByte(DIG_P4_MSB, &MSB);         
 	g_BMP280.P4 = MSB << 8 | LSB;    
 	BMP280_ReadByte(DIG_P5_LSB, &LSB);
 	BMP280_ReadByte(DIG_P5_MSB, &MSB);           
 	g_BMP280.P5 = MSB << 8 | LSB;  
 	BMP280_ReadByte(DIG_P6_LSB, &LSB);
 	BMP280_ReadByte(DIG_P6_MSB, &MSB);          
 	g_BMP280.P6 = MSB << 8 | LSB;  
 	BMP280_ReadByte(DIG_P7_LSB, &LSB);
 	BMP280_ReadByte(DIG_P7_MSB, &MSB);           
 	g_BMP280.P7 = MSB << 8 | LSB;  
 	BMP280_ReadByte(DIG_P8_LSB, &LSB);
 	BMP280_ReadByte(DIG_P8_MSB, &MSB);         
 	g_BMP280.P8 = MSB << 8 | LSB;  
 	BMP280_ReadByte(DIG_P9_LSB, &LSB);
 	BMP280_ReadByte(DIG_P9_MSB, &MSB);            
	g_BMP280.P9 = MSB << 8 | LSB; 
//	printf("%d,%d,%d,%d,%d,%d,%d",dig_P1,dig_P2,dig_P3,dig_P4,dig_P5,dig_P6,dig_P7);
	
	BMP280_WriteByte(RESET, BMP280_RESET_VALUE);
	
	bmp280_init_t BMP280_InitStructure;
	BMP280_InitStructure.T_Oversampling = T_OVERSAMPLING_1X;//0x001;
	BMP280_InitStructure.P_Oversampling = P_OVERSAMPLING_4X;//0x011;
	BMP280_InitStructure.Work_Mode = NORMAL_MODE;//0x11;
	BMP280_InitStructure.Time_Standby = T_SB1;//0x000;
	BMP280_InitStructure.Filter_Coefficient = FILTER_MODE4;//0x100;
	BMP280_InitStructure.SPI_EN = DISABLE;//0x00
	
	BMP280_RegisterInit(&BMP280_InitStructure);	
}

//24个整数位，八个小数位
double BMP280_GetTemperature(void)
{
	uint8_t LSB, MSB, XLSB;  
	long signed ADC_T;
	double temperature;
		
	BMP280_ReadByte(TEMP_XLSB, &XLSB);
	BMP280_ReadByte(TEMP_LSB, &LSB);
	BMP280_ReadByte(TEMP_MSB, &MSB);
	ADC_T = ((long)MSB << 12) | ((long)LSB << 4) | (XLSB >> 4); 
	temperature = BMP280_CompensateTemperature_double(ADC_T);     
	return temperature;
}

double BMP280_GetPressure(void)
{  
	uint8_t XLSB, LSB, MSB;  
	long signed ADC_P;
	double pressure;
	
	BMP280_ReadByte(PRESS_XLSB, &XLSB);
	BMP280_ReadByte(PRESS_LSB, &LSB);
	BMP280_ReadByte(PRESS_MSB, &MSB);
	ADC_P = ((long)MSB << 12) | ((long)LSB << 4) | (XLSB >> 4);
	pressure = BMP280_CompensatePressure_double(ADC_P);
	return pressure;
}

void BMP280_GetRawData(bmp_t *bmp280_raw)
{
	if (BMP280_GetStatus(BMP280_MEASURING) != RESET)
	{
		if(BMP280_GetStatus(BMP280_IM_UPDATE) != RESET)
		{
			bmp280_raw->Temperature = BMP280_GetTemperature();
			bmp280_raw->Presssure = BMP280_GetPressure();
			bmp280_raw->Altitude = 4433000.0f * (1.0f - powf((bmp280_raw->Presssure) / 101325.0f, 0.190295f));//使用 powf 而非 pow 是因为 powf 专用于单精度浮点数运算，速度更快。
			printf("\r\nTemperature %f C Pressure %f Pa",bmp280_raw->Temperature,bmp280_raw->Presssure);
			printf("\r\nAltitude %f cm",bmp280_raw->Altitude);
		}
	}
	else
		return;
}

//void BMP280CalAvgValue(uint8_t *pIndex, int32_t *pAvgBuffer, int32_t InVal, int32_t *pOutVal)
//{ 
//	uint8_t i;

//	*(pAvgBuffer + ((*pIndex) ++)) = InVal;
//	*pIndex &= 0x07;

//	*pOutVal = 0;
//	for(i = 0; i < 8; i ++)
//	{
//		*pOutVal += *(pAvgBuffer + i);
//	}
//	*pOutVal >>= 3;
//}

//void BMP280_GetData(int32_t *ps32Temperature, int32_t *ps32Pressure, int32_t *ps32Altitude)
//{
//	float CurTemperature, CurPressure;
//	int32_t CurAltitude;
//	static BMP280_AvgTypeDef BMP280_Filter[3];

//	BMP280_GetTemperature(&CurTemperature);
//	BMP280_GetPressure(&CurPressure);
//	BMP280_Calculate_AbsoluteAltitude(&CurAltitude, (*ps32Pressure));

//	BMP280CalAvgValue(&BMP280_Filter[2].Index, BMP280_Filter[2].AvgBuffer, (int32_t)CurTemperature, ps32Temperature);
//	BMP280CalAvgValue(&BMP280_Filter[0].Index, BMP280_Filter[0].AvgBuffer, (int32_t)CurPressure, ps32Pressure);	
//	BMP280CalAvgValue(&BMP280_Filter[1].Index, BMP280_Filter[1].AvgBuffer, CurAltitude, ps32Altitude);


//	printf("\r\nPressure: %.2f     Altitude: %.2f \r\n",(float)ps32Pressure / 100, (float)ps32Altitude);
//	printf("\r\nTemperature: %.1f \r\n", (float)ps32Temperature / 100);
//	printf("CurTemperature:%d\r\n", ps32Temperature);
//	printf("CurPressure:%d\r\n", ps32Pressure);
//	printf("AbsoluteAltitude:%d\r\n", ps32Altitude);
//}
