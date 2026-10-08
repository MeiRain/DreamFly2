#ifndef __AK09916_H
#define __AK09916_H	

#include "stm32f4xx.h"

// AK09916原始数据
typedef struct
{
	int16_t MagX;
	int16_t MagY;
	int16_t MagZ;
} _ak_raw;

// AK09916原始数据
typedef struct
{
	float MagX;
	float MagY;
	float MagZ;
} _ak_physics;

void AK09916_Check(void);
void AK09916_Init(void);
uint8_t AK09916_ReadMag(int16_t *mx, int16_t *my, int16_t *mz);
void AK09916_GetData(void);
void AK09916_ReadByte(uint8_t reg_address, uint8_t *pByte);
void AK09916_WriteByte(uint8_t reg_address, uint8_t Byte);
void AK09916_ReadBuffer(uint8_t reg_address, uint8_t *pBuffer, uint16_t NumByteToRead);
void AK09916_WriteBuffer(uint8_t reg_address, uint8_t *pBuffer, uint16_t NumByteToWrite);

#endif
