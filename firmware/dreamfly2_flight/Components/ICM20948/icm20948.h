//
// Created by 梅雨 on 2026/10/2.
//

#ifndef __ICM20948_H
#define __ICM20948_H	

#include "i2c.h"

typedef struct
{
	int16_t AccX;
	int16_t AccY;
	int16_t AccZ;
	int16_t GyrX;
	int16_t GyrY;
	int16_t GyrZ;
} icm_t;

uint8_t ICM20948_Check(void);
void ICM20948_Init(void);
void ICM20948_GetAccel(int16_t *ax, int16_t *ay, int16_t *az);
void ICM20948_GetGyro(int16_t *gx, int16_t *gy, int16_t *gz);
void ICM20948_WriteByte(uint8_t reg_address, uint8_t Byte);
void ICM20948_ReadByte(uint8_t reg_address, uint8_t* pByte);
void ICM20948_ReadBuffer(uint8_t reg_address, uint8_t* pBuffer, uint16_t NumByteToRead);
void ICM20948_SelectBank(uint8_t bank);

#endif
