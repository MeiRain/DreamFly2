#include "Delay.h"
#include "I2C.h"

#include "ICM20948.h"
#include "ICM20948_Register.h"
#include "AK09916.h"
#include "AK09916_Register.h"

uint8_t AK09916_CheckFlag = 0;
_ak_raw AK09916_Raw;
_ak_physics AK09916_Physics;

void AK09916_Check(void)
{
	uint8_t ID;

	AK09916_ReadByte(WIA2, &ID);

	if (ID == AK09916_DEVID)
	{
		AK09916_CheckFlag = 1;
//		printf("AK09916 Connect Success!\r\n\r\n");
	}
	else
	{
//		printf("id:%x\r\n\r\n",ID);
//		printf("AK09916 Not Connect!\r\n\r\n");
		while (1)
		{
//			WS2812B_Orange();
			Delay_ms(10);
		}
	}
}

void AK09916_Init(void)
{
	AK09916_Check();

	AK09916_WriteByte(CNTL3, 0x01); // 复位 AK09916
	Delay_ms(100);
	AK09916_WriteByte(CNTL2, BIT_CM_MODE_4); // 设置为连续测量模式4，100Hz
}

uint8_t AK09916_ReadMag(int16_t *mx, int16_t *my, int16_t *mz)
{
	uint8_t Ready_Flag, OverFlow_Flag;
	uint8_t Buffer[6];

	AK09916_ReadByte(ST1, &Ready_Flag);
	Ready_Flag = Ready_Flag & 0x01;

	if (!Ready_Flag)
	{
		// printf("AK09916 Data is not ready!\r\n\r\n");
		return 0;
	}
	else
	{
		AK09916_ReadBuffer(HXL, Buffer, 6);
		AK09916_ReadByte(ST2, &OverFlow_Flag);
		OverFlow_Flag = OverFlow_Flag & 0x08;

		if (OverFlow_Flag)
		{
			// printf("AK09916 Data is overflow!\r\n\r\n");
			return 0;
		}
		else
		{
			*mx = (int16_t)(Buffer[1] << 8 | Buffer[0]);
			*my = (int16_t)(Buffer[3] << 8 | Buffer[2]);
			*mz = (int16_t)(Buffer[5] << 8 | Buffer[4]);
			return 1;
		}
	}
}

void AK09916_GetData(void)
{
	if (AK09916_ReadMag(&AK09916_Raw.MagX, &AK09916_Raw.MagY, &AK09916_Raw.MagZ))
	{
//		printf("=== AK Raw TEST ===\r\n");
//		printf(" Raw MAGX: %d\r\n", AK09916_Raw.MagX);
//		printf(" Raw MAGY: %d\r\n", AK09916_Raw.MagY);
//		printf(" Raw MAGZ: %d\r\n", AK09916_Raw.MagZ);
//		printf("===================\r\n\r\n");

		AK09916_Physics.MagX = (float)AK09916_Raw.MagX * 0.15f;
		AK09916_Physics.MagY = (float)AK09916_Raw.MagY * 0.15f;
		AK09916_Physics.MagZ = (float)AK09916_Raw.MagZ * 0.15f;

		// printf("=== AK Physics TEST ===\r\n");
		// printf(" Physics MAGX: %0.2f\r\n", AK09916_Physics.MagX);
		// printf(" Physics MAGY: %0.2f\r\n", AK09916_Physics.MagY);
		// printf(" Physics MAGZ: %0.2f\r\n", AK09916_Physics.MagZ);
		// printf("=======================\r\n\r\n");
	}
	else
		return;
}

void AK09916_ReadByte(uint8_t reg_address, uint8_t *pByte)
{
	ICM20948_SelectBank(USER_BANK_3); // 选择 USER BANK 3 Register
	ICM20948_WriteByte(B3_I2C_SLV0_ADDR, AK09916_ADDR | AK09916_READ);
	ICM20948_WriteByte(B3_I2C_SLV0_REG, reg_address);
	ICM20948_WriteByte(B3_I2C_SLV0_CTRL, 0x81);

	Delay_ms(1);
	ICM20948_SelectBank(USER_BANK_0); // 选择 USER BANK 0 Register
	ICM20948_ReadByte(B0_EXT_SLV_SENS_DATA_00, pByte);
}

void AK09916_WriteByte(uint8_t reg_address, uint8_t Byte)
{
	ICM20948_SelectBank(USER_BANK_3); // 选择 USER BANK 0 Register
	ICM20948_WriteByte(B3_I2C_SLV0_ADDR, AK09916_ADDR | AK09916_WRITE
	);
	ICM20948_WriteByte(B3_I2C_SLV0_REG, reg_address);
	ICM20948_WriteByte(B3_I2C_SLV0_DO, Byte);
	ICM20948_WriteByte(B3_I2C_SLV0_CTRL, 0x81);
}

void AK09916_ReadBuffer(uint8_t reg_address, uint8_t *pBuffer, uint16_t NumByteToRead)
{
	ICM20948_SelectBank(USER_BANK_3); // 选择 USER BANK 0 Register
	ICM20948_WriteByte(B3_I2C_SLV0_ADDR, AK09916_ADDR | AK09916_READ);
	ICM20948_WriteByte(B3_I2C_SLV0_REG, reg_address);
	ICM20948_WriteByte(B3_I2C_SLV0_CTRL, 0x80 | NumByteToRead);

	Delay_ms(1);
	ICM20948_SelectBank(USER_BANK_0); // 选择 USER BANK 0 Register
	ICM20948_ReadBuffer(B0_EXT_SLV_SENS_DATA_00, pBuffer, NumByteToRead);
}

void AK09916_WriteBuffer(uint8_t reg_address, uint8_t *pBuffer, uint16_t NumByteToWrite)
{
	I2C1_WriteBuffer(AK09916_ADDR, reg_address, pBuffer, NumByteToWrite);
}
