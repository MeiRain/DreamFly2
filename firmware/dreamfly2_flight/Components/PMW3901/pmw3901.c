#include "Delay.h"
#include "SPI.h"
#include "USART.h"
#include <stdio.h>
#include "PMW3901MB.h"
#include "PMW3901MB_Register.h"

static uint8_t PMW3901MB_ReadByte(uint8_t reg_address)
{
	return SPI1_ReadByte(reg_address);
}

static uint8_t PMW3901MB_WriteByte(uint8_t reg_address, uint8_t byte)
{
	return SPI1_WriteByte(reg_address, byte);
}

static uint8_t PMW3901MB_ReadBuffer(uint8_t reg_address, uint8_t *read_pbuffer, uint8_t read_size)
{
	return SPI1_ReadBuffer(reg_address, read_pbuffer, read_size);
}

static uint8_t PMW3901MB_WriteBuffer(uint8_t reg_address, uint8_t *write_pbuffer, uint8_t write_size)
{
	return SPI1_WriteBuffer(reg_address, write_pbuffer, write_size);
}

uint8_t PMW3901MB_Check(void)
{
	printf("0x%02x\r\n", PMW3901MB_ReadByte(PRODUCT_ID));
	printf("0x%02x\r\n", PMW3901MB_ReadByte(INVERSE_PRODUCT_ID));
	if (PMW3901MB_ReadByte(PRODUCT_ID) == ~(PMW3901MB_ReadByte(INVERSE_PRODUCT_ID)))
		return 1;
	else
		return 0;
}

void PMW3901MB_Init(void)
{
	uint8_t C1, C2;
	
	PMW3901MB_Check();
	PMW3901MB_WriteByte(POWER_UP_RESET, 0x5A);// 上电复位
	Delay_ms(100);
	
	PMW3901MB_ReadByte(0x02);
	PMW3901MB_ReadByte(0x03);
	PMW3901MB_ReadByte(0x04);
	PMW3901MB_ReadByte(0x05);
	PMW3901MB_ReadByte(0x06);

	//配置寄存器
	PMW3901MB_WriteByte(0x7F, 0x00);
	PMW3901MB_WriteByte(0x55, 0x01);
	PMW3901MB_WriteByte(0x50, 0x07);
	PMW3901MB_WriteByte(0x7F, 0x0E);
	PMW3901MB_WriteByte(0x43, 0x10);
	if (PMW3901MB_ReadByte(0x47) != 0x08)
	{
		//循环读取三次
		uint8_t i;
		for(i = 0; i < 3; i++)
		{
			PMW3901MB_WriteByte(0x43, 0x10);
			if (PMW3901MB_ReadByte(0x47) == 0x08)
				break;
		}
		if(i == 3)
			return;
	}
	PMW3901MB_WriteByte(0x7F, 0x00);
	PMW3901MB_WriteByte(0x51, 0x7B);
	PMW3901MB_WriteByte(0x50, 0x00);
	PMW3901MB_WriteByte(0x55, 0x00);
	PMW3901MB_WriteByte(0x7F, 0x0E);
	if (PMW3901MB_ReadByte(0x73) == 0x00)
	{
		C1 = PMW3901MB_ReadByte(0x70);
		C2 = PMW3901MB_ReadByte(0x71);
		if (C1 >= 0 && C1 <= 28)
			C1+=14;
		else if(C1 > 28 && C1 <= 0x3F)
			C1+=11;
		else
			C1 = 0x3F;
		C2 = (C2 * 45)/100;
		PMW3901MB_WriteByte(0x7F, 0x00);
		PMW3901MB_WriteByte(0x61, 0xAD);
		PMW3901MB_WriteByte(0x7F, 0x0E);
		PMW3901MB_WriteByte(0x70, C1);
		PMW3901MB_WriteByte(0x71, C2);
	}
	PMW3901MB_WriteByte(0x7F, 0x00);
	PMW3901MB_WriteByte(0x61, 0xAD);
	printf("0x%02x\r\n", PMW3901MB_ReadByte(0x61));
	PMW3901MB_WriteByte(0x7F, 0x03);
	PMW3901MB_WriteByte(0x40, 0x00);
	PMW3901MB_WriteByte(0x7F, 0x05);
	PMW3901MB_WriteByte(0x41, 0xB3);
	PMW3901MB_WriteByte(0x43, 0xF1);
	PMW3901MB_WriteByte(0x45, 0x14);
	PMW3901MB_WriteByte(0x5B, 0x32);
	PMW3901MB_WriteByte(0x5F, 0x34);
	PMW3901MB_WriteByte(0x7B, 0x08);
	PMW3901MB_WriteByte(0x7F, 0x06);
	PMW3901MB_WriteByte(0x44, 0x1B);
	PMW3901MB_WriteByte(0x40, 0xBF);
	PMW3901MB_WriteByte(0x4E, 0x3F);
	PMW3901MB_WriteByte(0x7F, 0x08);
	PMW3901MB_WriteByte(0x65, 0x20);
	PMW3901MB_WriteByte(0x6A, 0x18);
	PMW3901MB_WriteByte(0x7F, 0x09);
	PMW3901MB_WriteByte(0x4F, 0xAF);
	PMW3901MB_WriteByte(0x5F, 0x40);
	PMW3901MB_WriteByte(0x48, 0x80);
	PMW3901MB_WriteByte(0x49, 0x80);
	PMW3901MB_WriteByte(0x57, 0x77);
	PMW3901MB_WriteByte(0x60, 0x78);
	PMW3901MB_WriteByte(0x61, 0x78);
	PMW3901MB_WriteByte(0x62, 0x08);
	PMW3901MB_WriteByte(0x63, 0x50);
	PMW3901MB_WriteByte(0x7F, 0x0A);
	PMW3901MB_WriteByte(0x45, 0x60);
	PMW3901MB_WriteByte(0x7F, 0x00);
	PMW3901MB_WriteByte(0x4D, 0x11);
	PMW3901MB_WriteByte(0x55, 0x80);
	PMW3901MB_WriteByte(0x74, 0x1F);
	PMW3901MB_WriteByte(0x75, 0x1F);
	PMW3901MB_WriteByte(0x4A, 0x78);
	PMW3901MB_WriteByte(0x4B, 0x78);
	PMW3901MB_WriteByte(0x44, 0x08);
	PMW3901MB_WriteByte(0x45, 0x50);
	PMW3901MB_WriteByte(0x64, 0xFF);
	PMW3901MB_WriteByte(0x65, 0x1F);
	PMW3901MB_WriteByte(0x7F, 0x14);
	PMW3901MB_WriteByte(0x65, 0x67);
	PMW3901MB_WriteByte(0x66, 0x08);
	PMW3901MB_WriteByte(0x63, 0x70);
	PMW3901MB_WriteByte(0x7F, 0x15);
	PMW3901MB_WriteByte(0x48, 0x48);
	PMW3901MB_WriteByte(0x7F, 0x07);
	PMW3901MB_WriteByte(0x41, 0x0D);
	PMW3901MB_WriteByte(0x43, 0x14);
	PMW3901MB_WriteByte(0x4B, 0x0E);
	PMW3901MB_WriteByte(0x45, 0x0F);
	PMW3901MB_WriteByte(0x44, 0x42);
	PMW3901MB_WriteByte(0x4C, 0x80);
	PMW3901MB_WriteByte(0x7F, 0x10);
	PMW3901MB_WriteByte(0x5B, 0x02);
	PMW3901MB_WriteByte(0x7F, 0x07);
	PMW3901MB_WriteByte(0x40, 0x41);
	PMW3901MB_WriteByte(0x70, 0x00);
	Delay_ms(10);
	PMW3901MB_WriteByte(0x32, 0x44);
	PMW3901MB_WriteByte(0x7F, 0x07);
	PMW3901MB_WriteByte(0x40, 0x40);
	PMW3901MB_WriteByte(0x7F, 0x06);
	PMW3901MB_WriteByte(0x62, 0xF0);
	PMW3901MB_WriteByte(0x63, 0x00);
	PMW3901MB_WriteByte(0x7F, 0x0D);
	PMW3901MB_WriteByte(0x48, 0xC0);
	PMW3901MB_WriteByte(0x6F, 0xD5);
	PMW3901MB_WriteByte(0x7F, 0x00);
	PMW3901MB_WriteByte(0x5B, 0xA0);
	PMW3901MB_WriteByte(0x4E, 0xA8);
	PMW3901MB_WriteByte(0x5A, 0x50);
	PMW3901MB_WriteByte(0x40, 0x80);
}

void PMW3901MB_GetRawData(int16_t *delta_x, int16_t *delta_y)
{
//	uint8_t Motion;		
//	uint8_t Observation;	
	uint8_t Delta_X_L;
	uint8_t Delta_X_H;
	uint8_t Delta_Y_L;
	uint8_t Delta_Y_H;
	uint8_t Squal;
//	uint8_t RawData_Sum;
//	uint8_t Maximum_RawData;
//	uint8_t Minimum_RawData;
	uint8_t Shutter_Upper;
//	uint8_t Shutter_Lower;
	uint8_t Buffer[12];	
	
	PMW3901MB_ReadBuffer(MOTION_BURST, Buffer, sizeof(Buffer));
//	Motion = Buffer[0];
//	Observation = Buffer[1];
	Delta_X_L = Buffer[2];
	Delta_X_H = Buffer[3];
	Delta_Y_L = Buffer[4];
	Delta_Y_H = Buffer[5];
	Squal = Buffer[6];
//	RawData_Sum = Buffer[7];
//	Maximum_RawData = Buffer[8];
//	Minimum_RawData = Buffer[9];
	Shutter_Upper = Buffer[10];
//	Shutter_Lower = Buffer[11];

	if ((Squal < 0x19) && (Shutter_Upper == 0x1F))
	{
		printf("failed\r\n");
		return;
	}
	else
	{
//		printf("success\r\n");
		*delta_x = (int16_t)((Buffer[3] << 8)| Buffer[2]);
		*delta_y = (int16_t)((Buffer[5] << 8)| Buffer[4]);
//		printf("%d, %d\r\n", *delta_x, *delta_y);
	}

//	spiBeginTransaction();
//	
//	NCS_PIN = 0;
//	
//	delay_us(50);	
//	spiExchange(1, &MOTION_BURST, &MOTION_BURST);
//	delay_us(50);
//	spiExchange(sizeof(motionBurst_t), (uint8_t*)motion, (uint8_t*)motion);	
//	delay_us(50);
//	
//	NCS_PIN = 1;
//	
//	spiEndTransaction();
//	delay_us(50);

//	uint16_t realShutter = (motion->shutter >> 8) & 0x0FF;
//	realShutter |= (motion->shutter & 0x0ff) << 8;
//	motion->shutter = realShutter;
}
