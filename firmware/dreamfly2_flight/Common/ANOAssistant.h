#ifndef __ANOASSISTANT_H
#define __ANOASSISTANT_H

#include "stm32f4xx.h"

typedef struct 
{
	uint8_t Frame0x01;
	uint8_t Frame0x02;
	uint8_t Frame0x03;
	uint8_t Frame0x04;
	uint8_t Frame0x05;
	uint8_t Frame0x06;
	uint8_t Frame0x0D;
} anosendflag_t;

void ANO_SendFrame0x01(int16_t acc_x, int16_t acc_y, int16_t acc_z, int16_t gyr_x, int16_t gyr_y, int16_t gyr_z, uint8_t shock_sta);
void ANO_SendFrame0x02(int16_t mag_x, int16_t mag_y, int16_t mag_z, int16_t tmp, uint8_t mag_sta);
void ANO_SendFrame0x03(float rol, float pit, float yaw, uint8_t fusion_sta);
void ANO_SendFrame0x05(int32_t alt_bar, int32_t alt_add, int32_t alt_fu, uint8_t alt_sta);
void ANO_SendFrame0x06(uint8_t mode, uint8_t sflag, uint8_t cid, uint8_t cmd0, uint8_t cmd01);
void ANO_SendFrame0x0D(uint8_t voltage, uint8_t current);
void ANOSendData_Remote(int16_t rol, int16_t pit, int16_t thr, int16_t yaw, int16_t aux1,
					    int16_t aux2, int16_t aux3, int16_t aux4, int16_t aux5, int16_t aux6);

#endif
