#ifndef __SI24R1_H
#define __SI24R1_H	

#include "stm32f4xx.h"

#define MODE_RX1				1				//普通接收模式
#define MODE_TX1				2				//普通发送模式
#define MODE_RX2				3				//接收模式，用于双向传输
#define MODE_TX2				4				//发送模式，用于双向传输
#define TX_ADR_WIDTH 			5				//发送地址宽度
#define RX_ADR_WIDTH 			5				//接收地址宽度
#define TX_PAYLOAD_WIDTH 		32				//发送数据宽度
#define RX_PAYLOAD_WIDTH 		32				//接收数据宽度

uint8_t Si24R1_Check(void);
void Si24R1_Init(uint8_t Mode);
void Si24R1_SetMode(uint8_t Mode);
void Si24R1_NormalPacket(uint8_t* txbuffer, uint8_t length);
void Si24R1_AckPayloadPacket(uint8_t* txbuffer, uint8_t length);
void Si24R1_IRQHandler(uint8_t *rxbuffer, uint16_t *errorcount);
uint8_t Si24R1_ReadByte(uint8_t reg_address);
uint8_t Si24R1_WriteByte(uint8_t reg_address, uint8_t Byte);
uint8_t Si24R1_ReadBuffer(uint8_t reg_address, uint8_t* pBuffer, uint8_t NumByteToRead);
uint8_t Si24R1_WriteBuffer(uint8_t reg_address, uint8_t* pBuffer, uint8_t NumByteToWrite);	
	
#endif
