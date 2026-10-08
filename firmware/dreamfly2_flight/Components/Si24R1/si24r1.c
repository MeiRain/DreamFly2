#include "SPI.h"

#include "Si24R1.h"
#include "Si24R1_Register.h"

/**********************************************代码移植修改区**********************************************/

#define Si24R1_CE_RCC    		RCC_AHB1Periph_GPIOA  								//端口时钟
#define Si24R1_CE_PORT   		GPIOA                 								//端口
#define Si24R1_CE_PIN        	GPIO_Pin_8            								//引脚
										
#define Si24R1_IRQ_RCC   		RCC_AHB1Periph_GPIOB  								//端口时钟
#define Si24R1_IRQ_PORT  		GPIOB                 								//端口
#define Si24R1_IRQ_PIN       	GPIO_Pin_2            								//引脚

#define Si24R1_CE_LOW    		Si24R1_CE_PORT->BSRRH  |= Si24R1_CE_PIN				// CE 引脚低电平
#define Si24R1_CE_HIGH   		Si24R1_CE_PORT->BSRRL  |= Si24R1_CE_PIN				// CE 引脚高电平

/**********************************************************************************************************/

uint8_t TX_ADDRESS[TX_ADR_WIDTH] = {0x34,0x43,0x10,0x10,0xFF};			//Si24R1 发送地址
uint8_t RX_ADDRESS[RX_ADR_WIDTH] = {0x34,0x43,0x10,0x10,0xFF}; 			//Si24R1 接收地址		

/*************************************************************************
*函  数：uint8_t Si24R1_InitCheck(void)
*功  能：检查 Si24R1 与 MCU 的 SPI 总线是否通信正常
*参  数：无
*返回值：0 未连接；1 已连接 
*备  注：此函数通过写入发送地址并读取地址，禁止在初始化后调用！！！
**************************************************************************/
uint8_t Si24R1_Check(void)
{
	uint8_t i = 0;
	uint8_t Buffer[5] = {0xA8, 0xA8, 0xA8, 0xA8, 0xA8};	 
	Si24R1_WriteBuffer(W_REGISTER + TX_ADDR, Buffer, 5); 				//写入5个字节的地址	
	Si24R1_ReadBuffer(TX_ADDR, Buffer, 5); 								//读出写入的地址  
	for (i = 0; i < 5; i ++)
	{
		if (Buffer[i] != 0xA8)
			break;	 		
	}		
	if (i == 5)
		return 1;
	else
		return 0;		
}

/*************************************************************************
*函  数：void Si24R1_Init(void)
*功  能：Si24R1 引脚 GPIO 初始化
*参  数：无
*返回值：无
*备  注：中断产生时，IRQ 引脚会被拉低，所以这里要配置成上拉输入
**************************************************************************/
void Si24R1_Init(uint8_t Mode)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(Si24R1_CE_RCC | Si24R1_IRQ_RCC, ENABLE);
		
	//配置 Si24R1 的使能端 CE
	GPIO_InitStructure.GPIO_Pin = Si24R1_CE_PIN; 			 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;       
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(Si24R1_CE_PORT, &GPIO_InitStructure); 
	GPIO_ResetBits(Si24R1_CE_PORT, Si24R1_CE_PIN);
	
	//配置 Si24R1 的中断引脚 IRQ
	GPIO_InitStructure.GPIO_Pin = Si24R1_IRQ_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(Si24R1_IRQ_PORT, &GPIO_InitStructure);
	
	Si24R1_SetMode(Mode);
}

void Si24R1_SetMode(uint8_t Mode)
{
	Si24R1_CE_LOW;

	Si24R1_WriteBuffer(W_REGISTER + TX_ADDR, (uint8_t*)TX_ADDRESS, TX_ADR_WIDTH); 		//写 TX节点 的地址 TX_ADDR（发送与接收地址需要一致）
	Si24R1_WriteBuffer(W_REGISTER + RX_ADDR_P0, (uint8_t*)RX_ADDRESS, RX_ADR_WIDTH);	//写 RX节点 的地址（主要是为了使能 Auto Ack） RX_ADDR_P0
	Si24R1_WriteByte(W_REGISTER + EN_AA, 0x01); 										//使能 AUTO ACK EN_AA
	Si24R1_WriteByte(W_REGISTER + EN_RXADDR, 0x01);										//使能 PIPE 0 EN_RXADDR
	Si24R1_WriteByte(W_REGISTER + SETUP_RETR, 0x1A);									//配置自动重发次数 SETUP_RETR
	Si24R1_WriteByte(W_REGISTER + RF_CH, 0x48);	                                    	//选择通信频率 RF_CH
	Si24R1_WriteByte(W_REGISTER + RF_SETUP, 0x0F); 										//配置发射参数（低噪放大器增益、发射功率、无线速率） RF_SETUP								
                                                                              
	if (Mode == 1)																		//普通 RX模式
	{
		Si24R1_WriteByte(W_REGISTER + RX_PW_P0, RX_PAYLOAD_WIDTH);						//选择通道 0 有效数据宽度 RX_PW_P0	
		Si24R1_WriteByte(W_REGISTER + CONFIG, 0x0F);   									//IRQ 收发完成中断开启，16位 CRC，主接收
	}
	
	else if (Mode == 2)																	//普通 TX模式
	{
		Si24R1_WriteByte(W_REGISTER + RX_PW_P0, RX_PAYLOAD_WIDTH);						//选择通道 0 有效数据宽度 RX_PW_P0	 
		Si24R1_WriteByte(W_REGISTER + CONFIG, 0x0E);   		 							// IRQ收发完成中断开启，16位CRC，主发送
	}
	
	else if (Mode == 3)																	//发送带有负载数据的 ACK信号 的 RX模式
	{		
		Si24R1_WriteByte(W_REGISTER + DYNPD, 0x01);										//使能接收 管道0 动态负载长度
		Si24R1_WriteByte(W_REGISTER + FEATURE, 0x06);                                   //使能动态负载长度，使能 ACK负载（带负载数据的 ACK包）
		Si24R1_WriteByte(W_REGISTER + CONFIG, 0x0F);   		 							// IRQ收发完成中断开启，16位CRC，主接收		
	}
	
	else if (Mode == 4)																	//发送带有负载数据的 ACK信号 的 TX模式
	{		
		Si24R1_WriteByte(W_REGISTER + DYNPD, 0x01);										//使能接收管道0 动态负载长度
		Si24R1_WriteByte(W_REGISTER + FEATURE, 0x06);									//使能动态负载长度，使能 ACK负载（带负载数据的 ACK包）
		Si24R1_WriteByte(W_REGISTER + CONFIG, 0x0E);   		 							// IRQ收发完成中断开启，16位CRC，主发送
	}
	
	Si24R1_CE_HIGH;
}

void Si24R1_NormalPacket(uint8_t* txbuffer, uint8_t length)
{
	Si24R1_CE_LOW;																		//CE 拉低，使能Si24R1配置
	Si24R1_WriteBuffer(W_TX_PAYLOAD, txbuffer, length);	   								//写数据到 TX FIFO ，32 个字节
	Si24R1_CE_HIGH;																		//CE 置高，使能发送	   																
}

void Si24R1_AckPayloadPacket(uint8_t* txbuffer, uint8_t length)
{
	Si24R1_CE_LOW;		 																//StandBy I模式	
	Si24R1_WriteBuffer(W_ACK_PLAYLOAD, txbuffer, length); 								//装载数据
	Si24R1_CE_HIGH;		 																//置高CE
}

void Si24R1_IRQHandler(uint8_t *rxbuffer, uint16_t *errorcount)
{
	uint8_t state = Si24R1_ReadByte(STATUS);  											//读取状态寄存器的值
	Si24R1_WriteByte(W_REGISTER + STATUS, state);               						//清除中断标志
	
	if (state & BIT_RX_DR)																//判断是否收到数据
	{
		uint8_t rx_len = Si24R1_ReadByte(R_RX_PL_WID);       							//读出接收字节数
		
		if (rx_len > 0 && rx_len < 33)																//判断是否超出接收字节限制
		{
			*errorcount = 0;
			Si24R1_ReadBuffer(R_RX_PAYLOAD, rxbuffer, rx_len); 					//未超出，接收数据
			Si24R1_WriteByte(FLUSH_RX, 0xFF); 											//清除RX FIFO
//			printf("== RX TEST AREA ==\r\n");
//			for(uint8_t i = 0; i < rx_len; i ++)
//			{
//				printf(" RXDATA[%d] = 0x%02x\r\n", i, rxbuffer[i]);
//			}
//			printf("==================\r\n\r\n");
		
		}
		else
		{
			Si24R1_WriteByte(FLUSH_RX, 0xFF);											//超出，清除 RX缓冲区，丢弃数据					
		}
	}

	if (state & BIT_TX_DS)
	{
//		printf("Send Success!\r\n\r\n");
	}

	if (state & BIT_MAX_RT)																//判断是否达到最大重发次数
	{
		if (state & BIT_TX_FULL)														//TX FIFO 满，清除 TX缓冲区
		{
			Si24R1_WriteByte(FLUSH_TX, 0xFF);		
		}
//		printf("Send Fail!\r\n\r\n");
	}
}

/*************************************************************************
*函  数：uint8_t Si24R1_ReadByte(uint8_t reg_address)
*功  能：从寄存器读一字节数据
*参  数：reg_address：寄存器地址
*返回值：status
*备  注：Si24R1 代码移植只需把 SPI 驱动修改成自己的即可
**************************************************************************/
uint8_t Si24R1_ReadByte(uint8_t reg_address)
{
	return SPI2_ReadByte(reg_address);
}

/*************************************************************************
*函  数：uint8_t Si24R1_WriteByte(uint8_t reg_address, uint8_t Byte)
*功  能：写一字节数据到寄存器
*参  数：reg_address：寄存器地址
*        Byte	   ：要写入的数据
*返回值：status
*备  注：Si24R1 代码移植只需把 SPI 驱动修改成自己的即可
**************************************************************************/
uint8_t Si24R1_WriteByte(uint8_t reg_address, uint8_t Byte)
{
	return SPI2_WriteByte(reg_address, Byte);
}

/***************************************************************************************************
*函  数：uint8_t Si24R1_ReadBuffer(uint8_t reg_address, uint8_t* pBuffer, uint8_t NumByteToRead)
*功  能：从寄存器读一组数据
*参  数：reg_address	  ：寄存器地址
*        pBuffer	  ：要读取数据的地址
*        NumByteToRead: 要读取的数据长度
*返回值：status
*备  注：Si24R1 代码移植只需把 SPI 驱动修改成自己的即可
****************************************************************************************************/
uint8_t Si24R1_ReadBuffer(uint8_t reg_address, uint8_t* pBuffer, uint8_t NumByteToRead)
{
	return SPI2_ReadBuffer(reg_address, pBuffer, NumByteToRead);
}

/***************************************************************************************************
*函  数：uint8_t Si24R1_WriteBuffer(uint8_t reg_address, uint8_t* pBuffer, uint8_t NumByteToWrite)
*功  能：写一组数据到寄存器
*参  数：reg_address    ：寄存器地址
*        pBuffer       ：要写入数据的地址
*        NumByteToWrite: 要写入的数据长度
*返回值：status
*备  注：Si24R1 代码移植只需把 SPI 驱动修改成自己的即可
****************************************************************************************************/
uint8_t Si24R1_WriteBuffer(uint8_t reg_address, uint8_t* pBuffer, uint8_t NumByteToWrite)
{
	return SPI2_WriteBuffer(reg_address, pBuffer, NumByteToWrite);
}
