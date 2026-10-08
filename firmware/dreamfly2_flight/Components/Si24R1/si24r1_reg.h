#ifndef __SI24R1_REGISTER_H
#define __SI24R1_REGISTER_H	

/************************************************************ Si24R1 命令 ************************************************************/

#define R_REGISTER   			0x00			//读配置寄存器，低五位为寄存器地址											
#define W_REGISTER   			0x20			//写配置寄存器，低五位为寄存器地址                                          
#define R_RX_PL_WID   			0x60			//读取收到的数据字节数                                                      
#define R_RX_PAYLOAD 			0x61			//读 RX 有效数据，1-32字节                                                 
#define W_TX_PAYLOAD 			0xA0			//写 TX 有效数据，1-32字节                                                 
#define FLUSH_TX     			0xE1			//清除 TX FIFO 寄存器，应用于发射模式                                       
#define FLUSH_RX     			0xE2			//清除 RX FIFO 寄存器，应用于接收模式                                       
#define REUSE_TX_PL  			0xE3			//重新使用上一数据包，CE 为高，数据包不断被发送                             
#define W_ACK_PLAYLOAD  		0xA8    		//接收方将数据通过ACK形式发送，最多允许三帧数据存在 FIFO 中                 
#define NOP          			0xFF			//空操作，可用于读取寄存器状态

/********************************************************* Si24R1 寄存器地址 *********************************************************/

#define CONFIG       			0x00			//配置寄存器，配置中断的开关，CRC校验，开机/关断模式，接收/发送模式
#define EN_AA        			0x01			//使能自动确认，使能 0 - 5通道的自动应答功能，默认都开启
#define EN_RXADDR    			0x02			//使能接收数据管道地址，接收地址允许 0 - 5通道，默认通道0、通道1 开启
#define SETUP_AW     			0x03			//设置地址宽度配置
#define SETUP_RETR   			0x04			//自动重发配置，设置自动重发延时和最大重发次数  
#define RF_CH        			0x05			//射频信道：设置芯片工作时的信道，分别对应第0 ~ 125个信道
#define RF_SETUP     			0x06			//射频配置：设置射频数据率和发射功率    
#define STATUS       			0x07			//状态寄存器，查看 Si24R1 状态
#define BIT_RX_DR   			0x40  			//状态寄存器 BIT 位，接收完成标志位，置 1 清除
#define BIT_TX_DS   			0x20  			//状态寄存器 BIT 位，发送完成标志位，置 1 清除
#define BIT_MAX_RT  			0x10  			//状态寄存器 BIT 位，达到最大重发次数标志位，置 1 清除
#define BIT_TX_FULL  			0x01 			//状态寄存器 BIT 位，TXFIFO 满标志位
#define OBSERVE_TX   			0x08   			//发射结果统计，数据包丢失计数器（写RF_CH时复位）和重发计数器（发送新数据包时复位） 
#define RSSI           			0x09   			//接收信号强度检测
#define RX_ADDR_P0   			0x0A   			//数据通道0 接收地址，最大长度：5字节（先写低字节，所写字节数量由 ETUP_AW 设定）
#define RX_ADDR_P1   			0x0B   			//数据通道1 接收地址，最大长度：5字节（先写低字节，所写字节数量由 ETUP_AW 设定）
#define RX_ADDR_P2   			0x0C   			//数据通道2 接收地址，最低字节可设置。高字节部分必须与 RX_ADDR_P1[39:8] 相等
#define RX_ADDR_P3   			0x0D   			//数据通道3 接收地址，最低字节可设置。高字节部分必须与 RX_ADDR_P1[39:8] 相等
#define RX_ADDR_P4   			0x0E   			//数据通道4 接收地址，最低字节可设置。高字节部分必须与 RX_ADDR_P1[39:8] 相等
#define RX_ADDR_P5   			0x0F   			//数据通道5 接收地址，最低字节可设置。高字节部分必须与 RX_ADDR_P1[39:8] 相等
#define TX_ADDR      			0x10   			//发射方发送地址
#define RX_PW_P0     			0x11   			//接收数据通道0 有效数据宽度（从 1 到 32 字节）
#define RX_PW_P1     			0x12   			//接收数据通道1 有效数据宽度（从 1 到 32 字节）
#define RX_PW_P2     			0x13   			//接收数据通道2 有效数据宽度（从 1 到 32 字节）
#define RX_PW_P3     			0x14   			//接收数据通道3 有效数据宽度（从 1 到 32 字节）
#define RX_PW_P4     			0x15   			//接收数据通道4 有效数据宽度（从 1 到 32 字节）
#define RX_PW_P5     			0x16   			//接收数据通道5 有效数据宽度（从 1 到 32 字节）
#define FIFO_STATUS  			0x17   			//FIFO 状态寄存器状态		
#define DYNPD  					0x1C   			//使能动态负载长度
#define FEATURE  				0x1D			//特征寄存器

/*************************************************************************************************************************************/
	
#endif
