#include "ws2812b.h"

/************************代码移植修改区************************************/

#define WS2812B_CLK RCC_AHB1Periph_GPIOB // 端口时钟
#define WS2812B_PORT GPIOB				 // 端口
#define WS2812B_PIN GPIO_Pin_10			 // 引脚

/**************************************************************************/

#define WS2812B_AMOUNT 4
#define WS2812B_SET1 WS2812B_PORT->BSRRL |= WS2812B_PIN
#define WS2812B_SET0 WS2812B_PORT->BSRRH |= WS2812B_PIN

// 跑马灯WS2812B三元色配出七彩跑马灯
static uint32_t Run_buf[][16] =
{
	{0xFFA500,0,0,0,0xFFA500,0,0,0,0xFFA500,0,0,0,0xFFA500,0,0,0,},//橙色
	{0x00FF00,0,0,0,0x00FF00,0,0,0,0x00FF00,0,0,0,0x00FF00,0,0,0,},//绿色
	{0xFF00FF,0,0,0,0xFF00FF,0,0,0,0xFF00FF,0,0,0,0xFF00FF,0,0,0,},//紫色
	{0x00FFFF,0,0,0,0x00FFFF,0,0,0,0x00FFFF,0,0,0,0x00FFFF,0,0,0,},//青色
	{0x0000FF,0,0,0,0x0000FF,0,0,0,0x0000FF,0,0,0,0x0000FF,0,0,0,},//蓝色
	{0xFFFF00,0,0,0,0xFFFF00,0,0,0,0xFFFF00,0,0,0,0xFFFF00,0,0,0,},//黄色
	{0xFFFFFF,0,0,0,0xFFFFFF,0,0,0,0xFFFFFF,0,0,0,0xFFFFFF,0,0,0,},//白色
};

uint8_t Run_flag = 1; // 跑马灯标志

/*********************************************************************
 *函  数：void LED_Init(void)
 *功  能：初始化LED控制引脚
 *参  数：无
 *返回值: 无
 *备  注: 无
 *********************************************************************/
void WS2812B_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(WS2812B_CLK, ENABLE);

	GPIO_InitStructure.GPIO_Pin = WS2812B_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;  // 选择模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; // 输出类型
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(WS2812B_PORT, &GPIO_InitStructure);
	
	GPIO_ResetBits(WS2812B_PORT, WS2812B_PIN);
}


