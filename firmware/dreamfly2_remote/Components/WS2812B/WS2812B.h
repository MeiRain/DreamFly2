#ifndef __WS2812B_H
#define __WS2812B_H

#include "stm32f10x.h" 

void WS2812B_Init(void);
void WS2812B_Reset(void);
void WS2812B_SetColour(uint8_t green, uint8_t red, uint8_t blue);
void WS2812B_Red(void);
void WS2812B_Orange(void);
void WS2812B_Yellow(void);
void WS2812B_Green(void);
void WS2812B_Cyan(void);
void WS2812B_Blue(void);
void WS2812B_Violet(void);
void WS2812B_White(void);
void WS2812B_OFF(void);
void WS2812B_BlueFlash(void);

#endif
