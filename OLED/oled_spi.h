#ifndef OLED_H
#define OLED_H

#include "stm32f4xx.h" 

void OLED_Init(void);
void OLED_ShowString(uint8_t x, uint8_t y, uint8_t *chr, uint8_t size);
void OLED_ShowNum(uint8_t x,uint8_t y,uint32_t num,uint8_t len, uint8_t size);

#endif
