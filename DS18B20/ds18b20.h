#ifndef DS18B20_H
#define DS18B20_H

#include "stm32f4xx.h"

uint8_t DS18B20_Init(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void DS18B20_ReadId(uint8_t * ds18b20_id, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
float DS18B20_GetTemp_SkipRom(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
float DS18B20_GetTemp_MatchRom(uint8_t *ds18b20_id,GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

#endif
