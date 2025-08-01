/*
 * ds18b20.h
 *
 *  Created on: Apr 24, 2025
 *      Author: Administrator
 */

#ifndef __DS18B20_H
#define __DS18B20_H

#include "stm32f1xx_hal.h"


//#define DS18B20_1_PORT GPIOA
//#define DS18B20_1_PIN  GPIO_PIN_1

#define DS18B20_2_PORT GPIOA
#define DS18B20_2_PIN  GPIO_PIN_2

void DS18B20_SetPinOutput(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void DS18B20_SetPinInput(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void DS18B20_DelayUs(uint16_t us);
uint8_t DS18B20_Start(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void DS18B20_Write(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t data);
uint8_t DS18B20_Read(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
//float DS18B20_GetTemp(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
// 整数版本函数 (返回温度*100，精度0.01°C)
int16_t DS18B20_GetTemp_Int(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

#endif /* INC_DS18B20_H_ */
