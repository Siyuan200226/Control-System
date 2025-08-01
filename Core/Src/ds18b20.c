#include "ds18b20.h"
#include "gpio.h"
#include "tim.h"
#include "usart.h"

void DS18B20_SetPinOutput(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void DS18B20_SetPinInput(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void DS18B20_DelayUs(uint16_t us) {
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    while (__HAL_TIM_GET_COUNTER(&htim1) < us);
}

uint8_t DS18B20_Start(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
    uint8_t response = 0;
    DS18B20_SetPinOutput(GPIOx, GPIO_Pin);
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
    DS18B20_DelayUs(500);

    DS18B20_SetPinInput(GPIOx, GPIO_Pin);
    DS18B20_DelayUs(70);

    if (!HAL_GPIO_ReadPin(GPIOx, GPIO_Pin)) {
        response = 1;
    }
    DS18B20_DelayUs(410);
    return response;
}

void DS18B20_Write(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t data) {
    for (int i = 0; i < 8; i++) {
        DS18B20_SetPinOutput(GPIOx, GPIO_Pin);
        HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);

        if ((data >> i) & 0x01) {
            DS18B20_DelayUs(3);
            DS18B20_SetPinInput(GPIOx, GPIO_Pin);
            DS18B20_DelayUs(60);
        } else {
            DS18B20_DelayUs(60);
            DS18B20_SetPinInput(GPIOx, GPIO_Pin);
        }
        DS18B20_DelayUs(1);
    }
}

uint8_t DS18B20_Read(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
    uint8_t value = 0;
    for (int i = 0; i < 8; i++) {
        DS18B20_SetPinOutput(GPIOx, GPIO_Pin);
        HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
        DS18B20_DelayUs(3);
        DS18B20_SetPinInput(GPIOx, GPIO_Pin);
        DS18B20_DelayUs(10);

        if (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin)) {
            value |= (1 << i);
        }
        DS18B20_DelayUs(50);
    }
    return value;
}

/*float DS18B20_GetTemp(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
    uint8_t temp_l, temp_h;
    int16_t temp;

    if (!DS18B20_Start(GPIOx, GPIO_Pin)) {
        return -1000.0;
    }

    DS18B20_Write(GPIOx, GPIO_Pin, 0xCC);
    DS18B20_Write(GPIOx, GPIO_Pin, 0x44);
    HAL_Delay(750);

    if (!DS18B20_Start(GPIOx, GPIO_Pin)) {
        return -1000.0;
    }

    DS18B20_Write(GPIOx, GPIO_Pin, 0xCC);
    DS18B20_Write(GPIOx, GPIO_Pin, 0xBE);

    temp_l = DS18B20_Read(GPIOx, GPIO_Pin);
    temp_h = DS18B20_Read(GPIOx, GPIO_Pin);

    temp = (temp_h << 8) | temp_l;
    return (float)temp / 16.0;
}*/

// 新增整数版本函数 (返回温度*100，精度0.01°C)
int16_t DS18B20_GetTemp_Int(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
    uint8_t temp_l, temp_h;
    int16_t temp;

    if (!DS18B20_Start(GPIOx, GPIO_Pin)) {
        return -1000;  // 错误代码
    }

    DS18B20_Write(GPIOx, GPIO_Pin, 0xCC);
    DS18B20_Write(GPIOx, GPIO_Pin, 0x44);
    HAL_Delay(750);

    if (!DS18B20_Start(GPIOx, GPIO_Pin)) {
        return -1000;  // 错误代码
    }

    DS18B20_Write(GPIOx, GPIO_Pin, 0xCC);
    DS18B20_Write(GPIOx, GPIO_Pin, 0xBE);

    temp_l = DS18B20_Read(GPIOx, GPIO_Pin);
    temp_h = DS18B20_Read(GPIOx, GPIO_Pin);

    temp = (temp_h << 8) | temp_l;

    // 转换为整数 (温度*100)
    // DS18B20精度为0.0625°C，所以temp需要除以16
    // 为了保持精度，先乘以100再除以16
    return (temp * 100) / 16;
}
