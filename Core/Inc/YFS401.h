#ifndef __YFS201C_H__
#define __YFS201C_H__

#include <stdint.h>
//#include "usart.h"
#include "stdio.h"

typedef struct
{
	uint8_t receive_flag;
	uint16_t pluse_1s;
	
	//float instant;   //浮点数版本
	//float acculat;
	// 整数版本 (新增)
    uint16_t instant_x100;      // 瞬时流量 * 100 (L/min)
    uint32_t acculat_ml;        // 累计流量 (mL)
}GOLBAL_FLOW;

extern GOLBAL_FLOW  golbal_flow;

//void Flow_Read(void); //浮点数版本

//整数版本函数
void Flow_Read_Int(void);

#endif
