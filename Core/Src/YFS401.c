#include "YFS401.h"
#include "usart.h"

GOLBAL_FLOW golbal_flow;

//流量传感器模型的枚举类型
typedef enum {
    MODE_4_PIPE = 0,   //
    MODE_6_PIPE = 1,   //
    MODE_6MM_PIPE = 2  // 6mm PIPE
} FlowModel;

// 定义流量系数 (整数版本, 乘以100)
uint16_t flowK_x100[3] = {500, 550, 8000};                    // 流量系数K * 100
uint16_t pulseCntPerLiter[3] = {300, 330, 4800};              // 每升水的脉冲数

// define Flow_Model
FlowModel flowModel = MODE_6MM_PIPE; // can change to the other kinds of pipe


uint32_t pluse1L;           //Test the number of pulses per 1L of water

void Flow_Read_Int(void)
{
   // 根据 Flow_Model 选择相应的流量参数
    uint16_t flowKValue_x100 = flowK_x100[flowModel];
    uint16_t pulseCntValue = pulseCntPerLiter[flowModel];
    
	if(golbal_flow.pluse_1s > 0)
	{
    /*计算公式：
            累计流量 = 累计脉冲数 / 每升水脉冲数
            单位转换：升->毫升 (*1000)
        */
		golbal_flow.acculat_ml += (golbal_flow.pluse_1s * 1000) / pulseCntValue;   // 单位mL
        pluse1L += golbal_flow.pluse_1s;
		 /*计算公式：
            瞬时流量 = ((脉冲频率) / 每升水脉冲数) * 60s
                     = (脉冲频率) / (流量系数K)
            为避免浮点运算，流量*100存储
        */
		golbal_flow.instant_x100 = (golbal_flow.pluse_1s * 10000) / flowKValue_x100;  // 单位：(L/min)*100

        if(golbal_flow.acculat_ml >= 1000000)        // 当累计流量达1000L
        {
            golbal_flow.acculat_ml = 0;
        }
    }
    else
    {
        golbal_flow.instant_x100 = 0;
    }
	
	 // 通过UART3输出调试信息
    char debug_msg[128];
    snprintf(debug_msg, sizeof(debug_msg),
             "Instantaneous flow: %d.%02d L/min, Accumulated flow: %d mL\r\n",
             golbal_flow.instant_x100/100, golbal_flow.instant_x100%100,
             golbal_flow.acculat_ml);
    HAL_UART_Transmit(&huart3, (uint8_t*)debug_msg, strlen(debug_msg), 100);
   
    golbal_flow.receive_flag = 0;   // 处理完成标志位清零
    golbal_flow.pluse_1s = 0;       // 清零脉冲计数
}

/*// 保留原有的浮点数版本函数 (兼容性)
void Flow_Read(void)
{
    // 根据 Flow_Model 选择相应的流量参数
    float flowKValue = (float)flowK_x100[flowModel] / 100.0f;
    float pulseCntValue = (float)pulseCntPerLiter[flowModel];

    if(golbal_flow.pluse_1s > 0)
    {
        //计算公式：累计流量 = 累计脉冲数 / 每升水脉冲数

        golbal_flow.acculat += (golbal_flow.pluse_1s * 1000 / pulseCntValue);   // 单位mL
        pluse1L += golbal_flow.pluse_1s;

        //计算公式：瞬时流量 = ((脉冲频率) / 每升水脉冲数) * 60s = (脉冲频率) / (流量系数K)

        golbal_flow.instant = golbal_flow.pluse_1s / flowKValue;  // 单位：L/min

        if(golbal_flow.acculat >= 1000000)        // 当累计流量达1000L
        {
            golbal_flow.acculat = 0;
        }
    }
    else
    {
        golbal_flow.instant = 0;
    }

    // 通过UART3输出调试信息
    char debug_msg[128];
    snprintf(debug_msg, sizeof(debug_msg),
             "Instantaneous flow: %.2f L/min, Accumulated flow: %.0f mL\r\n",
             golbal_flow.instant, golbal_flow.acculat);
    HAL_UART_Transmit(&huart3, (uint8_t*)debug_msg, strlen(debug_msg), 100);

    golbal_flow.receive_flag = 0;   // 处理完成标志位清零
    golbal_flow.pluse_1s = 0;       // 清零脉冲计数
}*/
