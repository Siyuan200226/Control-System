/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "ds18b20.h"
#include "YFS401.h"
#include <stdarg.h>  // 用于可变参数函数
#include <stdio.h>   // 用于vsnprintf
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//执行器GPIO 定义
#define HEATER_PORT GPIOB
#define HEATER_PIN  GPIO_PIN_0
#define PUMP_PORT   GPIOB
#define PUMP_PIN    GPIO_PIN_7
#define VACUUM_PUMP_PORT GPIOB
#define VACUUM_PUMP_PIN  GPIO_PIN_6

// LED定义
#define TEMP_LED_PORT GPIOA
#define TEMP_LED_PIN  GPIO_PIN_9
#define FLOW_LED_PORT GPIOA
#define FLOW_LED_PIN  GPIO_PIN_10
#define DO_LED_PORT   GPIOA
#define DO_LED_PIN    GPIO_PIN_11

// 温度控制阈值和滞后 (使用整数表示，精度0.01°C)
#define TEMP_SETPOINT       3700    // 37.00°C - 目标温度
#define TEMP_HYSTERESIS     100     // 1.00°C - 滞后带宽
#define TEMP_TURN_ON        (TEMP_SETPOINT - TEMP_HYSTERESIS)  // 目标-带宽 - 开启温度
#define TEMP_TURN_OFF       (TEMP_SETPOINT + TEMP_HYSTERESIS)  // 目标＋带宽 - 关闭温度

// 流量控制阈值和滞后 (精度0.01 L/min)
#define FLOW_SETPOINT       85      // 0.50 L/min - 目标流量
#define FLOW_HYSTERESIS     25      // 0.25 L/min - 滞后带宽
#define FLOW_TURN_ON        (FLOW_SETPOINT - FLOW_HYSTERESIS)  // 0.25 L/min - 开启流量
#define FLOW_TURN_OFF       (FLOW_SETPOINT + FLOW_HYSTERESIS)  // 0.75 L/min - 关闭流量

// DO控制阈值和滞后 (精度0.001 mg/L)
#define DO_SETPOINT         2000    // 3.000 mg/L - 目标DO值
#define DO_HYSTERESIS       500     // 0.500 mg/L - 滞后带宽
#define DO_TURN_ON          (DO_SETPOINT + DO_HYSTERESIS)  // 2.500 mg/L - 开启真空泵
#define DO_TURN_OFF         (DO_SETPOINT - DO_HYSTERESIS)  // 1.500 mg/L - 关闭真空泵

// 防抖动计数器阈值
#define DEBOUNCE_COUNT      3       // 需要连续3次满足条件才切换状态

// DO传感器相关定义
#define VREF_MV             3300    // VREF(mv)
#define ADC_RES             4096    // ADC Resolution (12-bit)
#define READ_TEMP           25      // Current water temperature ℃
#define CAL1_V              1600    // mv (25℃校准点)
#define CAL1_T              25      // ℃
#define CAL2_V              1300    // mv (15℃校准点)
#define CAL2_T              15      // ℃
#define FILTER_SIZE         5       // 5点移动平均

// 官方DO饱和度表 (0-40℃，单位：ug/L)
const uint16_t DO_Table[41] = {
    14460, 14220, 13820, 13440, 13090, 12740, 12420, 12110, 11810, 11530,
    11260, 11010, 10770, 10530, 10300, 10080, 9860, 9660, 9460, 9270,
    9080, 8900, 8730, 8570, 8410, 8250, 8110, 7960, 7820, 7690,
    7560, 7430, 7300, 7180, 7070, 6950, 6840, 6730, 6630, 6530, 6410
};

// FT812相关定义
uint8_t ui_state = 0;

// UI状态定义
#define UI_MAIN       0
#define UI_ZOOM_1     1
#define UI_ZOOM_2     2
#define UI_ZOOM_3     3
#define UI_SETTINGS   4

// 全局目标值变量（与屏幕同步）
int32_t target_temp = TEMP_SETPOINT;       // 目标温度 (x100格式)
int32_t target_flow = FLOW_SETPOINT;       // 目标流量 (x100格式)
int32_t target_do = DO_SETPOINT;           // 目标DO (x1000格式)

// 参数设置结构体
typedef struct {
    int32_t target_value;
    int32_t x_min, x_max;
    int32_t y_min, y_max;
} ChartSettings;

ChartSettings chart_settings[3];
uint8_t current_chart_setting = 0;
uint8_t current_param = 0;
char input_buffer[10] = {0};
uint8_t input_pos = 0;

// 数据滚动缓冲区定义
#define MAX_DATA_POINTS 80
#define DISPLAY_POINTS  50

// FT812相关定义
#define USED_FONT_COUNT 4
static const uint8_t used_fonts[USED_FONT_COUNT] = {20, 22, 26, 28};
static uint8_t font_width_tab[USED_FONT_COUNT][128];

typedef struct { uint16_t x1, y1, x2, y2; } Rect;
#define CHART1_RECT   (Rect){ 10,  10, 235, 131 }
#define CHART2_RECT   (Rect){245,  10, 470, 131 }
#define CHART3_RECT   (Rect){ 10, 141, 235, 262 }
#define WP_RECT       (Rect){250, 146, 455, 176 }
#define VP_RECT       (Rect){250, 186, 455, 216 }
#define HT_RECT       (Rect){250, 226, 455, 256 }

#define BTN_SETTING   (Rect){270, 15, 375, 50 }
#define BTN_CONFIRM   (Rect){370, 15, 470, 50 }

// FT812 引脚定义
#define EVE_CS_Pin        GPIO_PIN_4
#define EVE_CS_Port       GPIOA
#define EVE_PD_Pin        GPIO_PIN_3
#define EVE_PD_Port       GPIOB

// FT812 寄存器定义
#define REG_ID            0x302000
#define REG_HCYCLE        0x30202C
#define REG_HOFFSET       0x302030
#define REG_HSIZE         0x302034
#define REG_HSYNC0        0x302038
#define REG_HSYNC1        0x30203C
#define REG_VCYCLE        0x302040
#define REG_VOFFSET       0x302044
#define REG_VSIZE         0x302048
#define REG_VSYNC0        0x30204C
#define REG_VSYNC1        0x302050
#define REG_DLSWAP        0x302054
#define REG_PCLK_POL      0x30206C
#define REG_PCLK          0x302070
#define REG_SWIZZLE       0x302064
#define REG_CSPREAD       0x302068
#define REG_DITHER        0x302060
#define REG_PWM_HZ        0x3020D0
#define REG_PWM_DUTY      0x3020D4
#define REG_GPIO_DIR      0x302090
#define REG_GPIO          0x302094

#define REG_TOUCH_MODE          0x302104
#define REG_TOUCH_ADC_MODE      0x302108
#define REG_TOUCH_CHARGE        0x30210C
#define REG_TOUCH_SETTLE        0x302110
#define REG_TOUCH_OVERSAMPLE    0x302114
#define REG_TOUCH_RZTHRESH      0x302118
#define REG_TOUCH_SCREEN_XY     0x302124
#define REG_TOUCH_RZ            0x302120

#define RAM_DL            0x300000
#define DLSWAP_FRAME      2

#define CMD_DLSTART       0xFFFFFF00
#define CMD_SWAP          0xFFFFFF01
#define CMD_TEXT          0xFFFFFF0C
#define CMD_BUTTON        0xFFFFFF0D
#define CMD_FGCOLOR       0xFFFFFF0A
#define CMD_BGCOLOR       0xFFFFFF09
#define CMD_CALIBRATE     0xFFFFFF15

#define REG_CMD_WRITE     0x3020FC
#define REG_CMD_READ      0x3020F8
#define RAM_CMD           0x308000

#define CLEAR_COLOR_RGB(r,g,b) (0x02000000 | ((r) << 16) | ((g) << 8) | (b))
#define CLEAR(c,s,t)          (0x26000000 | ((c) << 2) | ((s) << 1) | (t))
#define DISPLAY()             0x00000000
#define COLOR_RGB(r,g,b)      (0x04000000 | ((r) << 16) | ((g) << 8) | (b))
#define BEGIN(prim)           (0x1F000000 | (prim))
#define VERTEX2II(x,y,h,c)    (0x80000000 | (((x) & 0x1FF) << 21) | (((y) & 0x1FF) << 12) | (((h) & 0x1F) << 7) | ((c) & 0x7F))
#define END()                 0x21000000
#define POINT_SIZE(s)         (0x0D000000 | ((s) & 0x1FFF))

#define BITMAPS    1
#define POINTS     2
#define LINES      3
#define RECTS      9

#define MEM_WRITE         0x80
#define MEM_READ          0x00
#define HOST_CMD_CLKEXT   0x44
#define HOST_CMD_ACTIVE   0x00

// Settings界面相关定义
#define KEYPAD_X_START  280
#define KEYPAD_Y_START  80
#define KEY_WIDTH       50
#define KEY_HEIGHT      35
#define KEY_SPACING     8

#define BTN_BACK        (Rect){380, 10, 460, 40}
#define BTN_SETTINGS_TITLE (Rect){280, 10, 370, 40}

#define PARAM_X_START   20
#define PARAM_Y_START   80
#define PARAM_WIDTH     240
#define PARAM_HEIGHT    25
#define PARAM_SPACING   35

const char* keypad_labels[12] = {
    "1", "2", "3",
    "4", "5", "6",
    "7", "8", "9",
    "<", "0", "OK"
};

const char* param_names[5] = {
    "Target Value:",
    "X Min:",
    "X Max:",
    "Y Min:",
    "Y Max:"
};

const char* chart_names[3] = {
    "Flow Rate",
    "Dissolved Oxygen",
    "Temperature"
};

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */


// 状态变量
typedef struct {
    uint8_t temp_control_active;  // 温度控制状态 (LED状态)
    uint8_t flow_control_active;  // 流量控制状态 (LED状态)
    uint8_t do_control_active;    // 溶氧控制状态 (LED状态)
    uint8_t heater_on;           // 加热器状态
    uint8_t pump_on;             // 水泵状态
    uint8_t vacuum_pump_on;      // 真空泵状态


    // 滞后控制状态
    uint8_t temp_debounce_count; // 温度防抖动计数器
    uint8_t flow_debounce_count; // 流量防抖动计数器
    uint8_t do_debounce_count;   // DO防抖动计数器
    uint8_t temp_pending_state;  // 温度待切换状态
    uint8_t flow_pending_state;  // 流量待切换状态
    uint8_t do_pending_state;    // DO待切换状态
} SystemState;

SystemState system_state = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// 流量相关变量 (使用整数)
uint32_t flow_pulses_per_min = 0;  // 每分钟脉冲数
uint32_t instant_flow_x100 = 0;    // 瞬时流量 * 100 (保留2位小数)

// DO传感器相关变量
uint32_t adc_raw;
uint16_t adc_voltage;
uint8_t temperature = READ_TEMP;
uint16_t do_value;

// 滤波相关变量
uint16_t voltage_buffer[FILTER_SIZE];   // 电压滤波缓冲区
uint8_t filter_index = 0;               // 滤波索引
uint8_t filter_filled = 0;              // 滤波器是否填满

// FT812相关变量
static uint8_t init_complete = 0;

// 图表配置结构体
typedef struct {
    const char* title;
    const char* x_label;
    const char* y_label;
    int32_t x_min, x_max;
    int32_t y_min, y_max;
    uint8_t x_divisions;
    uint8_t y_divisions;
    uint8_t x_decimal_places;
    uint8_t y_decimal_places;
} ChartConfig_Int;

// 数据点结构体
typedef struct {
    int32_t x, y;
} DataPoint_Int;

// 实时数据缓冲区结构
typedef struct {
    DataPoint_Int buffer[MAX_DATA_POINTS];
    int data_count;
    uint32_t start_time;
    uint32_t last_update_time;
    uint32_t update_interval;
    int32_t time_window;
    int32_t current_time_offset;
    uint8_t is_scrolling;
} RealTimeDataBuffer;

RealTimeDataBuffer chart_buffers[3];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void UpdateHeaterControl(void);
void UpdatePumpControl(void);
void UpdateVacuumPumpControl(void);
void UpdateTemperatureControlWithHysteresis(int16_t temp_x100);
void UpdateFlowControlWithHysteresis(uint32_t flow_x100);
void UpdateDOControlWithHysteresis(uint16_t do_x1000);
void Debug_Print(const char* message);
void Debug_Printf(const char* format, ...);

// DO传感器相关函数
uint16_t readDO(uint8_t temperature_c);
uint16_t filterVoltage(uint16_t new_voltage);
void ReadDOSensor(void);

// FT812函数
void FT812_SendHostCommand(uint8_t cmd);
uint8_t FT812_ReadByte(uint32_t addr);
void FT812_WriteByte(uint32_t addr, uint8_t data);
void FT812_WriteReg16(uint32_t addr, uint16_t data);
void FT812_WriteReg32(uint32_t addr, uint32_t data);
uint32_t FT812_ReadReg32(uint32_t addr);
uint16_t FT812_ReadReg16(uint32_t addr);
void FT812_SendCommand(uint32_t cmd);
void WaitForCoprocessor(void);
void FT812_Initialize(void);
void TouchInitialize_WithYourCalibrationData(void);
void LoadFontWidthTable(uint8_t font);
void LoadAllRequiredFonts(void);

// 显示函数
void DrawString_Proportional(uint32_t *dl_addr, int x, int y, int font, const char* str, int center);
void DrawArrow(uint32_t* dl_addr, int16_t x1, int16_t y1, int16_t x2, int16_t y2, int16_t size);
void DrawDashedLine(uint32_t* dl_addr, int16_t x1, int16_t y1, int16_t x2, int16_t y2);
void DrawDetailedChart_Int(uint32_t* dl_addr, Rect chart_area, const ChartConfig_Int* config, DataPoint_Int* data, int data_count, uint8_t chart_index);
void DrawMainUI_Int(void);
void DrawZoomChart_Int(uint8_t idx);
void DrawKeypad(uint32_t* dl_addr);
void DrawParameterSettings(uint32_t* dl_addr);
void DrawSettingsUI(void);

// 数据处理函数
void InitDefaultSettings(void);
void InitRealTimeBuffers(void);
void UpdateRealTimeData(uint8_t chart_index);
void GetRealTimeDisplayData(uint8_t chart_index, DataPoint_Int* display_data, int max_display_count);
void ResetChartData(uint8_t chart_index);
void OnSettingsChanged(uint8_t chart_index);
void IntToString(char* str, int32_t value, uint8_t decimal_places);
void ChartToScreen_Int_Safe(int32_t chart_x, int32_t chart_y, const ChartConfig_Int* config, Rect chart_area, uint16_t* screen_x, uint16_t* screen_y);

// 触摸处理函数
void ProcessTouchAndUI(uint16_t x, uint16_t y);
void ProcessSettingsTouch(uint16_t x, uint16_t y);
void ProcessNumberInput(char key);
void ProcessBackspace(void);
void ProcessConfirm(void);

// 目标值同步函数
void SyncTargetValuesToControl(void);
void SyncControlTargetsToChart(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//**********************************************************************************************************


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_3)
    {
        golbal_flow.pluse_1s++;
    }
}

//串口调试
void Debug_Print(const char* message)
{
    // 通过UART3硬件串口输出
    HAL_UART_Transmit(&huart3, (uint8_t*)message, strlen(message), 100);
}

//格式化调试信息输出函数 format: 格式化字符串 可变参数
void Debug_Printf(const char* format, ...)
{
    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    Debug_Print(buffer);
}

//5点移动平均滤波
uint16_t filterVoltage(uint16_t new_voltage)
{
    // 将新值存入缓冲区
    voltage_buffer[filter_index] = new_voltage;
    filter_index = (filter_index + 1) % FILTER_SIZE;

    // 检查缓冲区是否填满
    if (!filter_filled && filter_index == 0) {
        filter_filled = 1;
    }

    // 计算平均值
    uint32_t sum = 0;
    uint8_t count = filter_filled ? FILTER_SIZE : filter_index;

    for (uint8_t i = 0; i < count; i++) {
        sum += voltage_buffer[i];
    }

    return (uint16_t)(sum / count);
}

//基于官方算法计算DO值
uint16_t readDO(uint8_t temperature_c)
{
    // 限制温度范围
    if (temperature_c > 40) temperature_c = 40;

    // 从表中获取饱和DO值
    uint16_t do_sat = DO_Table[temperature_c];

    // 基于官方两点校准的线性插值
    uint16_t cal_voltage;
    if (temperature_c == CAL1_T) {
        cal_voltage = CAL1_V;
    } else if (temperature_c == CAL2_T) {
        cal_voltage = CAL2_V;
    } else {
        // 线性插值计算校准电压
        if (temperature_c > CAL1_T) {
            // 外推 (高于25℃)
            cal_voltage = CAL1_V;
        } else {
            // 插值 (15-25℃之间)
            cal_voltage = CAL2_V + (CAL1_V - CAL2_V) * (temperature_c - CAL2_T) / (CAL1_T - CAL2_T);
        }
    }

    // 计算DO浓度 (简化版官方算法)
    if (adc_voltage <= 0) return 0;

    // 假设零点为0V，线性计算
    uint32_t do_result = ((uint32_t)adc_voltage * do_sat) / cal_voltage;

    return (uint16_t)do_result;
}

// 读取DO传感器数据
void ReadDOSensor(void)
{
    // 读取ADC原始值
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK)
    {
        adc_raw = HAL_ADC_GetValue(&hadc1);
    }
    HAL_ADC_Stop(&hadc1);

    // 转换为电压 (mv)
    uint16_t raw_voltage = (uint16_t)(adc_raw * VREF_MV / ADC_RES);

    // 应用滤波
    adc_voltage = filterVoltage(raw_voltage);

    // 计算DO值 (基于官方算法)
    do_value = readDO(temperature);
}

// 带滞后的温度控制逻辑
void UpdateTemperatureControlWithHysteresis(int16_t temp_x100)
{
    uint8_t new_state;

    if (temp_x100 == -100000) {
        // 传感器错误，保持当前状态
        system_state.temp_debounce_count = 0;
        return;
    }

    // 根据滞后逻辑确定新状态
    if (system_state.temp_control_active) {
        // 当前是开启状态，检查是否应该关闭
        new_state = (temp_x100 < TEMP_TURN_OFF) ? 1 : 0;
    } else {
        // 当前是关闭状态，检查是否应该开启
        new_state = (temp_x100 < TEMP_TURN_ON) ? 1 : 0;
    }

    // 防抖动逻辑
    if (new_state == system_state.temp_pending_state) {
        system_state.temp_debounce_count++;
        if (system_state.temp_debounce_count >= DEBOUNCE_COUNT) {
            if (system_state.temp_control_active != new_state) {
                system_state.temp_control_active = new_state;

                // 更新LED状态
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, new_state ? GPIO_PIN_RESET : GPIO_PIN_SET);

                Debug_Printf("Temperature Control State Changed: %s\r\n", new_state ? "ON" : "OFF");
                Debug_Printf("Current: %d.%02d°C, Thresholds: ON<%d.%02d°C, OFF>%d.%02d°C\r\n",
                            temp_x100/100, temp_x100%100,
                            TEMP_TURN_ON/100, TEMP_TURN_ON%100,
                            TEMP_TURN_OFF/100, TEMP_TURN_OFF%100);
            }
            system_state.temp_debounce_count = 0;
        }
    } else {
        system_state.temp_pending_state = new_state;
        system_state.temp_debounce_count = 1;
    }
}

// 带滞后的流量控制逻辑
void UpdateFlowControlWithHysteresis(uint32_t flow_x100)
{
    uint8_t new_state;

    // 根据滞后逻辑确定新状态
    if (system_state.flow_control_active) {
        // 当前是开启状态，检查是否应该关闭
        new_state = (flow_x100 < FLOW_TURN_OFF) ? 1 : 0;
    } else {
        // 当前是关闭状态，检查是否应该开启
        new_state = (flow_x100 < FLOW_TURN_ON) ? 1 : 0;
    }

    // 防抖动逻辑
    if (new_state == system_state.flow_pending_state) {
        system_state.flow_debounce_count++;
        if (system_state.flow_debounce_count >= DEBOUNCE_COUNT) {
            if (system_state.flow_control_active != new_state) {
                system_state.flow_control_active = new_state;

                // 更新LED状态
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, new_state ? GPIO_PIN_RESET : GPIO_PIN_SET);

                Debug_Printf("Flow Control State Changed: %s\r\n", new_state ? "ON" : "OFF");
                Debug_Printf("Current: %d.%02d L/min, Thresholds: ON<%d.%02d, OFF>%d.%02d\r\n",
                            flow_x100/100, flow_x100%100,
                            FLOW_TURN_ON/100, FLOW_TURN_ON%100,
                            FLOW_TURN_OFF/100, FLOW_TURN_OFF%100);
            }
            system_state.flow_debounce_count = 0;
        }
    } else {
        system_state.flow_pending_state = new_state;
        system_state.flow_debounce_count = 1;
    }
}

// 带滞后的DO控制逻辑
void UpdateDOControlWithHysteresis(uint16_t do_x1000)
{
    uint8_t new_state;

    // 根据滞后逻辑确定新状态
    if (system_state.do_control_active) {
        // 当前是开启状态，检查是否应该关闭
        new_state = (do_x1000 > DO_TURN_OFF) ? 1 : 0;
    } else {
        // 当前是关闭状态，检查是否应该开启
        new_state = (do_x1000 > DO_TURN_ON) ? 1 : 0;
    }

    // 防抖动逻辑
    if (new_state == system_state.do_pending_state) {
        system_state.do_debounce_count++;
        if (system_state.do_debounce_count >= DEBOUNCE_COUNT) {
            if (system_state.do_control_active != new_state) {
                system_state.do_control_active = new_state;

                // 更新LED状态
                HAL_GPIO_WritePin(DO_LED_PORT, DO_LED_PIN, new_state ? GPIO_PIN_RESET : GPIO_PIN_SET);

                Debug_Printf("DO Control State Changed: %s\r\n", new_state ? "ON" : "OFF");
                Debug_Printf("Current: %d.%03d mg/L, Thresholds: ON>%d.%03d, OFF<%d.%03d\r\n",
                            do_x1000/1000, do_x1000%1000,
                            DO_TURN_ON/1000, DO_TURN_ON%1000,
                            DO_TURN_OFF/1000, DO_TURN_OFF%1000);
            }
            system_state.do_debounce_count = 0;
        }
    } else {
        system_state.do_pending_state = new_state;
        system_state.do_debounce_count = 1;
    }
}

//更新加热器控制 (低电平触发MOSFET)
void UpdateHeaterControl(void)
{
    if (system_state.temp_control_active) {
        // 温度LED亮起时，打开加热器 (输出低电平)
        if (!system_state.heater_on) {
            HAL_GPIO_WritePin(HEATER_PORT, HEATER_PIN, GPIO_PIN_SET);  // 低电平导通
            system_state.heater_on = 1;
            Debug_Print("Heater: ON (GPIO LOW)\r\n");
        }
    } else {
        // 温度LED关闭时，关闭加热器 (输出高电平)
        if (system_state.heater_on) {
            HAL_GPIO_WritePin(HEATER_PORT, HEATER_PIN, GPIO_PIN_RESET);    // 高电平断开
            system_state.heater_on = 0;
            Debug_Print("Heater: OFF (GPIO HIGH)\r\n");
        }
    }
}

// 水泵控制 (低电平触发MOSFET)
void UpdatePumpControl(void)
{
    if (system_state.flow_control_active) {
        // 流量LED亮起时，打开水泵 (输出低电平)
        if (!system_state.pump_on) {
            HAL_GPIO_WritePin(PUMP_PORT, PUMP_PIN, GPIO_PIN_SET);      // 低电平导通
            system_state.pump_on = 1;
            Debug_Print("Heater: ON (GPIO LOW)\r\n");
        }
    } else {
        // 流量LED关闭时，关闭水泵 (输出高电平)
        if (system_state.pump_on) {
            HAL_GPIO_WritePin(PUMP_PORT, PUMP_PIN, GPIO_PIN_RESET);        // 高电平断开
            system_state.pump_on = 0;
             Debug_Print("Heater: OFF (GPIO HIGH)\r\n");
        }
    }
}

// 真空泵控制 (低电平触发MOSFET)
void UpdateVacuumPumpControl(void)
{
    if (system_state.do_control_active) {
        // DO LED亮起时，打开真空泵 (输出低电平)
        if (!system_state.vacuum_pump_on) {
            HAL_GPIO_WritePin(VACUUM_PUMP_PORT, VACUUM_PUMP_PIN, GPIO_PIN_SET);  // 低电平导通
            system_state.vacuum_pump_on = 1;
            Debug_Print("Vacuum Pump: ON (GPIO LOW)\r\n");
        }
    } else {
        // DO LED关闭时，关闭真空泵 (输出高电平)
        if (system_state.vacuum_pump_on) {
            HAL_GPIO_WritePin(VACUUM_PUMP_PORT, VACUUM_PUMP_PIN, GPIO_PIN_RESET);    // 高电平断开
            system_state.vacuum_pump_on = 0;
            Debug_Print("Vacuum Pump: OFF (GPIO HIGH)\r\n");
        }
    }
}

//*******************************FT812******************************************************
// 目标值同步函数
void SyncTargetValuesToControl(void)
{
    // 将屏幕设置的目标值同步到控制系统
    target_temp = chart_settings[2].target_value;  // 温度图表(索引2)的目标值
    target_flow = chart_settings[0].target_value;  // 流量图表(索引0)的目标值
    target_do = chart_settings[1].target_value;    // DO图表(索引1)的目标值
}

void SyncControlTargetsToChart(void)
{
    // 将控制系统的目标值同步到屏幕设置
    chart_settings[2].target_value = target_temp;  // 温度
    chart_settings[0].target_value = target_flow;  // 流量
    chart_settings[1].target_value = target_do;    // DO
}

// FT812 GPIO控制
static void EVE_CS_LOW(void) {
    HAL_GPIO_WritePin(EVE_CS_Port, EVE_CS_Pin, GPIO_PIN_RESET);
}

static void EVE_CS_HIGH(void) {
    HAL_GPIO_WritePin(EVE_CS_Port, EVE_CS_Pin, GPIO_PIN_SET);
}

static void EVE_PD_LOW(void) {
    HAL_GPIO_WritePin(EVE_PD_Port, EVE_PD_Pin, GPIO_PIN_RESET);
}

static void EVE_PD_HIGH(void) {
    HAL_GPIO_WritePin(EVE_PD_Port, EVE_PD_Pin, GPIO_PIN_SET);
}

// FT812通信函数
void FT812_SendHostCommand(uint8_t cmd) {
    uint8_t command[3] = {cmd, 0x00, 0x00};
    EVE_CS_LOW();
    HAL_SPI_Transmit(&hspi1, command, 3, HAL_MAX_DELAY);
    EVE_CS_HIGH();
}

uint8_t FT812_ReadByte(uint32_t addr) {
    uint8_t cmd[4];
    uint8_t data = 0;

    cmd[0] = MEM_READ | ((addr >> 16) & 0x3F);
    cmd[1] = (addr >> 8) & 0xFF;
    cmd[2] = addr & 0xFF;
    cmd[3] = 0x00;

    EVE_CS_LOW();
    HAL_SPI_Transmit(&hspi1, cmd, 4, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, &data, 1, HAL_MAX_DELAY);
    EVE_CS_HIGH();

    return data;
}

void FT812_WriteByte(uint32_t addr, uint8_t data) {
    uint8_t cmd[4];

    cmd[0] = MEM_WRITE | ((addr >> 16) & 0x3F);
    cmd[1] = (addr >> 8) & 0xFF;
    cmd[2] = addr & 0xFF;
    cmd[3] = data;

    EVE_CS_LOW();
    HAL_SPI_Transmit(&hspi1, cmd, 4, HAL_MAX_DELAY);
    EVE_CS_HIGH();
}

void FT812_WriteReg16(uint32_t addr, uint16_t data) {
    FT812_WriteByte(addr, data & 0xFF);
    FT812_WriteByte(addr + 1, (data >> 8) & 0xFF);
}

void FT812_WriteReg32(uint32_t addr, uint32_t data) {
    FT812_WriteByte(addr, data & 0xFF);
    FT812_WriteByte(addr + 1, (data >> 8) & 0xFF);
    FT812_WriteByte(addr + 2, (data >> 16) & 0xFF);
    FT812_WriteByte(addr + 3, (data >> 24) & 0xFF);
}

uint32_t FT812_ReadReg32(uint32_t addr) {
    uint32_t data = FT812_ReadByte(addr);
    data |= (uint32_t)FT812_ReadByte(addr + 1) << 8;
    data |= (uint32_t)FT812_ReadByte(addr + 2) << 16;
    data |= (uint32_t)FT812_ReadByte(addr + 3) << 24;
    return data;
}

uint16_t FT812_ReadReg16(uint32_t addr) {
    uint16_t data = FT812_ReadByte(addr);
    data |= (uint16_t)FT812_ReadByte(addr + 1) << 8;
    return data;
}

// FT812初始化相关函数
int GetFontIndex(uint8_t font) {
    for(int i = 0; i < USED_FONT_COUNT; i++) {
        if(used_fonts[i] == font) {
            return i;
        }
    }
    return -1;
}

void LoadFontWidthTable(uint8_t font)
{
    int font_index = GetFontIndex(font);
    if(font_index == -1) return;

    uint32_t font_metrics_addr = 0x2FFFFC;
    uint32_t root_addr = FT812_ReadReg32(font_metrics_addr);
    uint32_t metrics_addr = root_addr + 148 * (font - 16);

    for(int i = 0; i < 128; i++) {
        font_width_tab[font_index][i] = FT812_ReadByte(metrics_addr + i);
    }
}

void LoadAllRequiredFonts(void) {
    for(int i = 0; i < USED_FONT_COUNT; i++) {
        LoadFontWidthTable(used_fonts[i]);
        HAL_Delay(10);
    }
}

void TouchInitialize_WithYourCalibrationData(void) {
    FT812_WriteReg32(0x302150, 32368);
    FT812_WriteReg32(0x302154, 613);
    FT812_WriteReg32(0x302158, -1006113);
    FT812_WriteReg32(0x30215C, 263);
    FT812_WriteReg32(0x302160, -20164);
    FT812_WriteReg32(0x302164, 19061412);

    uint32_t dl_addr = RAM_DL;
    FT812_WriteReg32(dl_addr, CLEAR_COLOR_RGB(0, 100, 0)); dl_addr += 4;
    FT812_WriteReg32(dl_addr, CLEAR(1, 1, 1)); dl_addr += 4;
    FT812_WriteReg32(dl_addr, COLOR_RGB(255, 255, 255)); dl_addr += 4;
    FT812_WriteReg32(dl_addr, BEGIN(BITMAPS)); dl_addr += 4;

    FT812_WriteReg32(dl_addr, END()); dl_addr += 4;
    FT812_WriteReg32(dl_addr, DISPLAY());
    FT812_WriteByte(REG_DLSWAP, DLSWAP_FRAME);

    HAL_Delay(500);
}

void FT812_Initialize(void) {
    EVE_CS_HIGH();
    HAL_Delay(10);
    EVE_PD_LOW();
    HAL_Delay(20);
    EVE_PD_HIGH();
    HAL_Delay(20);

    FT812_SendHostCommand(HOST_CMD_CLKEXT);
    HAL_Delay(10);
    FT812_SendHostCommand(HOST_CMD_ACTIVE);
    HAL_Delay(300);

    FT812_ReadByte(REG_ID);

    FT812_WriteReg16(REG_HCYCLE, 548);
    FT812_WriteReg16(REG_HOFFSET, 43);
    FT812_WriteReg16(REG_HSIZE, 480);
    FT812_WriteReg16(REG_HSYNC0, 0);
    FT812_WriteReg16(REG_HSYNC1, 41);
    FT812_WriteReg16(REG_VCYCLE, 292);
    FT812_WriteReg16(REG_VOFFSET, 12);
    FT812_WriteReg16(REG_VSIZE, 272);
    FT812_WriteReg16(REG_VSYNC0, 0);
    FT812_WriteReg16(REG_VSYNC1, 10);

    FT812_WriteByte(REG_SWIZZLE, 0);
    FT812_WriteByte(REG_PCLK_POL, 1);
    FT812_WriteByte(REG_CSPREAD, 1);
    FT812_WriteByte(REG_DITHER, 1);

    FT812_WriteByte(REG_TOUCH_MODE, 0x03);
    FT812_WriteByte(REG_TOUCH_ADC_MODE, 0x01);
    FT812_WriteReg16(REG_TOUCH_CHARGE, 6000);
    FT812_WriteByte(REG_TOUCH_SETTLE, 3);
    FT812_WriteByte(REG_TOUCH_OVERSAMPLE, 7);
    FT812_WriteReg16(REG_TOUCH_RZTHRESH, 1200);

    TouchInitialize_WithYourCalibrationData();

    FT812_WriteReg16(REG_PWM_HZ, 1000);
    FT812_WriteByte(REG_PWM_DUTY, 128);
    FT812_WriteByte(REG_GPIO_DIR, 0x80);
    FT812_WriteByte(REG_GPIO, 0x80);

    FT812_WriteByte(REG_PCLK, 5);
    init_complete = 1;
}

// 绘图函数
void DrawString_Proportional(uint32_t *dl_addr, int x, int y, int font, const char* str, int center)
{
    int font_index = GetFontIndex(font);
    if(font_index == -1) {
        for(int i = 0; str[i] != '\0'; i++) {
            FT812_WriteReg32(*dl_addr, VERTEX2II(x + i * 8, y, font, str[i]));
            *dl_addr += 4;
        }
        return;
    }

    int len = strlen(str);
    int sum = 0;

    if(center) {
        for(int i = 0; i < len; i++) {
            sum += font_width_tab[font_index][(uint8_t)str[i]];
        }
        x -= sum / 2;
    }

    int x_pos = x;
    for(int i = 0; str[i] != '\0'; i++) {
        FT812_WriteReg32(*dl_addr, VERTEX2II(x_pos, y, font, str[i]));
        *dl_addr += 4;
        x_pos += font_width_tab[font_index][(uint8_t)str[i]];
    }
}

void DrawArrow(uint32_t* dl_addr, int16_t x1, int16_t y1, int16_t x2, int16_t y2, int16_t size) {
    FT812_WriteReg32(*dl_addr, BEGIN(LINES)); *dl_addr += 4;
    FT812_WriteReg32(*dl_addr, VERTEX2II(x1, y1, 0, 0)); *dl_addr += 4;
    FT812_WriteReg32(*dl_addr, VERTEX2II(x2, y2, 0, 0)); *dl_addr += 4;

    int16_t dx = x2 - x1;
    int16_t dy = y2 - y1;

    if (abs(dx) > abs(dy)) {
        if (dx > 0) {
            FT812_WriteReg32(*dl_addr, VERTEX2II(x2, y2, 0, 0)); *dl_addr += 4;
            FT812_WriteReg32(*dl_addr, VERTEX2II(x2 - size, y2 - size/2, 0, 0)); *dl_addr += 4;
            FT812_WriteReg32(*dl_addr, VERTEX2II(x2, y2, 0, 0)); *dl_addr += 4;
            FT812_WriteReg32(*dl_addr, VERTEX2II(x2 - size, y2 + size/2, 0, 0)); *dl_addr += 4;
        } else {
            FT812_WriteReg32(*dl_addr, VERTEX2II(x2, y2, 0, 0)); *dl_addr += 4;
            FT812_WriteReg32(*dl_addr, VERTEX2II(x2 + size, y2 - size/2, 0, 0)); *dl_addr += 4;
            FT812_WriteReg32(*dl_addr, VERTEX2II(x2, y2, 0, 0)); *dl_addr += 4;
            FT812_WriteReg32(*dl_addr, VERTEX2II(x2 + size, y2 + size/2, 0, 0)); *dl_addr += 4;
        }
    } else {
        if (dy < 0) {
            FT812_WriteReg32(*dl_addr, VERTEX2II(x2, y2, 0, 0)); *dl_addr += 4;
            FT812_WriteReg32(*dl_addr, VERTEX2II(x2 - size/2, y2 + size, 0, 0)); *dl_addr += 4;
            FT812_WriteReg32(*dl_addr, VERTEX2II(x2, y2, 0, 0)); *dl_addr += 4;
            FT812_WriteReg32(*dl_addr, VERTEX2II(x2 + size/2, y2 + size, 0, 0)); *dl_addr += 4;
        } else {
            FT812_WriteReg32(*dl_addr, VERTEX2II(x2, y2, 0, 0)); *dl_addr += 4;
            FT812_WriteReg32(*dl_addr, VERTEX2II(x2 - size/2, y2 - size, 0, 0)); *dl_addr += 4;
            FT812_WriteReg32(*dl_addr, VERTEX2II(x2, y2, 0, 0)); *dl_addr += 4;
            FT812_WriteReg32(*dl_addr, VERTEX2II(x2 + size/2, y2 - size, 0, 0)); *dl_addr += 4;
        }
    }

    FT812_WriteReg32(*dl_addr, END()); *dl_addr += 4;
}

void DrawDashedLine(uint32_t* dl_addr, int16_t x1, int16_t y1, int16_t x2, int16_t y2) {
    int16_t dx = x2 - x1;
    int16_t dy = y2 - y1;
    int16_t steps = abs(dx) > abs(dy) ? abs(dx) : abs(dy);

    if (steps == 0) return;

    FT812_WriteReg32(*dl_addr, BEGIN(POINTS)); *dl_addr += 4;
    FT812_WriteReg32(*dl_addr, POINT_SIZE(1 * 16)); *dl_addr += 4;

    for (int i = 0; i <= steps; i += 8) {
        int16_t x = x1 + (dx * i) / steps;
        int16_t y = y1 + (dy * i) / steps;
        FT812_WriteReg32(*dl_addr, VERTEX2II(x, y, 0, 0)); *dl_addr += 4;
    }

    FT812_WriteReg32(*dl_addr, END()); *dl_addr += 4;
}

void IntToString(char* str, int32_t value, uint8_t decimal_places) {
    if (decimal_places == 0) {
        sprintf(str, "%ld", value);
        return;
    }

    uint8_t negative = 0;
    if (value < 0) {
        negative = 1;
        value = -value;
    }

    int32_t divisor = 1;
    for (uint8_t i = 0; i < decimal_places; i++) {
        divisor *= 10;
    }

    int32_t integer_part = value / divisor;
    int32_t decimal_part = value % divisor;

    if (negative) {
        sprintf(str, "-%ld.%0*ld", integer_part, decimal_places, decimal_part);
    } else {
        sprintf(str, "%ld.%0*ld", integer_part, decimal_places, decimal_part);
    }
}

void ChartToScreen_Int_Safe(int32_t chart_x, int32_t chart_y, const ChartConfig_Int* config,
                           Rect chart_area, uint16_t* screen_x, uint16_t* screen_y) {
    uint16_t plot_x1 = chart_area.x1 + 40;
    uint16_t plot_y1 = chart_area.y1 + 30;
    uint16_t plot_x2 = chart_area.x2 - 20;
    uint16_t plot_y2 = chart_area.y2 - 40;

    int32_t x_range = config->x_max - config->x_min;
    int32_t x_offset = chart_x - config->x_min;

    if(x_offset < 0) x_offset = 0;
    if(x_offset > x_range) x_offset = x_range;

    *screen_x = plot_x1 + (uint16_t)((x_offset * (plot_x2 - plot_x1)) / x_range);

    int32_t y_range = config->y_max - config->y_min;
    int32_t y_offset = chart_y - config->y_min;

    if(y_offset < 0) y_offset = 0;
    if(y_offset > y_range) y_offset = y_range;

    *screen_y = plot_y2 - (uint16_t)((y_offset * (plot_y2 - plot_y1)) / y_range);

    if(*screen_x < plot_x1) *screen_x = plot_x1;
    if(*screen_x > plot_x2) *screen_x = plot_x2;
    if(*screen_y < plot_y1) *screen_y = plot_y1;
    if(*screen_y > plot_y2) *screen_y = plot_y2;
}

// 数据处理函数
void InitDefaultSettings(void) {
    // 使用控制系统的目标值初始化
    chart_settings[0].target_value = target_flow;    // 流量
    chart_settings[0].x_min = 0;
    chart_settings[0].x_max = 1000;
    chart_settings[0].y_min = 0;
    chart_settings[0].y_max = 10000;

    chart_settings[1].target_value = target_do;      // DO
    chart_settings[1].x_min = 0;
    chart_settings[1].x_max = 1000;
    chart_settings[1].y_min = 0;
    chart_settings[1].y_max = 15000;

    chart_settings[2].target_value = target_temp;    // 温度
    chart_settings[2].x_min = 0;
    chart_settings[2].x_max = 1000;
    chart_settings[2].y_min = 0;
    chart_settings[2].y_max = 5000;
}

void InitRealTimeBuffers(void) {
    uint32_t current_time = HAL_GetTick();

    for(int i = 0; i < 3; i++) {
        chart_buffers[i].data_count = 0;
        chart_buffers[i].start_time = current_time;
        chart_buffers[i].last_update_time = current_time;
        chart_buffers[i].update_interval = 500; //
        chart_buffers[i].time_window = chart_settings[i].x_max - chart_settings[i].x_min;
        chart_buffers[i].current_time_offset = 0;
        chart_buffers[i].is_scrolling = 0;

        memset(chart_buffers[i].buffer, 0, sizeof(chart_buffers[i].buffer));
    }
}

void UpdateRealTimeData(uint8_t chart_index) {
    uint32_t current_time = HAL_GetTick();
    RealTimeDataBuffer* buffer = &chart_buffers[chart_index];

    if(current_time - buffer->last_update_time < buffer->update_interval) {
        return;
    }

    buffer->last_update_time = current_time;

    DataPoint_Int new_point;
    new_point.x = buffer->data_count * 50; // 每个点间隔0.5秒(50/100)

    int32_t x_max = chart_settings[chart_index].x_max;
    if(new_point.x > x_max) {
        buffer->is_scrolling = 1;
        new_point.x = x_max;
    }

    // 使用统一的传感器数据
    switch(chart_index) {
        case 0: // 流量数据
            new_point.y = instant_flow_x100;
            break;

        case 1: // DO数据
            ReadDOSensor(); // 实时读取
            new_point.y = do_value;
            break;

        case 2: // 温度数据
            {
                int16_t temp_x100 = DS18B20_GetTemp_Int(DS18B20_2_PORT, DS18B20_2_PIN);
                if(temp_x100 == -100000) {
                    new_point.y = chart_settings[chart_index].y_min;
                } else {
                    new_point.y = temp_x100;
                }
            }
            break;
    }

    // Y值范围限制
    int32_t y_min = chart_settings[chart_index].y_min;
    int32_t y_max = chart_settings[chart_index].y_max;
    if(new_point.y < y_min) new_point.y = y_min;
    if(new_point.y > y_max) new_point.y = y_max;

    // 修复：改进缓冲区管理
    if(!buffer->is_scrolling && buffer->data_count < MAX_DATA_POINTS) {
        // 填充模式：直接添加
        buffer->buffer[buffer->data_count] = new_point;
        buffer->data_count++;
    } else {
        // 滚动模式：移除旧数据，添加新数据
        buffer->is_scrolling = 1;

        // 左移所有数据
        for(int i = 0; i < buffer->data_count - 1; i++) {
            buffer->buffer[i] = buffer->buffer[i + 1];
        }

        // 在最后位置添加新数据
        if(buffer->data_count < MAX_DATA_POINTS) {
            buffer->buffer[buffer->data_count] = new_point;
            buffer->data_count++;
        } else {
            buffer->buffer[MAX_DATA_POINTS - 1] = new_point;
        }
    }

}

void GetRealTimeDisplayData(uint8_t chart_index, DataPoint_Int* display_data, int max_display_count) {
    RealTimeDataBuffer* buffer = &chart_buffers[chart_index];
    int32_t x_min = chart_settings[chart_index].x_min;
    int32_t x_max = chart_settings[chart_index].x_max;
    int32_t y_min = chart_settings[chart_index].y_min;

    if(buffer->data_count == 0) {
        for(int i = 0; i < max_display_count; i++) {
            display_data[i].x = x_min;
            display_data[i].y = y_min;
        }
        return;
    }

    if(buffer->is_scrolling) {
        int display_count = (buffer->data_count < max_display_count) ? buffer->data_count : max_display_count;

        for(int i = 0; i < display_count; i++) {
            display_data[i].y = buffer->buffer[i].y;
            display_data[i].x = x_min + (i * (x_max - x_min)) / (display_count - 1);
        }

        for(int i = display_count; i < max_display_count; i++) {
            display_data[i].x = x_min;
            display_data[i].y = y_min;
        }
    } else {
        int display_count = (buffer->data_count < max_display_count) ? buffer->data_count : max_display_count;

        for(int i = 0; i < display_count; i++) {
            display_data[i] = buffer->buffer[i];
            display_data[i].x += x_min;
        }

        for(int i = display_count; i < max_display_count; i++) {
            display_data[i].x = x_min;
            display_data[i].y = y_min;
        }
    }
}

void ResetChartData(uint8_t chart_index) {
    RealTimeDataBuffer* buffer = &chart_buffers[chart_index];

    buffer->data_count = 0;
    buffer->start_time = HAL_GetTick();
    buffer->last_update_time = buffer->start_time;
    buffer->current_time_offset = 0;
    buffer->is_scrolling = 0;
    buffer->time_window = chart_settings[chart_index].x_max - chart_settings[chart_index].x_min;

    memset(buffer->buffer, 0, sizeof(buffer->buffer));
}

void OnSettingsChanged(uint8_t chart_index) {
    ResetChartData(chart_index);
    // 同步目标值到控制系统
    SyncTargetValuesToControl();
}
//**************检查是否缺少**************
// 绘制详细图表的函数
void DrawDetailedChart_Int(uint32_t* dl_addr, Rect chart_area, const ChartConfig_Int* config,
                           DataPoint_Int* data, int data_count, uint8_t chart_index) {
    uint16_t plot_x1 = chart_area.x1 + 40;
    uint16_t plot_y1 = chart_area.y1 + 30;
    uint16_t plot_x2 = chart_area.x2 - 20;
    uint16_t plot_y2 = chart_area.y2 - 40;

    // 绘制坐标轴
    FT812_WriteReg32(*dl_addr, COLOR_RGB(0, 0, 0)); *dl_addr += 4;
    DrawArrow(dl_addr, plot_x1, plot_y2, plot_x2 + 15, plot_y2, 6);
    DrawArrow(dl_addr, plot_x1, plot_y2, plot_x1, plot_y1 - 15, 6);

    // 绘制刻度标记
    FT812_WriteReg32(*dl_addr, BEGIN(LINES)); *dl_addr += 4;

    for(int i = 0; i <= config->x_divisions; i++) {
        uint16_t x = plot_x1 + (plot_x2 - plot_x1) * i / config->x_divisions;
        FT812_WriteReg32(*dl_addr, VERTEX2II(x, plot_y2, 0, 0)); *dl_addr += 4;
        FT812_WriteReg32(*dl_addr, VERTEX2II(x, plot_y2 + 5, 0, 0)); *dl_addr += 4;
    }

    for(int i = 0; i <= config->y_divisions; i++) {
        uint16_t y = plot_y2 - (plot_y2 - plot_y1) * i / config->y_divisions;
        FT812_WriteReg32(*dl_addr, VERTEX2II(plot_x1 - 5, y, 0, 0)); *dl_addr += 4;
        FT812_WriteReg32(*dl_addr, VERTEX2II(plot_x1, y, 0, 0)); *dl_addr += 4;
    }
    FT812_WriteReg32(*dl_addr, END()); *dl_addr += 4;

    // 绘制刻度值
    FT812_WriteReg32(*dl_addr, BEGIN(BITMAPS)); *dl_addr += 4;

    for(int i = 0; i <= config->x_divisions; i++) {
        uint16_t x = plot_x1 + (plot_x2 - plot_x1) * i / config->x_divisions;
        int32_t value = config->x_min + (config->x_max - config->x_min) * i / config->x_divisions;
        char value_str[12];
        IntToString(value_str, value, config->x_decimal_places);
        DrawString_Proportional(dl_addr, x - 10, plot_y2 + 8, 20, value_str, 0);
    }

    for(int i = 0; i <= config->y_divisions; i++) {
        uint16_t y = plot_y2 - (plot_y2 - plot_y1) * i / config->y_divisions;
        int32_t value = config->y_min + (config->y_max - config->y_min) * i / config->y_divisions;
        char value_str[12];
        IntToString(value_str, value, config->y_decimal_places);
        DrawString_Proportional(dl_addr, plot_x1 - 35, y - 6, 20, value_str, 0);
    }
    FT812_WriteReg32(*dl_addr, END()); *dl_addr += 4;

    // 绘制坐标轴标签
    FT812_WriteReg32(*dl_addr, COLOR_RGB(60, 60, 60)); *dl_addr += 4;
    FT812_WriteReg32(*dl_addr, BEGIN(BITMAPS)); *dl_addr += 4;

    int x_label_width = strlen(config->x_label) * 8;
    DrawString_Proportional(dl_addr, (plot_x1 + plot_x2) / 2 - x_label_width/2, plot_y2 + 25, 20, config->x_label, 0);
    DrawString_Proportional(dl_addr, plot_x1 - 35, plot_y1 - 15, 20, config->y_label, 0);

    FT812_WriteReg32(*dl_addr, END()); *dl_addr += 4;

    // 计算有效数据点数量
    int valid_data_count = 0;
    int32_t y_min_threshold = config->y_min;

    for(int i = 0; i < data_count; i++) {
        if(data[i].y > y_min_threshold) {
            valid_data_count = i + 1;
        }
    }

    // 绘制数据折线
    if(valid_data_count > 1) {
        const uint32_t colors[] = {0xFF0000, 0x00AA00, 0x0066FF};
        uint32_t color = colors[chart_index % 3];

        FT812_WriteReg32(*dl_addr, COLOR_RGB((color >> 16) & 0xFF, (color >> 8) & 0xFF, color & 0xFF));
        *dl_addr += 4;
        FT812_WriteReg32(*dl_addr, BEGIN(LINES)); *dl_addr += 4;

        for(int i = 0; i < valid_data_count - 1; i++) {
            if(data[i].y > y_min_threshold && data[i+1].y > y_min_threshold) {
                uint16_t x1, y1, x2, y2;
                ChartToScreen_Int_Safe(data[i].x, data[i].y, config, chart_area, &x1, &y1);
                ChartToScreen_Int_Safe(data[i+1].x, data[i+1].y, config, chart_area, &x2, &y2);

                FT812_WriteReg32(*dl_addr, VERTEX2II(x1, y1, 0, 0)); *dl_addr += 4;
                FT812_WriteReg32(*dl_addr, VERTEX2II(x2, y2, 0, 0)); *dl_addr += 4;
            }
        }
        FT812_WriteReg32(*dl_addr, END()); *dl_addr += 4;

        // 绘制数据点
        FT812_WriteReg32(*dl_addr, POINT_SIZE(3 * 16)); *dl_addr += 4;
        FT812_WriteReg32(*dl_addr, BEGIN(POINTS)); *dl_addr += 4;

        for(int i = 0; i < valid_data_count; i += 2) {
            if(data[i].y > y_min_threshold) {
                uint16_t x, y;
                ChartToScreen_Int_Safe(data[i].x, data[i].y, config, chart_area, &x, &y);
                FT812_WriteReg32(*dl_addr, VERTEX2II(x, y, 0, 0)); *dl_addr += 4;
            }
        }
        FT812_WriteReg32(*dl_addr, END()); *dl_addr += 4;
    }

    // 绘制期望值虚线
    int32_t target_value = chart_settings[chart_index].target_value;
    if (target_value > config->y_min && target_value < config->y_max) {
        int32_t y_range = config->y_max - config->y_min;
        int32_t y_offset = target_value - config->y_min;
        uint16_t target_y = plot_y2 - (uint16_t)((y_offset * (plot_y2 - plot_y1)) / y_range);

        FT812_WriteReg32(*dl_addr, COLOR_RGB(255, 100, 100)); *dl_addr += 4;
        DrawDashedLine(dl_addr, plot_x1, target_y, plot_x2, target_y);

        FT812_WriteReg32(*dl_addr, COLOR_RGB(200, 0, 0)); *dl_addr += 4;
        FT812_WriteReg32(*dl_addr, BEGIN(BITMAPS)); *dl_addr += 4;

        char target_str[15];
        IntToString(target_str, target_value, 2);

        DrawString_Proportional(dl_addr, plot_x2 - 100, target_y - 8, 16, "Target:", 0);
        DrawString_Proportional(dl_addr, plot_x2 - 40, target_y - 8, 16, target_str, 0);

        FT812_WriteReg32(*dl_addr, END()); *dl_addr += 4;
    }

    // 绘制标题
    FT812_WriteReg32(*dl_addr, COLOR_RGB(0, 0, 0)); *dl_addr += 4;
    FT812_WriteReg32(*dl_addr, BEGIN(BITMAPS)); *dl_addr += 4;
    int title_width = strlen(config->title) * 12;
    DrawString_Proportional(dl_addr, (chart_area.x1 + chart_area.x2) / 2 - title_width/2 + 45 , chart_area.y1 + 8, 26, config->title, 0);
    FT812_WriteReg32(*dl_addr, END()); *dl_addr += 4;
}

// 主界面绘制函数 - **关键修改：同步执行器状态**
void DrawMainUI_Int(void) {
    for(int i = 0; i < 3; i++) {
        UpdateRealTimeData(i);
    }

    uint32_t dl_addr = RAM_DL;

    FT812_WriteReg32(dl_addr, CLEAR_COLOR_RGB(240, 240, 240)); dl_addr += 4;
    FT812_WriteReg32(dl_addr, CLEAR(1,1,1)); dl_addr += 4;

    ChartConfig_Int configs[3] = {
        {"Flow Rate", "Time(s)", "(L/min)",
         chart_settings[0].x_min, chart_settings[0].x_max,
         chart_settings[0].y_min, chart_settings[0].y_max, 5, 5, 1, 2},
        {"Dissolved Oxygen", "Time(s)", "(mg/L)",
         chart_settings[1].x_min, chart_settings[1].x_max,
         chart_settings[1].y_min, chart_settings[1].y_max, 5, 5, 1, 0},
        {"Temperature", "Time(s)", "(C)",
         chart_settings[2].x_min, chart_settings[2].x_max,
         chart_settings[2].y_min, chart_settings[2].y_max, 5, 5, 1, 2}
    };

    Rect chart_rects[3] = { CHART1_RECT, CHART2_RECT, CHART3_RECT };

    for(int i = 0; i < 3; i++) {
        DataPoint_Int data[20];
        GetRealTimeDisplayData(i, data, 20);
        DrawDetailedChart_Int(&dl_addr, chart_rects[i], &configs[i], data, 20, i);
    }

    // **关键修改：状态显示区域与控制系统状态同步**
    struct {
        const char* name;
        uint8_t stat;
        Rect r;
    } items[] = {
        {"Water pump status: ", system_state.pump_on, WP_RECT},
        {"Vacuum pump status: ", system_state.vacuum_pump_on, VP_RECT},
        {"Heating status: ", system_state.heater_on, HT_RECT},
    };

    for(int i=0; i<3; i++){
        Rect r = items[i].r;
        FT812_WriteReg32(dl_addr, COLOR_RGB(200,200,200)); dl_addr += 4;
        FT812_WriteReg32(dl_addr, BEGIN(RECTS)); dl_addr += 4;
        FT812_WriteReg32(dl_addr, VERTEX2II(r.x1, r.y1, 0, 0)); dl_addr += 4;
        FT812_WriteReg32(dl_addr, VERTEX2II(r.x2, r.y2, 0, 0)); dl_addr += 4;
        FT812_WriteReg32(dl_addr, END()); dl_addr += 4;

        FT812_WriteReg32(dl_addr, COLOR_RGB(0,0,0)); dl_addr += 4;
        FT812_WriteReg32(dl_addr, BEGIN(BITMAPS)); dl_addr += 4;
        const char* txt = items[i].name;
        int xx = r.x1 + 10, yy = r.y1 + 10;
        DrawString_Proportional(&dl_addr, xx, yy, 26, txt, 0);

        const char* v = items[i].stat ? "ON" : "OFF";
        FT812_WriteReg32(dl_addr, COLOR_RGB(items[i].stat?0:200, items[i].stat?160:0, 0)); dl_addr += 4;
        DrawString_Proportional(&dl_addr, r.x2-45, yy, 26, v, 0);
        FT812_WriteReg32(dl_addr, END()); dl_addr += 4;
    }

    FT812_WriteReg32(dl_addr, DISPLAY());
    FT812_WriteByte(REG_DLSWAP, DLSWAP_FRAME);
}

// 放大图表绘制函数
void DrawZoomChart_Int(uint8_t idx) {
    uint32_t dl_addr = RAM_DL;

    FT812_WriteReg32(dl_addr, CLEAR_COLOR_RGB(250,250,250)); dl_addr += 4;
    FT812_WriteReg32(dl_addr, CLEAR(1,1,1)); dl_addr += 4;

    UpdateRealTimeData(idx - 1);

    ChartConfig_Int configs[3] = {
        {"Flow Rate Monitor", "Time (seconds)", "(L/min)",
         chart_settings[0].x_min, chart_settings[0].x_max,
         chart_settings[0].y_min, chart_settings[0].y_max, 10, 10, 1, 2},
        {"Dissolved Oxygen Monitor", "Time (seconds)", "(mg/L)",
         chart_settings[1].x_min, chart_settings[1].x_max,
         chart_settings[1].y_min, chart_settings[1].y_max, 10, 10, 1, 0},
        {"Temperature Monitor", "Time (seconds)", "(C)",
         chart_settings[2].x_min, chart_settings[2].x_max,
         chart_settings[2].y_min, chart_settings[2].y_max, 10, 10, 1, 2}
    };

    Rect large_chart = {20, 70, 460, 230};

    DataPoint_Int data[50];
    GetRealTimeDisplayData(idx - 1, data, 50);

    DrawDetailedChart_Int(&dl_addr, large_chart, &configs[idx-1], data, 50, idx-1);

    // 添加时间状态显示
    FT812_WriteReg32(dl_addr, COLOR_RGB(100, 100, 100)); dl_addr += 4;
    FT812_WriteReg32(dl_addr, BEGIN(BITMAPS)); dl_addr += 4;

    RealTimeDataBuffer* buffer = &chart_buffers[idx - 1];
    char status_text[50];
    uint32_t elapsed_ms = HAL_GetTick() - buffer->start_time;
    uint32_t elapsed_sec = elapsed_ms / 1000;

    if(buffer->is_scrolling) {
        sprintf(status_text, "Scrolling Mode - Runtime: %lds", elapsed_sec);
    } else {
        sprintf(status_text, "Filling Mode - Runtime: %lds", elapsed_sec);
    }

    DrawString_Proportional(&dl_addr, 25, 50, 20, status_text, 0);
    FT812_WriteReg32(dl_addr, END()); dl_addr += 4;

    // 绘制控制按钮
    FT812_WriteReg32(dl_addr, COLOR_RGB(150,150,200)); dl_addr += 4;
    FT812_WriteReg32(dl_addr, BEGIN(RECTS)); dl_addr += 4;
    FT812_WriteReg32(dl_addr, VERTEX2II(BTN_SETTING.x1,BTN_SETTING.y1,0,0)); dl_addr += 4;
    FT812_WriteReg32(dl_addr, VERTEX2II(BTN_SETTING.x2,BTN_SETTING.y2,0,0)); dl_addr += 4;
    FT812_WriteReg32(dl_addr, END()); dl_addr += 4;

    FT812_WriteReg32(dl_addr, COLOR_RGB(0,0,0)); dl_addr += 4;
    FT812_WriteReg32(dl_addr, BEGIN(BITMAPS)); dl_addr += 4;
    DrawString_Proportional(&dl_addr, BTN_SETTING.x1+10, BTN_SETTING.y1+12, 26, "Settings", 0);
    FT812_WriteReg32(dl_addr, END()); dl_addr += 4;

    FT812_WriteReg32(dl_addr, COLOR_RGB(200,220,180)); dl_addr += 4;
    FT812_WriteReg32(dl_addr, BEGIN(RECTS)); dl_addr += 4;
    FT812_WriteReg32(dl_addr, VERTEX2II(BTN_CONFIRM.x1,BTN_CONFIRM.y1,0,0)); dl_addr += 4;
    FT812_WriteReg32(dl_addr, VERTEX2II(BTN_CONFIRM.x2,BTN_CONFIRM.y2,0,0)); dl_addr += 4;
    FT812_WriteReg32(dl_addr, END()); dl_addr += 4;

    FT812_WriteReg32(dl_addr, COLOR_RGB(0,0,0)); dl_addr += 4;
    FT812_WriteReg32(dl_addr, BEGIN(BITMAPS)); dl_addr += 4;
    DrawString_Proportional(&dl_addr, BTN_CONFIRM.x1+10, BTN_CONFIRM.y1+12, 26, "Back", 0);
    FT812_WriteReg32(dl_addr, END()); dl_addr += 4;

    FT812_WriteReg32(dl_addr, DISPLAY());
    FT812_WriteByte(REG_DLSWAP, DLSWAP_FRAME);
}

// Settings界面相关函数
void DrawKeypad(uint32_t* dl_addr) {
    for (int i = 0; i < 12; i++) {
        int row = i / 3;
        int col = i % 3;

        int16_t x = KEYPAD_X_START + col * (KEY_WIDTH + KEY_SPACING);
        int16_t y = KEYPAD_Y_START + row * (KEY_HEIGHT + KEY_SPACING);

        FT812_WriteReg32(*dl_addr, COLOR_RGB(200, 200, 200)); *dl_addr += 4;
        FT812_WriteReg32(*dl_addr, BEGIN(RECTS)); *dl_addr += 4;
        FT812_WriteReg32(*dl_addr, VERTEX2II(x, y, 0, 0)); *dl_addr += 4;
        FT812_WriteReg32(*dl_addr, VERTEX2II(x + KEY_WIDTH, y + KEY_HEIGHT, 0, 0)); *dl_addr += 4;
        FT812_WriteReg32(*dl_addr, END()); *dl_addr += 4;

        FT812_WriteReg32(*dl_addr, COLOR_RGB(100, 100, 100)); *dl_addr += 4;
        FT812_WriteReg32(*dl_addr, BEGIN(LINES)); *dl_addr += 4;
        FT812_WriteReg32(*dl_addr, VERTEX2II(x, y, 0, 0)); *dl_addr += 4;
        FT812_WriteReg32(*dl_addr, VERTEX2II(x + KEY_WIDTH, y, 0, 0)); *dl_addr += 4;
        FT812_WriteReg32(*dl_addr, VERTEX2II(x + KEY_WIDTH, y, 0, 0)); *dl_addr += 4;
        FT812_WriteReg32(*dl_addr, VERTEX2II(x + KEY_WIDTH, y + KEY_HEIGHT, 0, 0)); *dl_addr += 4;
        FT812_WriteReg32(*dl_addr, VERTEX2II(x + KEY_WIDTH, y + KEY_HEIGHT, 0, 0)); *dl_addr += 4;
        FT812_WriteReg32(*dl_addr, VERTEX2II(x, y + KEY_HEIGHT, 0, 0)); *dl_addr += 4;
        FT812_WriteReg32(*dl_addr, VERTEX2II(x, y + KEY_HEIGHT, 0, 0)); *dl_addr += 4;
        FT812_WriteReg32(*dl_addr, VERTEX2II(x, y, 0, 0)); *dl_addr += 4;
        FT812_WriteReg32(*dl_addr, END()); *dl_addr += 4;

        FT812_WriteReg32(*dl_addr, COLOR_RGB(0, 0, 0)); *dl_addr += 4;
        FT812_WriteReg32(*dl_addr, BEGIN(BITMAPS)); *dl_addr += 4;
        DrawString_Proportional(dl_addr, x + KEY_WIDTH/2, y + KEY_HEIGHT/2 - 8, 28, keypad_labels[i], 1);
        FT812_WriteReg32(*dl_addr, END()); *dl_addr += 4;
    }
}

void DrawParameterSettings(uint32_t* dl_addr) {
    FT812_WriteReg32(*dl_addr, COLOR_RGB(0, 0, 100)); *dl_addr += 4;
    FT812_WriteReg32(*dl_addr, BEGIN(BITMAPS)); *dl_addr += 4;
    DrawString_Proportional(dl_addr, PARAM_X_START, 20, 26, chart_names[current_chart_setting], 0);
    FT812_WriteReg32(*dl_addr, END()); *dl_addr += 4;

    for (int i = 0; i < 5; i++) {
        int16_t y = PARAM_Y_START + i * PARAM_SPACING;

        if (i == current_param) {
            FT812_WriteReg32(*dl_addr, COLOR_RGB(255, 255, 200)); *dl_addr += 4;
            FT812_WriteReg32(*dl_addr, BEGIN(RECTS)); *dl_addr += 4;
            FT812_WriteReg32(*dl_addr, VERTEX2II(PARAM_X_START - 5, y - 2, 0, 0)); *dl_addr += 4;
            FT812_WriteReg32(*dl_addr, VERTEX2II(PARAM_X_START + PARAM_WIDTH + 5, y + PARAM_HEIGHT + 2, 0, 0)); *dl_addr += 4;
            FT812_WriteReg32(*dl_addr, END()); *dl_addr += 4;
        }

        FT812_WriteReg32(*dl_addr, COLOR_RGB(0, 0, 0)); *dl_addr += 4;
        FT812_WriteReg32(*dl_addr, BEGIN(BITMAPS)); *dl_addr += 4;
        DrawString_Proportional(dl_addr, PARAM_X_START, y, 20, param_names[i], 0);

        char value_str[20];
        int32_t value;
        switch (i) {
            case 0: value = chart_settings[current_chart_setting].target_value; break;
            case 1: value = chart_settings[current_chart_setting].x_min; break;
            case 2: value = chart_settings[current_chart_setting].x_max; break;
            case 3: value = chart_settings[current_chart_setting].y_min; break;
            case 4: value = chart_settings[current_chart_setting].y_max; break;
        }

        if (i == current_param && input_pos > 0) {
            sprintf(value_str, "%s_", input_buffer);
        } else {
            IntToString(value_str, value, 2);
        }

        DrawString_Proportional(dl_addr, PARAM_X_START + 120, y, 20, value_str, 0);
        FT812_WriteReg32(*dl_addr, END()); *dl_addr += 4;
    }
}

void DrawSettingsUI(void) {
    uint32_t dl_addr = RAM_DL;

    FT812_WriteReg32(dl_addr, CLEAR_COLOR_RGB(240, 240, 240)); dl_addr += 4;
    FT812_WriteReg32(dl_addr, CLEAR(1,1,1)); dl_addr += 4;

    FT812_WriteReg32(dl_addr, COLOR_RGB(0, 0, 150)); dl_addr += 4;
    FT812_WriteReg32(dl_addr, BEGIN(BITMAPS)); dl_addr += 4;
    DrawString_Proportional(&dl_addr, BTN_SETTINGS_TITLE.x1, BTN_SETTINGS_TITLE.y1 + 10, 26, "Settings", 0);
    FT812_WriteReg32(dl_addr, END()); dl_addr += 4;

    FT812_WriteReg32(dl_addr, COLOR_RGB(200, 150, 150)); dl_addr += 4;
    FT812_WriteReg32(dl_addr, BEGIN(RECTS)); dl_addr += 4;
    FT812_WriteReg32(dl_addr, VERTEX2II(BTN_BACK.x1, BTN_BACK.y1, 0, 0)); dl_addr += 4;
    FT812_WriteReg32(dl_addr, VERTEX2II(BTN_BACK.x2, BTN_BACK.y2, 0, 0)); dl_addr += 4;
    FT812_WriteReg32(dl_addr, END()); dl_addr += 4;

    FT812_WriteReg32(dl_addr, COLOR_RGB(0, 0, 0)); dl_addr += 4;
    FT812_WriteReg32(dl_addr, BEGIN(BITMAPS)); dl_addr += 4;
    DrawString_Proportional(&dl_addr, BTN_BACK.x1 + 15, BTN_BACK.y1 + 10, 22, "Back", 0);
    FT812_WriteReg32(dl_addr, END()); dl_addr += 4;

    FT812_WriteReg32(dl_addr, COLOR_RGB(150, 150, 150)); dl_addr += 4;
    FT812_WriteReg32(dl_addr, BEGIN(LINES)); dl_addr += 4;
    FT812_WriteReg32(dl_addr, VERTEX2II(250, 40, 0, 0)); dl_addr += 4;
    FT812_WriteReg32(dl_addr, VERTEX2II(250, 240, 0, 0)); dl_addr += 4;
    FT812_WriteReg32(dl_addr, END()); dl_addr += 4;

    DrawKeypad(&dl_addr);
    DrawParameterSettings(&dl_addr);

    FT812_WriteReg32(dl_addr, DISPLAY());
    FT812_WriteByte(REG_DLSWAP, DLSWAP_FRAME);
}

// 触摸处理函数
static inline uint8_t in_rect(uint16_t x, uint16_t y, Rect r) {
    return (x >= r.x1 && x <= r.x2 && y >= r.y1 && y <= r.y2);
}

void ProcessTouchAndUI(uint16_t x, uint16_t y)
{
    if(ui_state == 0) {
        if(in_rect(x,y,CHART1_RECT)) { ui_state = 1; return; }
        if(in_rect(x,y,CHART2_RECT)) { ui_state = 2; return; }
        if(in_rect(x,y,CHART3_RECT)) { ui_state = 3; return; }
    } else if (ui_state >= 1 && ui_state <= 3) {
        if(in_rect(x,y,BTN_SETTING)) {
            current_chart_setting = ui_state - 1;
            ui_state = UI_SETTINGS;
            return;
        }
        if(in_rect(x,y,BTN_CONFIRM)) { ui_state = 0; return; }
    }
    else if (ui_state == UI_SETTINGS) {
        ProcessSettingsTouch(x, y);
    }
}

void ProcessSettingsTouch(uint16_t x, uint16_t y) {
    if (x >= BTN_BACK.x1 && x <= BTN_BACK.x2 &&
        y >= BTN_BACK.y1 && y <= BTN_BACK.y2) {
        ui_state = current_chart_setting + 1;
        return;
    }

    for (int i = 0; i < 12; i++) {
        int row = i / 3;
        int col = i % 3;

        int16_t key_x = KEYPAD_X_START + col * (KEY_WIDTH + KEY_SPACING);
        int16_t key_y = KEYPAD_Y_START + row * (KEY_HEIGHT + KEY_SPACING);

        if (x >= key_x && x <= key_x + KEY_WIDTH &&
            y >= key_y && y <= key_y + KEY_HEIGHT) {

            if (i == 9) {
                ProcessBackspace();
            } else if (i == 11) {
                ProcessConfirm();
            } else if (i == 10) {
                ProcessNumberInput('0');
            } else {
                ProcessNumberInput('1' + i);
            }
            return;
        }
    }

    for (int i = 0; i < 5; i++) {
        int16_t param_y = PARAM_Y_START + i * PARAM_SPACING;
        if (x >= PARAM_X_START && x <= PARAM_X_START + PARAM_WIDTH &&
            y >= param_y && y <= param_y + PARAM_HEIGHT) {
            current_param = i;
            memset(input_buffer, 0, sizeof(input_buffer));
            input_pos = 0;
            return;
        }
    }
}

void ProcessNumberInput(char key) {
    if (key >= '0' && key <= '9' && input_pos < 8) {
        input_buffer[input_pos++] = key;
        input_buffer[input_pos] = '\0';
    }
}

void ProcessBackspace(void) {
    if (input_pos > 0) {
        input_buffer[--input_pos] = '\0';
    }
}

void ProcessConfirm(void) {
    if (input_pos > 0) {
        int32_t value = atoi(input_buffer) * 100;

        int32_t old_x_min = chart_settings[current_chart_setting].x_min;
        int32_t old_x_max = chart_settings[current_chart_setting].x_max;

        switch (current_param) {
            case 0: chart_settings[current_chart_setting].target_value = value; break;
            case 1: chart_settings[current_chart_setting].x_min = value; break;
            case 2: chart_settings[current_chart_setting].x_max = value; break;
            case 3: chart_settings[current_chart_setting].y_min = value; break;
            case 4: chart_settings[current_chart_setting].y_max = value; break;
        }

        if(current_param == 1 || current_param == 2) {
            int32_t new_x_min = chart_settings[current_chart_setting].x_min;
            int32_t new_x_max = chart_settings[current_chart_setting].x_max;

            if(old_x_min != new_x_min || old_x_max != new_x_max) {
                OnSettingsChanged(current_chart_setting);
            }
        }

        // **关键修改：目标值修改时同步到控制系统**
        if(current_param == 0) {
            SyncTargetValuesToControl();
            Debug_Printf("Target value updated for chart %d: %ld\r\n", current_chart_setting, value);
        }

        memset(input_buffer, 0, sizeof(input_buffer));
        input_pos = 0;
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_SPI1_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_Base_Start_IT(&htim2);

  // 初始化加热器、水泵和真空泵为关闭状态 (现在关闭是低电平)
  HAL_GPIO_WritePin(HEATER_PORT, HEATER_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(PUMP_PORT, PUMP_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(VACUUM_PUMP_PORT, VACUUM_PUMP_PIN, GPIO_PIN_RESET);

  // 初始化LED为关闭状态
  HAL_GPIO_WritePin(TEMP_LED_PORT, TEMP_LED_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(FLOW_LED_PORT, FLOW_LED_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(DO_LED_PORT, DO_LED_PIN, GPIO_PIN_SET);

  // 等待系统稳定
  HAL_Delay(3000);

  // 初始化FT812显示屏
  FT812_Initialize();

  // 加载字体
  LoadAllRequiredFonts();

  // 初始化默认设置（使用控制系统的目标值）
  InitDefaultSettings();

  // 初始化实时数据缓冲区
  InitRealTimeBuffers();

  // 启动ADC校准
  HAL_ADCEx_Calibration_Start(&hadc1);


  Debug_Printf("Temperature Control: ON<%d.%02d°C, OFF>%d.%02d°C\r\n",
               TEMP_TURN_ON/100, TEMP_TURN_ON%100,
               TEMP_TURN_OFF/100, TEMP_TURN_OFF%100);
  Debug_Printf("Flow Control: ON<%d.%02d L/min, OFF>%d.%02d L/min\r\n",
               FLOW_TURN_ON/100, FLOW_TURN_ON%100,
               FLOW_TURN_OFF/100, FLOW_TURN_OFF%100);
  Debug_Printf("DO Control: ON>%d.%03d mg/L, OFF<%d.%03d mg/L\r\n",
               DO_TURN_ON/1000, DO_TURN_ON%1000,
               DO_TURN_OFF/1000, DO_TURN_OFF%1000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


	  // 1. 读取传感器数据
	  // 温度读取 (返回整数，精度0.01°C)
      int16_t temp2_x100 = DS18B20_GetTemp_Int(DS18B20_2_PORT, DS18B20_2_PIN);

      // 流量计算 (整数运算)
      flow_pulses_per_min = golbal_flow.pluse_1s * 60;  // 每秒脉冲数 * 60 = 每分钟脉冲数     *****************************？？？？？？？？？？？？？？？？？
      instant_flow_x100 = (golbal_flow.pluse_1s * 10000) / 80;  // (脉冲数 * 10000) / 80 = 流量*100   ****可能需要修改 75？ 80？***

      // 读取DO传感器数据
      ReadDOSensor();


      // 2. 调试输出基本数据
      if (temp2_x100 != -100000) {
          Debug_Printf("Temperature: %d.%02d°C\r\n", temp2_x100/100, temp2_x100%100);
      } else {
          Debug_Print("Temperature: Sensor Error\r\n");
      }

      Debug_Printf("Flow Rate: %d.%02d L/min\r\n", instant_flow_x100/100, instant_flow_x100%100);
      Debug_Printf("DO Value: %d.%03d mg/L\r\n", do_value/1000, do_value%1000);

      // 3. 执行控制逻辑 使用滞后控制逻辑
      UpdateTemperatureControlWithHysteresis(temp2_x100);
      UpdateFlowControlWithHysteresis(instant_flow_x100);
      UpdateDOControlWithHysteresis(do_value);

      // 4. 更新执行器状态
      UpdateHeaterControl();
      UpdatePumpControl();
      UpdateVacuumPumpControl();

      // 5. 输出系统状态
      Debug_Printf("Status -> Temp LED: %s | Flow LED: %s | DO LED: %s | Heater: %s | Pump: %s | Vacuum: %s\r\n",
          system_state.temp_control_active ? "ON" : "OFF",
          system_state.flow_control_active ? "ON" : "OFF",
          system_state.do_control_active ? "ON" : "OFF",
          system_state.heater_on ? "ON" : "OFF",
          system_state.pump_on ? "ON" : "OFF",
          system_state.vacuum_pump_on ? "ON" : "OFF");
      // 6. **处理FT812触摸和显示更新**
    uint32_t current_time = HAL_GetTick();

    // 检查触摸事件
    uint32_t touch_xy = FT812_ReadReg32(REG_TOUCH_SCREEN_XY);
    uint16_t touch_rz = FT812_ReadReg16(REG_TOUCH_RZ);
    uint16_t tx = (touch_xy >> 16) & 0xFFFF, ty = touch_xy & 0xFFFF;

    // 处理有效触摸（防抖动）
    if (tx != 0x8000 && ty != 0x8000 && touch_rz < 1200) {
        static uint32_t last_touch_time = 0;
        static uint16_t last_tx = 0, last_ty = 0;

        if (current_time - last_touch_time > 400 ||
            abs(tx - last_tx) > 15 || abs(ty - last_ty) > 15) {

            // 限制坐标范围
            if (tx >= 480) tx = 479;
            if (ty >= 272) ty = 271;

            // 处理触摸事件
            ProcessTouchAndUI(tx, ty);

            last_touch_time = current_time;
            last_tx = tx;
            last_ty = ty;
        }
    }

    // 7. 更新显示界面
    switch(ui_state) {
        case UI_MAIN:
            DrawMainUI_Int();
            break;
        case UI_ZOOM_1:
        case UI_ZOOM_2:
        case UI_ZOOM_3:
            DrawZoomChart_Int(ui_state);
            break;
        case UI_SETTINGS:
            DrawSettingsUI();
            break;
        default:
            ui_state = UI_MAIN;
            break;
    }

    // 8. 控制循环延时（1秒，与原控制系统保持一致）
    HAL_Delay(500);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
        instant_flow_x100 = (golbal_flow.pluse_1s * 10000) / 75;  // 流量*100
        golbal_flow.pluse_1s = 0;
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
