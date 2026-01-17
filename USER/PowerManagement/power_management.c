#include "power_management.h"
#include "adc.h"

/* ==================== 私有变量 ==================== */

/* 电池电压值（mV） */
static uint16_t battery_voltage = 0;

/* ==================== 函数实现 ==================== */

/**
 * @brief 初始化电源管理模块
 */
void Power_Init(void)
{
    battery_voltage = 0;

    /* ADC校准 */
    HAL_ADCEx_Calibration_Start(&hadc2);

    /* 开启ADC中断转换 */
    HAL_ADC_Start_IT(&hadc2);
}

/**
 * @brief 更新电池电压采样
 */
void Power_UpdateVoltage(void)
{
    /* 轮询转换 */
    HAL_ADC_PollForConversion(&hadc2, 50);

    /* 计算电池电压(mV)
     * 公式: (ADC值 / 4096) * 3300 * 11
     * 3300: 参考电压 3.3V
     * 11: 分压系数
     */
    battery_voltage = ((float)HAL_ADC_GetValue(&hadc2) / 4096) * 3300 * 11;
}

/**
 * @brief 获取电池电压（mV）
 */
uint16_t Power_GetBatteryVoltage(void)
{
    return battery_voltage;
}
