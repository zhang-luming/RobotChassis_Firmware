/**
 ******************************************************************************
 * @file    power_management.c
 * @brief   电源管理模块 - 电池电压监测
 *
 * 功能说明：
 * - 电池电压ADC采样
 * - 电压数据上报
 ******************************************************************************
 */

#include "power_management.h"
#include "comm_protocol.h"
#include "adc.h"

/* ==================== 私有变量 ==================== */

/* 电池电压值（mV） */
static uint16_t g_battery_voltage = 0;

/* ==================== 公共接口实现 ==================== */

/**
 * @brief 初始化电源管理模块
 */
void Power_Init(void) {
    g_battery_voltage = 0;

    /* ADC校准 */
    HAL_ADCEx_Calibration_Start(&hadc2);

    /* 开启ADC中断转换 */
    HAL_ADC_Start_IT(&hadc2);
}

/**
 * @brief 更新电池电压
 *
 * 功能：
 * - 启动ADC转换并读取值
 * - 转换为mV单位
 *
 */
void Power_Update(void) {
    /* 轮询转换 */
    HAL_ADC_PollForConversion(&hadc2, 50);

    /* 计算电池电压(mV)
     * 公式: (ADC值 / 4096) * 3300 * 11
     * 3300: 参考电压 3.3V
     * 11: 分压系数
     */
    g_battery_voltage = ((float)HAL_ADC_GetValue(&hadc2) / 4096) * 3300 * 11;
}

/**
 * @brief 发送电池电压
 *
 * 发送电池电压到上位机
 */
void Power_Send(void) {
    int16_t voltage_data = (int16_t)g_battery_voltage;
    Comm_SendDataFrame(FUNC_BATTERY_VOLTAGE, &voltage_data, 1);
}
