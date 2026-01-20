/**
 ******************************************************************************
 * @file    power_management.h
 * @brief   电源管理模块 - 电池电压监测
 *
 * 功能说明：
 * - 电池电压ADC采样
 * - 电压数据上报
 *
 * 模块接口：
 * - Power_Init(): 初始化模块
 * - Power_Update(): 每次调用更新电压（内部判断200ms采样）
 * - Power_Send(): 发送电池电压
 ******************************************************************************
 */

#ifndef __POWER_MANAGEMENT_H
#define __POWER_MANAGEMENT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ==================== 核心接口 ==================== */

/**
 * @brief 初始化电源管理模块
 */
void Power_Init(void);

/**
 * @brief 更新电池电压
 *
 * 功能：
 * - 启动ADC转换
 * - 读取电压值
 * - 转换为mV单位并存储
 *
 * 注意：此函数立即执行ADC采样，调用频率由main.c控制
 */
void Power_Update(void);

/**
 * @brief 发送电池电压
 *
 * 功能：
 * - 直接发送电池电压到上位机
 * - 不判断时间
 *
 * 注意：需要主循环控制发送时机（每200ms）
 */
void Power_Send(void);

/* ==================== 兼容性接口（临时） ==================== */

/**
 * @brief 兼容性接口：更新电池电压采样
 * @deprecated 请使用 Power_Update() 替代
 * @note 保留此函数以保持向后兼容
 */
void Power_UpdateVoltage(void);

/**
 * @brief 兼容性接口：获取电池电压
 * @deprecated 请使用 Power_Send() 替代
 * @note 保留此函数以保持向后兼容
 */
uint16_t Power_GetBatteryVoltage(void);

#ifdef __cplusplus
}
#endif

#endif /* __POWER_MANAGEMENT_H */
