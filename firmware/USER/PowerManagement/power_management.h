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
 * - Power_Update(): 更新电压
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
 */
void Power_Update(void);

/**
 * @brief 发送电池电压
 *
 * 功能：
 * - 发送电池电压到上位机
 *
 */
void Power_Send(void);


#ifdef __cplusplus
}
#endif

#endif /* __POWER_MANAGEMENT_H */
