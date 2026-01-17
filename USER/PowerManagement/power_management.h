#ifndef __POWER_MANAGEMENT_H
#define __POWER_MANAGEMENT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ==================== 函数接口 ==================== */

/**
 * @brief 初始化电源管理模块
 */
void Power_Init(void);

/**
 * @brief 更新电池电压采样
 */
void Power_UpdateVoltage(void);

/**
 * @brief 获取电池电压（mV）
 * @return 电池电压值（mV）
 */
uint16_t Power_GetBatteryVoltage(void);

#ifdef __cplusplus
}
#endif

#endif /* __POWER_MANAGEMENT_H */
