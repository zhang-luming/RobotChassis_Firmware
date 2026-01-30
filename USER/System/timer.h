/**
 ******************************************************************************
 * @file    timer.h
 * @brief   定时器模块
 *
 * 功能说明：
 * - TIM6: 10ms系统时基
 * - TIM7: 64位微秒时间戳（~584942年溢出周期）
 * - delay_us(): 微秒延时
 ******************************************************************************
 */

#ifndef __TIMER_H__
#define __TIMER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* ==================== TIM6: 10ms系统时基 ==================== */

void Timer_TIM6IRQHandler(void);
uint8_t Timer_IsTim6Timeout(void);

/* ==================== TIM7: 64位微秒时间戳 ==================== */

/**
 * @brief 初始化时间戳模块
 */
void Time_Init(void);

/**
 * @brief 获取64位微秒时间戳
 * @return 64位微秒时间戳
 *
 * @note 分辨率1微秒，溢出周期约584942年
 */
uint64_t Time_GetUs(void);

/**
 * @brief TIM7中断处理函数（由HAL_TIM_PeriodElapsedCallback调用）
 */
void Time_TIM7IRQHandler(void);

/* ==================== 工具函数 ==================== */

/**
 * @brief 微秒延时函数
 * @param udelay 延时时间（微秒）
 */
void delay_us(uint32_t udelay);

#ifdef __cplusplus
}
#endif

#endif /* __TIMER_H__ */
