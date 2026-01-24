/**
 ******************************************************************************
 * @file    timer.h
 * @brief   统一定时器模块
 *
 * 功能说明：
 * - TIM6: 10ms系统时基管理
 * - TIM7: 微秒时间戳（32位，约1.2小时溢出周期）
 * - 微秒延时函数
 *
 * 时间范围：
 * - 32位时间戳：约1.2小时溢出周期
 * - 分辨率：1微秒
 *
 * 使用示例：
 *   // 判断10ms时基
 *   if (Timer_IsTim6Timeout()) {
 *       // 每10ms执行一次
 *   }
 *
 *   // 微秒时间戳
 *   uint32_t t1 = Time_GetUs();
 *   // ... 执行操作 ...
 *   uint32_t t2 = Time_GetUs();
 *   uint32_t elapsed = Time_Elapsed(t1, t2);  // 经过的微秒数
 ******************************************************************************
 */

#ifndef __TIMER_H__
#define __TIMER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* ==================== TIM6：10ms系统时基 ==================== */

/**
 * @brief TIM6中断处理函数（由HAL_TIM_PeriodElapsedCallback调用）
 * @note 设置10ms时基标志，供主循环使用
 */
void Timer_TIM6IRQHandler(void);

/**
 * @brief 判断10ms定时时间是否到
 * @return 1-时间到, 0-时间未到
 */
uint8_t Timer_IsTim6Timeout(void);

/* ==================== TIM7：微秒时间戳 ==================== */

/**
 * @brief 初始化时间戳模块
 *
 * 功能：
 * - 配置TIM7为1MHz计数频率
 * - 启动定时器
 * - 使能溢出中断
 *
 * 注意：需要在系统初始化时调用一次
 */
void Time_Init(void);

/**
 * @brief 获取当前32位微秒时间戳
 *
 * 功能：
 * - 读取TIM7->CNT作为低16位
 * - 读取软件维护的高16位
 * - 处理溢出边界情况
 *
 * @return 32位微秒时间戳
 *
 * @note 此函数是中断安全的，可以在中断和主循环中调用
 * @note 时间戳约1.2小时溢出一次，长时间间隔需处理溢出
 */
uint32_t Time_GetUs(void);

/**
 * @brief TIM7中断处理函数（由HAL_TIM_PeriodElapsedCallback调用）
 * @note 递增微秒时间戳高16位
 */
void Time_TIM7IRQHandler(void);

/* ==================== 时间戳工具函数 ==================== */

/**
 * @brief 计算两个时间戳之间的差值
 * @param start 起始时间戳
 * @param end 结束时间戳
 * @return 经过的微秒数（自动处理32位溢出）
 *
 * @note 此函数正确处理时间戳溢出的情况
 */
static inline uint32_t Time_Elapsed(uint32_t start, uint32_t end) {
    return end - start;
}

/**
 * @brief 微秒转毫秒
 * @param us 微秒值
 * @return 毫秒值
 */
static inline uint32_t Time_UsToMs(uint32_t us) {
    return us / 1000;
}

/**
 * @brief 毫秒转微秒
 * @param ms 毫秒值
 * @return 微秒值
 */
static inline uint32_t Time_MsToUs(uint32_t ms) {
    return ms * 1000;
}

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
