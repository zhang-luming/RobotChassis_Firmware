/**
 ******************************************************************************
 * @file    timestamp.h
 * @brief   微秒时间戳模块
 *
 * 功能说明：
 * - 提供32位微秒级时间戳
 * - 基于TIM7定时器（1MHz计数频率）
 * - 低16位由硬件计数器CNT提供
 * - 高16位由溢出中断维护
 * - 最小化中断开销
 *
 * 时间范围：
 * - 32位时间戳：约1.2小时溢出周期
 * - 分辨率：1微秒
 *
 * 使用示例：
 *   uint32_t t1 = Time_GetUs();
 *   // ... 执行操作 ...
 *   uint32_t t2 = Time_GetUs();
 *   uint32_t elapsed = t2 - t1;  // 经过的微秒数
 ******************************************************************************
 */

#ifndef __TIMESTAMP_H__
#define __TIMESTAMP_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* ==================== 时间戳接口 ==================== */

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

/**
 * @brief TIM7溢出中断回调
 *
 * @note 在HAL_TIM_PeriodElapsedCallback中调用
 */
void Time_TIM7IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* __TIMESTAMP_H__ */
