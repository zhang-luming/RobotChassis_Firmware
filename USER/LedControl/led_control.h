/**
 ******************************************************************************
 * @file    led_control.h
 * @brief   LED控制模块 - LED状态指示
 *
 * 功能说明：
 * - LED状态控制
 * - 多种闪烁模式（内部管理定时）
 *
 * 模块接口：
 * - LED_Init(): 初始化模块
 * - LED_Update(): 更新LED状态（每10ms调用一次）
 * - LED_Off/On/SlowBlink/FastBlink/Heartbeat/Error(): 设置LED状态
 ******************************************************************************
 */

#ifndef __LED_CONTROL_H
#define __LED_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ==================== 内部状态定义 ==================== */
typedef enum {
    LED_STATE_OFF = 0,        /* 关闭 */
    LED_STATE_ON,             /* 常亮 */
    LED_STATE_SLOW_BLINK,     /* 慢闪 (400ms周期) */
    LED_STATE_FAST_BLINK,     /* 快闪 (100ms周期) */
    LED_STATE_HEARTBEAT,      /* 心跳 (2000ms周期) */
    LED_STATE_ERROR,          /* 错误提示 (50ms快闪) */
} LedState_t;

/* ==================== 核心接口 ==================== */

/**
 * @brief 初始化LED控制模块
 */
void LED_Init(void);

/**
 * @brief 更新LED状态（每10ms调用一次）
 *
 * 功能：
 * - 内部管理定时计数器
 * - 根据当前状态更新LED
 * - 处理各种闪烁模式
 *
 * 注意：需要在主循环中每次调用
 */
void LED_Update(void);

/* ==================== 状态控制接口 ==================== */

/**
 * @brief 关闭LED
 *
 * 功能：关闭LED，不闪烁
 */
void LED_Off(void);

/**
 * @brief 打开LED（常亮）
 *
 * 功能：LED常亮，不闪烁
 */
void LED_On(void);

/**
 * @brief 慢闪模式（400ms周期）
 *
 * 功能：LED以400ms周期闪烁（200ms亮，200ms灭）
 */
void LED_SlowBlink(void);

/**
 * @brief 快闪模式（100ms周期）
 *
 * 功能：LED以100ms周期闪烁（50ms亮，50ms灭）
 */
void LED_FastBlink(void);

/**
 * @brief 心跳模式（2000ms周期）
 *
 * 功能：LED以2000ms周期闪烁（100ms亮，1900ms灭）
 */
void LED_Heartbeat(void);

/**
 * @brief 错误模式（50ms快闪）
 *
 * 功能：LED快速闪烁（约40ms周期），用于错误提示
 */
void LED_Error(void);

#ifdef __cplusplus
}
#endif

#endif /* __LED_CONTROL_H */
