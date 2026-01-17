#ifndef __LED_CONTROL_H
#define __LED_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ==================== LED状态定义 ==================== */
typedef enum {
    LED_STATE_OFF = 0,        /* 关闭 */
    LED_STATE_ON,             /* 常亮 */
    LED_STATE_SLOW_BLINK,     /* 慢闪 (400ms周期) */
    LED_STATE_FAST_BLINK,     /* 快闪 (100ms周期) */
    LED_STATE_HEARTBEAT,      /* 心跳 (2000ms周期) */
    LED_STATE_ERROR,          /* 错误提示 (50ms快闪) */
} LedState_t;

/* ==================== 函数接口 ==================== */

/**
 * @brief 初始化LED控制模块
 */
void LED_Init(void);

/**
 * @brief 设置LED状态
 * @param state LED状态
 */
void LED_SetState(LedState_t state);

/**
 * @brief LED状态更新（在定时器中周期调用）
 * @note 建议在10ms定时器中调用
 */
void LED_Update(void);

/**
 * @brief 直接控制LED开关
 * @param state 0-关闭, 1-打开
 */
void LED_SetDirect(uint8_t state);

/**
 * @brief 切换LED状态
 */
void LED_Toggle(void);

#ifdef __cplusplus
}
#endif

#endif /* __LED_CONTROL_H */
