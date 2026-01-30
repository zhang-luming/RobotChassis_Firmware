/**
 ******************************************************************************
 * @file    led_control.h
 * @brief   LED控制模块
 ******************************************************************************
 */

#ifndef __LED_CONTROL_H
#define __LED_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ==================== 类型定义 ==================== */
typedef enum {
    LED_STATE_OFF = 0,
    LED_STATE_ON,
    LED_STATE_SLOW_BLINK,     /* 400ms周期 */
    LED_STATE_FAST_BLINK,     /* 100ms周期 */
    LED_STATE_HEARTBEAT,      /* 2000ms周期 */
    LED_STATE_ERROR,          /* 100ms快闪 */
} LedState_t;

/* ==================== 核心接口 ==================== */
void LED_Init(void);
void LED_Update(void);

/* ==================== 状态控制 ==================== */
void LED_Off(void);
void LED_On(void);
void LED_SlowBlink(void);
void LED_FastBlink(void);
void LED_Heartbeat(void);
void LED_Error(void);

#ifdef __cplusplus
}
#endif

#endif /* __LED_CONTROL_H */
