/**
 ******************************************************************************
 * @file    led_control.c
 * @brief   LED控制模块 - LED状态指示
 *
 * 功能说明：
 * - LED状态控制
 * - 多种闪烁模式（内部管理定时）
 ******************************************************************************
 */

#include "led_control.h"

/* ==================== 私有变量 ==================== */

static LedState_t g_current_state = LED_STATE_OFF;
static uint16_t g_blink_counter = 0; /* 内部计数器，每10ms递增 */
static uint8_t g_led_status = 0;     /* 0-灭, 1-亮 */

/* ==================== 私有函数 ==================== */

/**
 * @brief 切换LED状态（内部使用）
 */
static void LED_Toggle(void) {
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    g_led_status = !g_led_status;
}

/* ==================== 公共接口实现 ==================== */

/**
 * @brief 初始化LED控制模块
 */
void LED_Init(void) {
    g_current_state = LED_STATE_OFF;
    g_blink_counter = 0;
    g_led_status = 0;

    /* 初始状态：点亮LED表示系统运行 */
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET); /* 低电平点亮 */
    g_led_status = 1;
}

/**
 * @brief 更新LED状态（每10ms调用一次）
 *
 * 功能：
 * - 内部管理定时计数器
 * - 根据当前状态更新LED
 * - 处理各种闪烁模式
 */
void LED_Update(void) {
    g_blink_counter++;

    switch (g_current_state) {
        case LED_STATE_OFF:
            /* 保持关闭 */
            break;

        case LED_STATE_ON:
            /* 保持常亮 */
            break;

        case LED_STATE_SLOW_BLINK:
            /* 慢闪：400ms周期 (200ms亮, 200ms灭) */
            if (g_blink_counter >= 20) {
                /* 20 * 10ms = 200ms */
                g_blink_counter = 0;
                LED_Toggle();
            }
            break;

        case LED_STATE_FAST_BLINK:
            /* 快闪：100ms周期 (50ms亮, 50ms灭) */
            if (g_blink_counter >= 5) {
                /* 5 * 10ms = 50ms */
                g_blink_counter = 0;
                LED_Toggle();
            }
            break;

        case LED_STATE_HEARTBEAT:
            /* 心跳：2000ms周期 (100ms亮, 1900ms灭) */
            if (g_blink_counter == 10) {
                /* 100ms时关闭 */
                HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
                g_led_status = 0;
            } else if (g_blink_counter >= 200) {
                /* 2000ms时点亮并重置 */
                g_blink_counter = 0;
                HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
                g_led_status = 1;
            }
            break;

        case LED_STATE_ERROR:
            /* 错误提示：50ms快闪 */
            if (g_blink_counter >= 2) {
                /* 2 * 10ms = 20ms (实际约40ms周期) */
                g_blink_counter = 0;
                LED_Toggle();
            }
            break;

        default:
            break;
    }
}

/* ==================== 状态控制接口 ==================== */

/**
 * @brief 关闭LED
 */
void LED_Off(void) {
    g_current_state = LED_STATE_OFF;
    g_blink_counter = 0;
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    g_led_status = 0;
}

/**
 * @brief 打开LED（常亮）
 */
void LED_On(void) {
    g_current_state = LED_STATE_ON;
    g_blink_counter = 0;
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
    g_led_status = 1;
}

/**
 * @brief 慢闪模式（400ms周期）
 */
void LED_SlowBlink(void) {
    g_current_state = LED_STATE_SLOW_BLINK;
    g_blink_counter = 0;
    /* 初始状态：点亮 */
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
    g_led_status = 1;
}

/**
 * @brief 快闪模式（100ms周期）
 */
void LED_FastBlink(void) {
    g_current_state = LED_STATE_FAST_BLINK;
    g_blink_counter = 0;
    /* 初始状态：点亮 */
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
    g_led_status = 1;
}

/**
 * @brief 心跳模式（2000ms周期：100ms亮，1900ms灭）
 */
void LED_Heartbeat(void) {
    g_current_state = LED_STATE_HEARTBEAT;
    g_blink_counter = 0;
    /* 初始状态：点亮 */
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
    g_led_status = 1;
}

/**
 * @brief 错误模式（50ms快闪）
 */
void LED_Error(void) {
    g_current_state = LED_STATE_ERROR;
    g_blink_counter = 0;
    /* 初始状态：点亮 */
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
    g_led_status = 1;
}
