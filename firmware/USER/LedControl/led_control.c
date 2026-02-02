/**
 ******************************************************************************
 * @file    led_control.c
 * @brief   LED控制模块实现
 ******************************************************************************
 */

#include "led_control.h"

/* ==================== 私有变量 ==================== */
static LedState_t g_state = LED_STATE_OFF;
static uint16_t g_counter = 0;

/* ==================== 私有函数 ==================== */

static void led_set(LedState_t state) {
    g_state = state;
    g_counter = 0;
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
}

/* ==================== 公共接口实现 ==================== */

void LED_Init(void) {
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
}

void LED_Update(void) {
    g_counter++;

    switch (g_state) {
        case LED_STATE_OFF:
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
            break;

        case LED_STATE_ON:
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
            break;

        case LED_STATE_SLOW_BLINK:
            if (g_counter >= 20) {
                g_counter = 0;
                HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
            }
            break;

        case LED_STATE_FAST_BLINK:
        case LED_STATE_ERROR:
            if (g_counter >= 5) {
                g_counter = 0;
                HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
            }
            break;

        case LED_STATE_HEARTBEAT:
            if (g_counter == 5) {
                HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
            } else if (g_counter >= 200) {
                g_counter = 0;
                HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
            }
            break;

        default:
            break;
    }
}

/* ==================== 状态控制接口 ==================== */

void LED_Off(void) {
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    g_state = LED_STATE_OFF;
}

void LED_On(void) {
    led_set(LED_STATE_ON);
}

void LED_SlowBlink(void) {
    led_set(LED_STATE_SLOW_BLINK);
}

void LED_FastBlink(void) {
    led_set(LED_STATE_FAST_BLINK);
}

void LED_Heartbeat(void) {
    led_set(LED_STATE_HEARTBEAT);
}

void LED_Error(void) {
    led_set(LED_STATE_ERROR);
}
