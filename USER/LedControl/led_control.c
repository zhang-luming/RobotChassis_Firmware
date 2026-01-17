#include "led_control.h"

/* ==================== 私有变量 ==================== */

static LedState_t current_state = LED_STATE_OFF;
static uint16_t blink_counter = 0;
static uint8_t led_status = 0;  /* 0-灭, 1-亮 */

/* ==================== 函数实现 ==================== */

/**
 * @brief 初始化LED控制模块
 */
void LED_Init(void)
{
    current_state = LED_STATE_OFF;
    blink_counter = 0;
    led_status = 0;

    /* 初始状态：点亮LED表示系统运行 */
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);  /* 低电平点亮 */
    led_status = 1;
}

/**
 * @brief 设置LED状态
 */
void LED_SetState(LedState_t state)
{
    current_state = state;
    blink_counter = 0;

    /* 立即响应关闭和常亮状态 */
    if(state == LED_STATE_OFF)
    {
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
        led_status = 0;
    }
    else if(state == LED_STATE_ON)
    {
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
        led_status = 1;
    }
}

/**
 * @brief LED状态更新（在10ms定时器中调用）
 */
void LED_Update(void)
{
    blink_counter++;

    switch(current_state)
    {
        case LED_STATE_OFF:
            /* 保持关闭 */
            break;

        case LED_STATE_ON:
            /* 保持常亮 */
            break;

        case LED_STATE_SLOW_BLINK:
            /* 慢闪：400ms周期 (200ms亮, 200ms灭) */
            if(blink_counter >= 20)  /* 20 * 10ms = 200ms */
            {
                blink_counter = 0;
                LED_Toggle();
            }
            break;

        case LED_STATE_FAST_BLINK:
            /* 快闪：100ms周期 (50ms亮, 50ms灭) */
            if(blink_counter >= 5)  /* 5 * 10ms = 50ms */
            {
                blink_counter = 0;
                LED_Toggle();
            }
            break;

        case LED_STATE_HEARTBEAT:
            /* 心跳：2000ms周期 (100ms亮, 1900ms灭) */
            if(blink_counter == 10)  /* 100ms时关闭 */
            {
                HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
                led_status = 0;
            }
            else if(blink_counter >= 200)  /* 2000ms时点亮并重置 */
            {
                blink_counter = 0;
                HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
                led_status = 1;
            }
            break;

        case LED_STATE_ERROR:
            /* 错误提示：50ms快闪 */
            if(blink_counter >= 2)  /* 2 * 10ms = 20ms (实际约40ms周期) */
            {
                blink_counter = 0;
                LED_Toggle();
            }
            break;

        default:
            break;
    }
}

/**
 * @brief 直接控制LED开关
 */
void LED_SetDirect(uint8_t state)
{
    if(state)
    {
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
        led_status = 1;
    }
    else
    {
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
        led_status = 0;
    }
}

/**
 * @brief 切换LED状态
 */
void LED_Toggle(void)
{
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    led_status = !led_status;
}
