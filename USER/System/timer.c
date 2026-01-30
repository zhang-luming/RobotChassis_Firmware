/**
 ******************************************************************************
 * @file    timer.c
 * @brief   定时器模块实现
 ******************************************************************************
 */

#include "timer.h"
#include "tim.h"

/* ==================== TIM6: 10ms系统时基 ==================== */

static volatile uint8_t g_tim6_timeout = 0;

void Timer_TIM6IRQHandler(void) {
  g_tim6_timeout = 1;
}

uint8_t Timer_IsTim6Timeout(void) {
  if (g_tim6_timeout) {
    g_tim6_timeout = 0;
    return 1;
  }
  return 0;
}

void HAL_TIM_PeriodElapsedCallback(const TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM6) {
    Timer_TIM6IRQHandler();
  } else if (htim->Instance == TIM7) {
    Time_TIM7IRQHandler();
  }
}

/* ==================== TIM7: 64位微秒时间戳 ==================== */

/* 时间戳高48位（低16位为TIM7->CNT），每65.536ms溢出递增
 * 注意：使用uint64_t类型确保可以递增到48位 */
static volatile uint64_t g_time_high48 = 0;

void Time_Init(void) {
  g_time_high48 = 0;
  HAL_TIM_Base_Start_IT(&htim7);
}

/**
 * @brief 获取64位微秒时间戳（原子版本，用于中断）
 *
 * 在中断中调用时，禁用中断确保读取原子性
 *
 * @return 64位微秒时间戳
 */
uint64_t Time_GetUs(void) {
  uint16_t cnt_low;
  uint64_t high;

  /* 禁用中断确保原子读取 */
  __disable_irq();

  cnt_low = __HAL_TIM_GET_COUNTER(&htim7);
  high = g_time_high48;

  /* 处理溢出边界：如果CNT很小，可能刚发生溢出，重新读取高48位 */
  if (cnt_low < 32768) {
    uint64_t high_check = g_time_high48;
    if (high_check != high) {
      high = high_check;
    }
  }

  /* 恢复中断 */
  __enable_irq();

  return (high << 16) | cnt_low;
}

void Time_TIM7IRQHandler(void) {
  g_time_high48++;
}

/* ==================== 工具函数 ==================== */

void delay_us(uint32_t udelay) {
  __IO uint32_t Delay = udelay * 72 / 8;  /* SystemCoreClock/8/1M */
  do {
    __NOP();
  } while (Delay--);
}
