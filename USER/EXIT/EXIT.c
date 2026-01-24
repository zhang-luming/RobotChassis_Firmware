#include "main.h"
#include "EXIT.h"

/* ==================== 私有变量 ==================== */

/* TIM6中断标志 */
static uint8_t tim6_timeout = 0;

/* ==================== 工具函数 ==================== */

/**
 * @brief 微秒延时函数
 * @param udelay 延时时间（微秒）
 */
void delay_us(uint32_t udelay)
{
  __IO uint32_t Delay = udelay * 72 / 8;  /* SystemCoreClock / 8U / 1000000U */
  do
  {
    __NOP();
  }
  while (Delay --);
}

/* ==================== 函数实现 ==================== */

/**
 * @brief TIM6定时器中断处理函数（由TIM6_IRQHandler调用）
 * @note 设置10ms时基标志，供主循环使用
 */
void Time_TIM6IRQHandler(void)
{
    tim6_timeout = 1;
}

/**
 * @brief TIM定时器中断回调（HAL库弱定义回调）
 * @param htim TIM句柄
 *
 * 注意：TIM6中断已改用Time_TIM6IRQHandler()直接处理
 * 此回调保留给其他可能使用HAL定时器回调的场景
 */
void HAL_TIM_PeriodElapsedCallback(const TIM_HandleTypeDef *htim)
{
    /* TIM6已改用直接调用方式，不再在此处理 */
    /* 如需处理其他定时器，可在此添加 */
}

/**
 * @brief 判断10ms定时时间是否到
 * @return 1-时间到, 0-时间未到
 */
uint8_t Judge_Time_OUT(void)
{
    if(tim6_timeout)
    {
        tim6_timeout = 0;
        return 1;
    }
    else
    {
        return 0;
    }
}
