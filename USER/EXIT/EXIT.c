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
 * @brief TIM6定时器中断回调
 * @param htim TIM句柄
 */
void HAL_TIM_PeriodElapsedCallback(const TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM6)  /* 定时器6基地址 */
    {
        tim6_timeout = 1;
    }
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
