#ifndef __EXIT_H
#define __EXIT_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== 函数接口 ==================== */

/**
 * @brief TIM6定时器中断处理函数（由TIM6_IRQHandler调用）
 * @note 设置10ms时基标志，供主循环使用
 */
void Time_TIM6IRQHandler(void);

/**
 * @brief 判断10ms定时时间是否到
 * @return 1-时间到, 0-时间未到
 */
uint8_t Judge_Time_OUT(void);

/**
 * @brief 微秒延时函数
 * @param udelay 延时时间（微秒）
 */
void delay_us(uint32_t udelay);

#ifdef __cplusplus
}
#endif

#endif /* __EXIT_H */
