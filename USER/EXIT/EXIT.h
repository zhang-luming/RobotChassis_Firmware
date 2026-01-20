#ifndef __EXIT_H
#define __EXIT_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== 函数接口 ==================== */

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
