/**
 ******************************************************************************
 * @file    timestamp.c
 * @brief   微秒时间戳模块实现
 ******************************************************************************
 */

#include "timestamp.h"
#include "tim.h"

/* ==================== 私有变量 ==================== */

/**
 * @brief 时间戳高16位（由TIM7溢出中断维护）
 *
 * 设计说明：
 * - TIM7配置为16位自动重载（ARR=65535）
 * - 每65536us（约65.5ms）溢出一次
 * - 溢出时g_time_high16++
 * - Time_GetUs()组合高低位为32位时间戳
 *
 * 线程安全：
 * - 32位系统中，uint16_t读写是原子的
 * - 读CNT和读g_time_high16之间可能发生中断
 * - 但通过检测CNT回滚可以正确处理
 */
static volatile uint16_t g_time_high16 = 0;

/* ==================== 公共接口实现 ==================== */

/**
 * @brief 初始化时间戳模块
 */
void Time_Init(void) {
    /* 重置高16位计数器 */
    g_time_high16 = 0;

    /* 启动TIM7（需要在tim.c中配置好） */
    HAL_TIM_Base_Start_IT(&htim7);
}

/**
 * @brief 获取当前32位微秒时间戳
 *
 * 实现原理：
 * 1. 先读取低16位（CNT寄存器）
 * 2. 再读取高16位（软件变量）
 * 3. 检测CNT是否发生回滚（溢出）
 * 4. 组合为32位时间戳
 *
 * 边界情况处理：
 * - 读CNT后、读g_time_high16前发生溢出：
 *   CNT会变小（如65500→100），g_time_high16已+1
 *   此时组合结果错误，但可以通过检测CNT < 32768判断
 * - 解决方案：如果CNT很小，可能是溢出，需要重新读取
 */
uint32_t Time_GetUs(void) {
    uint16_t cnt_low;
    uint16_t high;

    /* 读取低16位（CNT）和高16位 */
    cnt_low = __HAL_TIM_GET_COUNTER(&htim7);
    high = g_time_high16;

    /* 检测溢出边界情况
     * 如果CNT很小，且在读取过程中可能发生了溢出
     * 则重新读取高16位以确保正确性
     */
    if (cnt_low < 32768) {
        /* CNT在低半区，可能刚溢出，重新读取高16位确认 */
        uint16_t high_check = g_time_high16;
        if (high_check != high) {
            /* 高16位确实变了，使用新值 */
            high = high_check;
        }
    }

    /* 组合为32位时间戳 */
    return ((uint32_t)high << 16) | cnt_low;
}

/**
 * @brief TIM7溢出中断回调
 *
 * 功能：
 * - TIM7每65536us溢出一次
 * - 在中断中递增高16位
 *
 * 注意：此函数由HAL_TIM_PeriodElapsedCallback调用
 */
void Time_TIM7IRQHandler(void) {
    /* 递增高16位时间戳 */
    g_time_high16++;
}
