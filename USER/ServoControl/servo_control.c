/**
 ******************************************************************************
 * @file    servo_control.c
 * @brief   舵机控制模块 - PWM舵机控制
 *
 * 功能说明：
 * - 双路舵机PWM控制
 * - 舵机角度设置
 ******************************************************************************
 */

#include "servo_control.h"
#include "user_config.h"
#include "tim.h"

/* ==================== 公共接口实现 ==================== */

/**
 * @brief 初始化舵机控制模块
 */
void Servo_Init(void) {
    /* 启动舵机PWM */
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

    /* 设置初始位置（90度） */
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 1500);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 1500);
}

/**
 * @brief 更新舵机状态（预留接口）
 *
 * 注意：预留接口，当前为空函数
 */
void Servo_Update(void) {
    /* 舵机目前是开环控制，无需Update */
    /* 预留给未来闭环控制使用 */
}

/**
 * @brief 发送舵机状态（预留接口）
 *
 * 注意：预留接口，当前为空函数
 */
void Servo_Send(void) {
    /* 预留接口，当前无数据发送 */
}

/* ==================== 控制指令处理 ==================== */

/**
 * @brief 处理舵机角度控制指令
 * @param servo_id 舵机ID (0-1)
 * @param angle 角度值 (0-180度)
 */
void Servo_ProcessSetAngle(uint8_t servo_id, int16_t angle) {
    /* 角度限幅 */
    if (angle > SERVO_MAX_ANGLE) {
        angle = SERVO_MAX_ANGLE;
    }
    if (angle < 0) {
        angle = 0;
    }

    /* 计算脉宽: 0度->500us, 180度->2500us
     * 公式: pulse = 2000 * angle / 180 + 500
     */
    uint16_t pulse = 2000 * angle / 180 + 500;

    /* 设置PWM比较值 */
    if (servo_id == SERVO_ID_1) {
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, pulse);
    } else if (servo_id == SERVO_ID_2) {
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, pulse);
    }
}
