#include "servo_control.h"
#include "user_config.h"
#include "tim.h"

/* ==================== 函数实现 ==================== */

/**
 * @brief 初始化舵机控制模块
 */
void Servo_Init(void)
{
    /* 启动舵机PWM */
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

    /* 设置初始位置（90度） */
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 1500);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 1500);
}

/**
 * @brief 设置舵机角度
 * @param servo_id 舵机ID (0-1)
 * @param angle 角度 (0-180度)
 */
void Servo_SetAngle(uint8_t servo_id, uint8_t angle)
{
    uint16_t pulse;

    /* 角度限幅 */
    if(angle > SERVO_MAX_ANGLE)
    {
        angle = SERVO_MAX_ANGLE;
    }

    /* 计算脉宽: 0度->500us, 180度->2500us
     * 公式: pulse = 2000 * angle / 180 + 500
     */
    pulse = 2000 * angle / 180 + 500;

    /* 设置PWM比较值 */
    if(servo_id == SERVO_ID_1)
    {
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, pulse);
    }
    else if(servo_id == SERVO_ID_2)
    {
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, pulse);
    }
}
