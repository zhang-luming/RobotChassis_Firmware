/**
 ******************************************************************************
 * @file    servo_control.c
 * @brief   舵机控制模块 - PWM舵机控制
 ******************************************************************************
 */

#include "servo_control.h"
#include "user_config.h"
#include "tim.h"

/* ==================== 私有变量 ==================== */
static const uint32_t SERVO_CHANNEL[SERVO_COUNT] = {
    TIM_CHANNEL_2,  /* SERVO_ID_1 */
    TIM_CHANNEL_3   /* SERVO_ID_2 */
};

/* ==================== 核心接口实现 ==================== */

void Servo_Init(void) {
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

    /* 设置初始位置到中点 */
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 1500);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 1500);
}

void Servo_ProcessSetAngle(uint8_t servo_id, int16_t angle) {
    if (servo_id >= SERVO_COUNT) return;

    /* 角度限幅 */
    if (angle < SERVO_MIN_ANGLE) angle = SERVO_MIN_ANGLE;
    else if (angle > SERVO_MAX_ANGLE) angle = SERVO_MAX_ANGLE;

    /* 计算脉宽: pulse = (angle / 180) * 2000 + 500 */
    uint16_t pulse = SERVO_MIN_PULSE +
                     (uint16_t)((uint32_t)angle * (SERVO_MAX_PULSE - SERVO_MIN_PULSE) /
                                (SERVO_MAX_ANGLE - SERVO_MIN_ANGLE));

    __HAL_TIM_SetCompare(&htim1, SERVO_CHANNEL[servo_id], pulse);
}

/* ==================== 扩展接口实现（预留） ==================== */

void Servo_MoveTo(uint8_t servo_id, int16_t angle, uint16_t speed) {
    /* 预留接口：未实现 */
    (void)servo_id; (void)angle; (void)speed;
}

int16_t Servo_GetAngle(uint8_t servo_id) {
    /* 预留接口：未实现 */
    (void)servo_id;
    return 0;
}

void Servo_SetEnable(uint8_t servo_id, uint8_t enable) {
    /* 预留接口：未实现 */
    (void)servo_id; (void)enable;
}

void Servo_ResetToCenter(uint8_t servo_id) {
    if (servo_id < SERVO_COUNT) {
        Servo_ProcessSetAngle(servo_id, 90);
    }
}

void Servo_SetCalibration(uint8_t servo_id, int16_t offset) {
    /* 预留接口：未实现 */
    (void)servo_id; (void)offset;
}

void Servo_SetMultiple(int16_t *angles, uint8_t count) {
    /* 预留接口：未实现 */
    (void)angles; (void)count;
}

void Servo_Update(void) {
    /* 预留接口：用于平滑移动等定时更新功能 */
}

void Servo_Send(void) {
    /* 预留接口：用于上报舵机状态 */
}
