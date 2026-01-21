/**
 ******************************************************************************
 * @file    motor_control.c
 * @brief   电机控制模块 - 四路电机PID控制
 *
 * 功能说明：
 * - 四路编码器速度读取
 * - PID闭环控制
 * - PWM输出控制
 * - 编码器数据上报
 * - 动态时间计算（自适应调用周期）
 *
 * 模块接口：
 * - Motor_Init(): 初始化模块
 * - Motor_Update(): 每次调用执行编码器读取+PID控制+PWM输出
 * - Motor_Send(): 发送编码器数据
 * - Motor_ProcessSetSpeed(): 设置目标速度
 * - Motor_ProcessSetPID(): 设置PID参数
 ******************************************************************************
 */

#include "motor_control.h"
#include "comm_protocol.h"
#include "user_config.h"
#include "tim.h"
#include "timestamp.h"

/* ==================== 私有变量 ==================== */

/* 目标速度（单位：CPS - Counts Per Second，编码器计数/秒） */
static int16_t g_target_speeds[MOTOR_COUNT] = {0};

/* 编码器累加值 */
static int16_t g_encoder_accum[MOTOR_COUNT] = {0};

/* 编码器上一次的值 */
static int16_t g_encoder_last[MOTOR_COUNT] = {ENCODER_MIDDLE, ENCODER_MIDDLE, ENCODER_MIDDLE, ENCODER_MIDDLE};

/* 编码器相对变化值（速度） */
static int16_t g_encoder_deltas[MOTOR_COUNT] = {0};

/* 上次调用时间戳（微秒） */
static uint32_t g_last_update_time_us = 0;

/* 实际调用周期（毫秒）- 动态计算 */
static float g_update_period_ms = 10.0f;

/* PID状态结构体 */
typedef struct {
    int16_t kp;             /* 比例系数（从外部变量获取） */
    int16_t ki;             /* 积分系数（从外部变量获取） */
    int16_t kd;             /* 微分系数（从外部变量获取） */
    int32_t bias;           /* 当前误差 */
    int32_t bias_last;      /* 上次误差 */
    int32_t bias_integral;  /* 误差累加 */
    int16_t pwm_out;        /* PWM输出值 */
} MotorPID_t;

/* 4个电机的PID状态 */
static MotorPID_t g_motor_pid[MOTOR_COUNT] = {0};

/* 外部PID参数定义（在main.c中定义） */
extern int16_t PID_P;
extern int16_t PID_I;
extern int16_t PID_D;

/* ==================== 私有函数前向声明 ==================== */
static int16_t Motor_PIDControl(uint8_t motor_id, int16_t target_speed, int16_t actual_speed);
static void Motor_SetSpeed(uint8_t motor_id, int16_t speed);

/* ==================== 公共接口实现 ==================== */

/**
 * @brief 初始化电机控制模块
 */
void Motor_Init(void) {
    /* 重置编码器计数器到中间值 */
    TIM2->CNT = ENCODER_MIDDLE;
    TIM3->CNT = ENCODER_MIDDLE;
    TIM4->CNT = ENCODER_MIDDLE;
    TIM5->CNT = ENCODER_MIDDLE;

    /* 启动编码器捕获 */
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);

    /* 启动电机PWM输出 */
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);

    /* 初始化PID状态 */
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        g_motor_pid[i].kp = PID_P;
        g_motor_pid[i].ki = PID_I;
        g_motor_pid[i].kd = PID_D;
        g_motor_pid[i].bias = 0;
        g_motor_pid[i].bias_last = 0;
        g_motor_pid[i].bias_integral = 0;
        g_motor_pid[i].pwm_out = 0;
    }

    /* 初始化时间戳 */
    g_last_update_time_us = Time_GetUs();
}

/**
 * @brief 更新电机控制模块（自适应调用周期）
 *
 * 功能：
 * 1. 动态计算与上次调用的时间差
 * 2. 读取四路编码器当前值
 * 3. 计算编码器增量（相对速度，带溢出检测）
 * 4. 对每个电机执行PID控制
 * 5. 输出PWM到电机驱动
 *
 * 注意：
 * - 自动适应不同的调用周期（10ms、20ms、40ms等）
 * - 动态更新g_update_period_ms用于速度转换
 */
void Motor_Update(void) {
    int16_t current_counter;
    uint32_t current_time_us;
    uint32_t elapsed_us;

    /* 1. 计算时间差（动态） */
    current_time_us = Time_GetUs();
    elapsed_us = Time_Elapsed(g_last_update_time_us, current_time_us);
    g_last_update_time_us = current_time_us;

    /* 转换为毫秒（浮点数，用于精确计算） */
    g_update_period_ms = elapsed_us / 1000.0f;

    /* 2. 读取当前编码器值并计算增量（带溢出检测） */

    /* 电机A - TIM2 */
    current_counter = __HAL_TIM_GET_COUNTER(&htim2);
    int16_t delta0 = current_counter - g_encoder_last[0];
    if (delta0 < -32767) {  /* 正转溢出：65535→0 */
        delta0 += 65536;
    } else if (delta0 > 32767) {  /* 反转溢出：0→65535 */
        delta0 -= 65536;
    }
    g_encoder_deltas[0] = delta0;

    /* 电机B - TIM3 */
    current_counter = __HAL_TIM_GET_COUNTER(&htim3);
    int16_t delta1 = current_counter - g_encoder_last[1];
    if (delta1 < -32767) {
        delta1 += 65536;
    } else if (delta1 > 32767) {
        delta1 -= 65536;
    }
    g_encoder_deltas[1] = -delta1;  /* TIM3方向相反 */

    /* 电机C - TIM4 */
    current_counter = __HAL_TIM_GET_COUNTER(&htim4);
    int16_t delta2 = current_counter - g_encoder_last[2];
    if (delta2 < -32767) {
        delta2 += 65536;
    } else if (delta2 > 32767) {
        delta2 -= 65536;
    }
    g_encoder_deltas[2] = delta2;

    /* 电机D - TIM5 */
    current_counter = __HAL_TIM_GET_COUNTER(&htim5);
    int16_t delta3 = current_counter - g_encoder_last[3];
    if (delta3 < -32767) {
        delta3 += 65536;
    } else if (delta3 > 32767) {
        delta3 -= 65536;
    }
    g_encoder_deltas[3] = -delta3;  /* TIM5方向相反 */

    /* 3. 重置编码器计数器到中间值 */
    TIM2->CNT = ENCODER_MIDDLE;
    TIM3->CNT = ENCODER_MIDDLE;
    TIM4->CNT = ENCODER_MIDDLE;
    TIM5->CNT = ENCODER_MIDDLE;
    g_encoder_last[0] = ENCODER_MIDDLE;
    g_encoder_last[1] = ENCODER_MIDDLE;
    g_encoder_last[2] = ENCODER_MIDDLE;
    g_encoder_last[3] = ENCODER_MIDDLE;

    /* 4. 计算编码器累加值 */
    g_encoder_accum[0] += g_encoder_deltas[0];
    g_encoder_accum[1] += g_encoder_deltas[1];
    g_encoder_accum[2] += g_encoder_deltas[2];
    g_encoder_accum[3] += g_encoder_deltas[3];

    /* 5. PID控制并输出PWM */
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        int16_t pwm = Motor_PIDControl(i, g_target_speeds[i], g_encoder_deltas[i]);
        Motor_SetSpeed(i, pwm);
    }
}

/**
 * @brief 发送电机编码器数据
 *
 * 直接发送编码器累加值到上位机，不判断时间
 */
void Motor_Send(void) {
    Comm_SendEncoder(g_encoder_accum);
}

/**
 * @brief 处理电机速度控制指令
 * @param motor_id 电机ID (0-3)
 * @param speed 目标速度（单位：centi-CPS - CPS/100，编码器计数/秒÷100）
 *
 * 功能：
 * - 将centi-CPS转换为编码器增量并存储
 * - 转换公式使用动态计算的周期：
 *   encoder_delta = (speed × 100) × g_update_period_ms / 1000
 *   = speed × g_update_period_ms / 10
 *
 * 注意：
 * - 使用centi-CPS（CPS/100）单位以支持更大的速度范围
 * - g_update_period_ms是动态计算的，自适应不同调用周期
 * - 协议中传输的是CPS除以100后的值，例如：
 *   - 实际100000 CPS → 协议中发送1000
 *   - 实际50000 CPS → 协议中发送500
 *   - 实际1000 CPS → 协议中发送10
 *
 * 示例（动态周期）：
 * - 10ms调用周期，协议值10 → 10 × 10 / 10 = 10 counts/10ms
 * - 20ms调用周期，协议值10 → 10 × 20 / 10 = 20 counts/20ms
 * - 40ms调用周期，协议值10 → 10 × 40 / 10 = 40 counts/40ms
 */
void Motor_ProcessSetSpeed(uint8_t motor_id, int16_t speed) {
    if (motor_id < MOTOR_COUNT) {
        /* centi-CPS转换为编码器增量（使用动态周期） */
        /* speed单位是CPS/100，需要乘以100转换为CPS，再转换为增量 */
        g_target_speeds[motor_id] = (int16_t)(speed * g_update_period_ms / 10.0f);
    }
}

/**
 * @brief 处理PID参数设置指令
 * @param motor_id 电机ID
 * @param kp P参数（放大100倍）
 * @param ki I参数（放大100倍）
 * @param kd D参数（放大100倍）
 */
void Motor_ProcessSetPID(uint8_t motor_id, int16_t kp, int16_t ki, int16_t kd) {
    if (motor_id < MOTOR_COUNT) {
        g_motor_pid[motor_id].kp = kp;
        g_motor_pid[motor_id].ki = ki;
        g_motor_pid[motor_id].kd = kd;

        /* 重置积分和误差历史，避免参数切换时的突变 */
        g_motor_pid[motor_id].bias_integral = 0;
        g_motor_pid[motor_id].bias_last = 0;

        /* 同时更新外部全局变量（保持兼容性） */
        if (motor_id == 0) {
            PID_P = kp;
            PID_I = ki;
            PID_D = kd;
        }
    }
}

/* ==================== 私有函数实现 ==================== */

/**
 * @brief 电机PID控制算法（增量式累加）
 * @param motor_id 电机ID
 * @param target_speed 目标速度
 * @param actual_speed 实际速度（编码器增量）
 * @return PWM输出值
 */
static int16_t Motor_PIDControl(uint8_t motor_id, int16_t target_speed, int16_t actual_speed) {
    if (motor_id >= MOTOR_COUNT) {
        return 0;
    }

    MotorPID_t *pid = &g_motor_pid[motor_id];

    if (target_speed == 0) {
        pid->pwm_out = 0;
        pid->bias = 0;
        pid->bias_last = 0;
        pid->bias_integral = 0;
    } else {
        /* 计算当前误差 */
        pid->bias = target_speed - actual_speed;

        /* 积分项累加 */
        pid->bias_integral += pid->bias;

        /* 积分限幅（防止积分饱和） */
        if (pid->bias_integral > 1000) {
            pid->bias_integral = 1000;
        } else if (pid->bias_integral < -1000) {
            pid->bias_integral = -1000;
        }

        /* PID增量计算并累加到PWM输出 */
        pid->pwm_out += (int16_t)(pid->kp * pid->bias * PID_SCALE
                     + pid->ki * pid->bias_integral * PID_SCALE
                     + pid->kd * (pid->bias - pid->bias_last) * PID_SCALE);

        /* 保存上次误差 */
        pid->bias_last = pid->bias;
    }

    return pid->pwm_out;
}

/**
 * @brief 设置电机PWM输出
 * @param motor_id 电机ID (0-3)
 * @param speed PWM值（正值正转，负值反转，0停止）
 */
static void Motor_SetSpeed(uint8_t motor_id, int16_t speed) {
    if (speed == 0) {
        /* 停止电机 */
        switch (motor_id) {
            case MOTOR_ID_A:
                HAL_GPIO_WritePin(MOTOR_A_IN1_GPIO_Port, MOTOR_A_IN1_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(MOTOR_A_IN2_GPIO_Port, MOTOR_A_IN2_Pin, GPIO_PIN_RESET);
                break;
            case MOTOR_ID_B:
                HAL_GPIO_WritePin(MOTOR_B_IN1_GPIO_Port, MOTOR_B_IN1_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(MOTOR_B_IN2_GPIO_Port, MOTOR_B_IN2_Pin, GPIO_PIN_RESET);
                break;
            case MOTOR_ID_C:
                HAL_GPIO_WritePin(MOTOR_C_IN1_GPIO_Port, MOTOR_C_IN1_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(MOTOR_C_IN2_GPIO_Port, MOTOR_C_IN2_Pin, GPIO_PIN_RESET);
                break;
            case MOTOR_ID_D:
                HAL_GPIO_WritePin(MOTOR_D_IN1_GPIO_Port, MOTOR_D_IN1_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(MOTOR_D_IN2_GPIO_Port, MOTOR_D_IN2_Pin, GPIO_PIN_RESET);
                break;
            default:
                return;
        }
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1 + motor_id, 0);
    } else if (speed > 0) {
        /* 正转 */
        uint16_t pwm = (speed > MOTOR_PWM_MAX) ? MOTOR_PWM_MAX : speed;
        switch (motor_id) {
            case MOTOR_ID_A:
                HAL_GPIO_WritePin(MOTOR_A_IN1_GPIO_Port, MOTOR_A_IN1_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(MOTOR_A_IN2_GPIO_Port, MOTOR_A_IN2_Pin, GPIO_PIN_RESET);
                __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, pwm);
                break;
            case MOTOR_ID_B:
                HAL_GPIO_WritePin(MOTOR_B_IN1_GPIO_Port, MOTOR_B_IN1_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(MOTOR_B_IN2_GPIO_Port, MOTOR_B_IN2_Pin, GPIO_PIN_RESET);
                __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, pwm);
                break;
            case MOTOR_ID_C:
                HAL_GPIO_WritePin(MOTOR_C_IN1_GPIO_Port, MOTOR_C_IN1_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(MOTOR_C_IN2_GPIO_Port, MOTOR_C_IN2_Pin, GPIO_PIN_RESET);
                __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, pwm);
                break;
            case MOTOR_ID_D:
                HAL_GPIO_WritePin(MOTOR_D_IN1_GPIO_Port, MOTOR_D_IN1_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(MOTOR_D_IN2_GPIO_Port, MOTOR_D_IN2_Pin, GPIO_PIN_RESET);
                __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, pwm);
                break;
            default:
                return;
        }
    } else {
        /* 反转 */
        uint16_t pwm = ((-speed) > MOTOR_PWM_MAX) ? MOTOR_PWM_MAX : (-speed);
        switch (motor_id) {
            case MOTOR_ID_A:
                HAL_GPIO_WritePin(MOTOR_A_IN1_GPIO_Port, MOTOR_A_IN1_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(MOTOR_A_IN2_GPIO_Port, MOTOR_A_IN2_Pin, GPIO_PIN_SET);
                __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, pwm);
                break;
            case MOTOR_ID_B:
                HAL_GPIO_WritePin(MOTOR_B_IN1_GPIO_Port, MOTOR_B_IN1_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(MOTOR_B_IN2_GPIO_Port, MOTOR_B_IN2_Pin, GPIO_PIN_SET);
                __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, pwm);
                break;
            case MOTOR_ID_C:
                HAL_GPIO_WritePin(MOTOR_C_IN1_GPIO_Port, MOTOR_C_IN1_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(MOTOR_C_IN2_GPIO_Port, MOTOR_C_IN2_Pin, GPIO_PIN_SET);
                __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, pwm);
                break;
            case MOTOR_ID_D:
                HAL_GPIO_WritePin(MOTOR_D_IN1_GPIO_Port, MOTOR_D_IN1_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(MOTOR_D_IN2_GPIO_Port, MOTOR_D_IN2_Pin, GPIO_PIN_SET);
                __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, pwm);
                break;
            default:
                return;
        }
    }
}
