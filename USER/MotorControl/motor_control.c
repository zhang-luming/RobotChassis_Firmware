#include "motor_control.h"
#include "user_config.h"
#include "tim.h"
#include "gpio.h"
#include "stdio.h"
#include "stdlib.h"

/* ==================== 私有变量 ==================== */

/* 编码器累加绝对值 */
static int16_t encoder[MOTOR_COUNT] = {0};

/* 编码器相对变化值 */
static int16_t encoder_delta[MOTOR_COUNT] = {0};

/* 编码器目标速度值 */
static int16_t encoder_delta_target[MOTOR_COUNT] = {0};

/* PID状态结构体 */
typedef struct {
    int32_t bias;         /* 当前误差 */
    int32_t bias_last;    /* 上次误差 */
    int32_t bias_integral;/* 误差累加 */
    int16_t pwm_out;      /* PWM输出值 */
} MotorPID_t;

/* 4个电机的PID状态 */
static MotorPID_t motor_pid[MOTOR_COUNT] = {0};

/* ==================== 函数实现 ==================== */

/**
 * @brief 初始化电机控制模块
 */
void Motor_Init(void)
{
    uint8_t i;

    /* 初始化编码器捕获初始值 */
    Motor_ResetEncoder();

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
    for(i = 0; i < MOTOR_COUNT; i++)
    {
        motor_pid[i].bias = 0;
        motor_pid[i].bias_last = 0;
        motor_pid[i].bias_integral = 0;
        motor_pid[i].pwm_out = 0;
    }
}

/**
 * @brief 设置电机目标速度
 */
void Motor_SetTargetSpeed(uint8_t motor_id, int16_t speed)
{
    if(motor_id < MOTOR_COUNT)
    {
        encoder_delta_target[motor_id] = speed;
    }
}

/**
 * @brief 电机PID控制（统一算法）
 */
int16_t Motor_PIDControl(uint8_t motor_id, int16_t ideal_speed, int16_t actual_speed)
{
    if(motor_id >= MOTOR_COUNT)
    {
        return 0;
    }

    MotorPID_t *pid = &motor_pid[motor_id];

    if(ideal_speed == 0)
    {
        pid->pwm_out = 0;
        pid->bias = 0;
        pid->bias_last = 0;
        pid->bias_integral = 0;
    }
    else
    {
        pid->bias = ideal_speed - actual_speed;
        pid->bias_integral += pid->bias;

        /* 积分限幅 */
        if(pid->bias_integral > 1000) pid->bias_integral = 1000;
        if(pid->bias_integral < -1000) pid->bias_integral = -1000;

        /* PID计算 */
        pid->pwm_out += (int16_t)(PID_P * pid->bias * PID_SCALE
                     + PID_I * pid->bias_integral * PID_SCALE
                     + PID_D * (pid->bias - pid->bias_last) * PID_SCALE);

        pid->bias_last = pid->bias;
    }

    return pid->pwm_out;
}

/**
 * @brief 设置电机PWM输出
 */
void Motor_SetSpeed(uint8_t motor_id, int16_t speed)
{
    if(speed == 0)
    {
        /* 停止电机 */
        switch(motor_id)
        {
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
    }
    else if(speed > 0)
    {
        /* 正转 */
        switch(motor_id)
        {
            case MOTOR_ID_A:
                HAL_GPIO_WritePin(MOTOR_A_IN1_GPIO_Port, MOTOR_A_IN1_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(MOTOR_A_IN2_GPIO_Port, MOTOR_A_IN2_Pin, GPIO_PIN_RESET);
                __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, (speed > MOTOR_PWM_MAX) ? MOTOR_PWM_MAX : speed);
                break;
            case MOTOR_ID_B:
                HAL_GPIO_WritePin(MOTOR_B_IN1_GPIO_Port, MOTOR_B_IN1_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(MOTOR_B_IN2_GPIO_Port, MOTOR_B_IN2_Pin, GPIO_PIN_RESET);
                __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, (speed > MOTOR_PWM_MAX) ? MOTOR_PWM_MAX : speed);
                break;
            case MOTOR_ID_C:
                HAL_GPIO_WritePin(MOTOR_C_IN1_GPIO_Port, MOTOR_C_IN1_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(MOTOR_C_IN2_GPIO_Port, MOTOR_C_IN2_Pin, GPIO_PIN_RESET);
                __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, (speed > MOTOR_PWM_MAX) ? MOTOR_PWM_MAX : speed);
                break;
            case MOTOR_ID_D:
                HAL_GPIO_WritePin(MOTOR_D_IN1_GPIO_Port, MOTOR_D_IN1_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(MOTOR_D_IN2_GPIO_Port, MOTOR_D_IN2_Pin, GPIO_PIN_RESET);
                __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, (speed > MOTOR_PWM_MAX) ? MOTOR_PWM_MAX : speed);
                break;
            default:
                return;
        }
    }
    else
    {
        /* 反转 */
        speed = abs(speed);
        switch(motor_id)
        {
            case MOTOR_ID_A:
                HAL_GPIO_WritePin(MOTOR_A_IN1_GPIO_Port, MOTOR_A_IN1_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(MOTOR_A_IN2_GPIO_Port, MOTOR_A_IN2_Pin, GPIO_PIN_SET);
                __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, (speed > MOTOR_PWM_MAX) ? MOTOR_PWM_MAX : speed);
                break;
            case MOTOR_ID_B:
                HAL_GPIO_WritePin(MOTOR_B_IN1_GPIO_Port, MOTOR_B_IN1_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(MOTOR_B_IN2_GPIO_Port, MOTOR_B_IN2_Pin, GPIO_PIN_SET);
                __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, (speed > MOTOR_PWM_MAX) ? MOTOR_PWM_MAX : speed);
                break;
            case MOTOR_ID_C:
                HAL_GPIO_WritePin(MOTOR_C_IN1_GPIO_Port, MOTOR_C_IN1_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(MOTOR_C_IN2_GPIO_Port, MOTOR_C_IN2_Pin, GPIO_PIN_SET);
                __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, (speed > MOTOR_PWM_MAX) ? MOTOR_PWM_MAX : speed);
                break;
            case MOTOR_ID_D:
                HAL_GPIO_WritePin(MOTOR_D_IN1_GPIO_Port, MOTOR_D_IN1_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(MOTOR_D_IN2_GPIO_Port, MOTOR_D_IN2_Pin, GPIO_PIN_SET);
                __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, (speed > MOTOR_PWM_MAX) ? MOTOR_PWM_MAX : speed);
                break;
            default:
                return;
        }
    }
}

/**
 * @brief 读取编码器相对变化值并更新累加值
 */
void Motor_UpdateEncoderDelta(void)
{
    /* 读取编码器相对变化值 */
    encoder_delta[0] = (__HAL_TIM_GetCounter(&htim2) - ENCODER_MIDDLE);
    encoder_delta[1] = -(__HAL_TIM_GetCounter(&htim3) - ENCODER_MIDDLE);
    encoder_delta[2] = (__HAL_TIM_GetCounter(&htim4) - ENCODER_MIDDLE);
    encoder_delta[3] = -(__HAL_TIM_GetCounter(&htim5) - ENCODER_MIDDLE);

    /* 重置编码器捕获初始值 */
    Motor_ResetEncoder();

    /* 计算编码器累加值 */
    encoder[0] = encoder[0] + encoder_delta[0];
    encoder[1] = encoder[1] + encoder_delta[1];
    encoder[2] = encoder[2] + encoder_delta[2];
    encoder[3] = encoder[3] + encoder_delta[3];
}

/**
 * @brief 获取编码器累加值
 */
void Motor_GetEncoder(int16_t *encoder_values)
{
    if(encoder_values != NULL)
    {
        encoder_values[0] = encoder[0];
        encoder_values[1] = encoder[1];
        encoder_values[2] = encoder[2];
        encoder_values[3] = encoder[3];
    }
}

/**
 * @brief 重置编码器计数器到中间值
 */
void Motor_ResetEncoder(void)
{
    TIM2->CNT = ENCODER_MIDDLE;
    TIM3->CNT = ENCODER_MIDDLE;
    TIM4->CNT = ENCODER_MIDDLE;
    TIM5->CNT = ENCODER_MIDDLE;
}

/**
 * @brief 获取编码器目标速度数组
 */
int16_t* Motor_GetTargetSpeedArray(void)
{
    return encoder_delta_target;
}

/**
 * @brief 获取编码器相对变化值数组
 */
int16_t* Motor_GetEncoderDeltaArray(void)
{
    return encoder_delta;
}
