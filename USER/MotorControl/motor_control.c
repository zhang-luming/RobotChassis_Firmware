/**
 ******************************************************************************
 * @file    motor_control.c
 * @brief   电机控制模块 - 四路电机PID控制（分离式架构）
 *
 * 功能说明：
 * - 四路编码器速度读取
 * - PID闭环控制
 * - PWM输出控制
 * - 编码器数据上报
 * - 动态时间计算（自适应调用周期）
 *
 * 分离式架构：
 * - Motor_ReadAndReport(): 读取编码器位置并上报（100Hz，IMU中断）
 * - Motor_UpdateControl(): 执行PID控制并输出PWM（25Hz，TIM6中断）
 *
 * 模块接口：
 * - Motor_Init(): 初始化模块
 * - Motor_ReadAndReport(): 读取并上报编码器位置（高优先级）
 * - Motor_UpdateControl(): 执行电机控制（低优先级）
 * - Motor_ProcessSetSpeed(): 设置目标速度
 * - Motor_ProcessSetPID(): 设置PID参数
 ******************************************************************************
 */

#include "motor_control.h"

#include "comm_protocol.h"
#include "tim.h"
#include "timer.h"
#include "user_config.h"

/* ==================== 私有变量 ==================== */

/* 目标速度（单位：CPS - Counts Per Second，编码器计数/秒） */
static int16_t g_target_speeds[MOTOR_COUNT] = {0};

/* ========== 控制模块专用变量 ========== */
static int16_t g_control_last[MOTOR_COUNT] = {ENCODER_MIDDLE, ENCODER_MIDDLE,
                                                ENCODER_MIDDLE, ENCODER_MIDDLE};
static int16_t g_control_deltas[MOTOR_COUNT] = {0};

/* ========== 上报模块专用变量 ========== */
static int16_t g_report_last[MOTOR_COUNT] = {ENCODER_MIDDLE, ENCODER_MIDDLE,
                                               ENCODER_MIDDLE, ENCODER_MIDDLE};
static int16_t g_report_accum[MOTOR_COUNT] = {0};


/* 上次调用时间戳（微秒） */
static uint64_t g_last_update_time_us = 0;

/* 实际调用周期（毫秒）- 动态计算 */
static float g_update_period_ms = 10.0f;

/* PID状态结构体 */
typedef struct {
  int16_t kp;            /* 比例系数（从外部变量获取） */
  int16_t ki;            /* 积分系数（从外部变量获取） */
  int16_t kd;            /* 微分系数（从外部变量获取） */
  int32_t bias;          /* 当前误差 */
  int32_t bias_last;     /* 上次误差 */
  int32_t bias_integral; /* 误差累加 */
  int16_t pwm_out;       /* PWM输出值 */
} MotorPID_t;

/* 4个电机的PID状态 */
static MotorPID_t g_motor_pid[MOTOR_COUNT] = {0};

/* PID默认参数（可通过串口动态调整） */
static int16_t g_pid_p = 150;  /* 比例系数 */
static int16_t g_pid_i = 10;   /* 积分系数 */
static int16_t g_pid_d = 5;    /* 微分系数 */

/* ==================== 私有函数前向声明 ==================== */
static int16_t Motor_PIDControl(uint8_t motor_id, int16_t target_speed,
                                int16_t actual_speed);
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
    g_motor_pid[i].kp = g_pid_p;
    g_motor_pid[i].ki = g_pid_i;
    g_motor_pid[i].kd = g_pid_d;
    g_motor_pid[i].bias = 0;
    g_motor_pid[i].bias_last = 0;
    g_motor_pid[i].bias_integral = 0;
    g_motor_pid[i].pwm_out = 0;
  }

  /* 初始化时间戳 */
  g_last_update_time_us = Time_GetUs();
}

/**
 * @brief 读取编码器位置并上报（在IMU中断中调用）
 *
 * 功能：
 * - 独立读取编码器增量并累加
 * - 上报累加位置给上位机
 *
 * 优势：
 * - 100Hz实时更新
 * - 与Motor_UpdateControl()完全解耦
 * - 不受控制模块重置CNT影响
 *
 * 注意：
 * - 此函数在IMU中断（100Hz）中调用
 * - 维护独立的last和accum变量
 */
void Motor_ReadAndReport(void) {
  int16_t current_counter;
  int16_t delta;

  /* 电机A - TIM2 */
  current_counter = __HAL_TIM_GET_COUNTER(&htim2);
  delta = current_counter - g_report_last[0];
  if (delta < -32767) delta += 65536;
  else if (delta > 32767) delta -= 65536;
  g_report_accum[0] += delta;
  g_report_last[0] = current_counter;

  /* 电机B - TIM3（反向） */
  current_counter = __HAL_TIM_GET_COUNTER(&htim3);
  delta = current_counter - g_report_last[1];
  if (delta < -32767) delta += 65536;
  else if (delta > 32767) delta -= 65536;
  g_report_accum[1] -= delta;
  g_report_last[1] = current_counter;

  /* 电机C - TIM4 */
  current_counter = __HAL_TIM_GET_COUNTER(&htim4);
  delta = current_counter - g_report_last[2];
  if (delta < -32767) delta += 65536;
  else if (delta > 32767) delta -= 65536;
  g_report_accum[2] += delta;
  g_report_last[2] = current_counter;

  /* 电机D - TIM5（反向） */
  current_counter = __HAL_TIM_GET_COUNTER(&htim5);
  delta = current_counter - g_report_last[3];
  if (delta < -32767) delta += 65536;
  else if (delta > 32767) delta -= 65536;
  g_report_accum[3] -= delta;
  g_report_last[3] = current_counter;

  /* 发送编码器数据 */
  Comm_SendDataFrame(FUNC_ENCODER, g_report_accum, MOTOR_COUNT);
}

/**
 * @brief 执行电机控制（在TIM6中断中调用）
 *
 * 功能：
 * 1. 计算时间差（自适应调用周期）
 * 2. 读取编码器增量（用于PID控制）
 * 3. 重置编码器到中间值
 * 4. 执行PID控制并输出PWM
 *
 * 注意：
 * - 与Motor_ReadAndReport()完全解耦
 * - 维护独立的control_last和control_deltas变量
 * - 不再维护encoder_accum（由上报模块负责）
 */
void Motor_UpdateControl(void) {
  int16_t current_counter;
  uint64_t current_time_us = Time_GetUs();
  uint64_t elapsed_us = current_time_us - g_last_update_time_us;

  g_last_update_time_us = current_time_us;
  g_update_period_ms = elapsed_us / 1000.0f;

  /* 1. 读取编码器增量（用于PID控制） */
  /* 电机A - TIM2 */
  current_counter = __HAL_TIM_GET_COUNTER(&htim2);
  int16_t delta0 = current_counter - g_control_last[0];
  if (delta0 < -32767) {
    delta0 += 65536;
  } else if (delta0 > 32767) {
    delta0 -= 65536;
  }
  g_control_deltas[0] = delta0;

  /* 电机B - TIM3 */
  current_counter = __HAL_TIM_GET_COUNTER(&htim3);
  int16_t delta1 = current_counter - g_control_last[1];
  if (delta1 < -32767) {
    delta1 += 65536;
  } else if (delta1 > 32767) {
    delta1 -= 65536;
  }
  g_control_deltas[1] = -delta1;

  /* 电机C - TIM4 */
  current_counter = __HAL_TIM_GET_COUNTER(&htim4);
  int16_t delta2 = current_counter - g_control_last[2];
  if (delta2 < -32767) {
    delta2 += 65536;
  } else if (delta2 > 32767) {
    delta2 -= 65536;
  }
  g_control_deltas[2] = delta2;

  /* 电机D - TIM5 */
  current_counter = __HAL_TIM_GET_COUNTER(&htim5);
  int16_t delta3 = current_counter - g_control_last[3];
  if (delta3 < -32767) {
    delta3 += 65536;
  } else if (delta3 > 32767) {
    delta3 -= 65536;
  }
  g_control_deltas[3] = -delta3;

  /* 2. 重置编码器到中间值 */
  TIM2->CNT = ENCODER_MIDDLE;
  TIM3->CNT = ENCODER_MIDDLE;
  TIM4->CNT = ENCODER_MIDDLE;
  TIM5->CNT = ENCODER_MIDDLE;
  g_control_last[0] = ENCODER_MIDDLE;
  g_control_last[1] = ENCODER_MIDDLE;
  g_control_last[2] = ENCODER_MIDDLE;
  g_control_last[3] = ENCODER_MIDDLE;

  /* 3. PID控制并输出PWM */
  for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
    int16_t pwm = Motor_PIDControl(i, g_target_speeds[i], g_control_deltas[i]);
    Motor_SetSpeed(i, pwm);
  }
}

/**
 * @brief 处理电机速度控制帧（从通信协议调用）
 * @param frame 帧数据指针
 * @param frame_len 帧长度
 *
 * 帧格式：[FC][0x06][速度A高][速度A低][速度B高][速度B低][速度C高][速度C低][速度D高][速度D低][Checksum][DF]
 * 索引：  0    1     2      3       4      5       6      7       8      9      10       11
 *
 * 功能：
 * - 从帧中解析4个电机的速度（大端序int16_t）
 * - 单位：centi-CPS（CPS/100）
 * - 转换为编码器增量并存储
 */
void Motor_ProcessSpeedFrame(uint8_t *frame, uint16_t frame_len) {
  /* 帧数据从索引2开始，格式：4个int16_t（大端序） */
  uint8_t *data = &frame[2];

  /* 解析并设置4个电机的速度 */
  for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
    /* 大端序解析int16_t */
    int16_t speed = (int16_t)((data[i * 2] << 8) | data[i * 2 + 1]);

    /* centi-CPS转换为编码器增量（直接转换，无需额外函数调用） */
    if (i < MOTOR_COUNT) {
      /* speed单位是CPS/100，转换为编码器增量，扩大一百倍防止目标速度太大无法使用两字节表示 */
      g_target_speeds[i] = (int16_t)(speed * g_update_period_ms / 1000.f * 100.0f);
    }
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

    /* 同时更新默认值（当设置电机0时） */
    if (motor_id == 0) {
      g_pid_p = kp;
      g_pid_i = ki;
      g_pid_d = kd;
    }
  }
}

/**
 * @brief 处理PID参数设置帧（从通信协议调用）
 * @param frame 帧数据指针
 * @param frame_len 帧长度
 *
 * 帧格式：[FC][0x07][Kp高][Kp低][Ki高][Ki低][Kd高][Kd低][Checksum][DF]
 * 索引：  0    1     2    3     4    5     6    7      8        9
 *
 * 功能：
 * - 从帧中解析PID参数（3个int16_t，大端序）
 * - 为所有4个电机设置相同的PID参数
 */
void Motor_ProcessPIDFrame(uint8_t *frame, uint16_t frame_len) {
  /* 帧数据从索引2开始，格式：3个int16_t（大端序） */
  uint8_t *data = &frame[2];

  /* 解析PID参数 */
  int16_t kp = (int16_t)((data[0] << 8) | data[1]);
  int16_t ki = (int16_t)((data[2] << 8) | data[3]);
  int16_t kd = (int16_t)((data[4] << 8) | data[5]);

  /* 为所有4个电机设置相同的PID参数 */
  for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
    Motor_ProcessSetPID(i, kp, ki, kd);
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
static int16_t Motor_PIDControl(uint8_t motor_id, int16_t target_speed,
                                int16_t actual_speed) {
  if (motor_id >= MOTOR_COUNT) {
    return 0;
  }

  MotorPID_t* pid = &g_motor_pid[motor_id];

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
    pid->pwm_out +=
        (int16_t)(pid->kp * pid->bias * PID_SCALE +
                  pid->ki * pid->bias_integral * PID_SCALE +
                  pid->kd * (pid->bias - pid->bias_last) * PID_SCALE);

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
        HAL_GPIO_WritePin(MOTOR_A_IN1_GPIO_Port, MOTOR_A_IN1_Pin,
                          GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_A_IN2_GPIO_Port, MOTOR_A_IN2_Pin,
                          GPIO_PIN_RESET);
        break;
      case MOTOR_ID_B:
        HAL_GPIO_WritePin(MOTOR_B_IN1_GPIO_Port, MOTOR_B_IN1_Pin,
                          GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_B_IN2_GPIO_Port, MOTOR_B_IN2_Pin,
                          GPIO_PIN_RESET);
        break;
      case MOTOR_ID_C:
        HAL_GPIO_WritePin(MOTOR_C_IN1_GPIO_Port, MOTOR_C_IN1_Pin,
                          GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_C_IN2_GPIO_Port, MOTOR_C_IN2_Pin,
                          GPIO_PIN_RESET);
        break;
      case MOTOR_ID_D:
        HAL_GPIO_WritePin(MOTOR_D_IN1_GPIO_Port, MOTOR_D_IN1_Pin,
                          GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_D_IN2_GPIO_Port, MOTOR_D_IN2_Pin,
                          GPIO_PIN_RESET);
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
        HAL_GPIO_WritePin(MOTOR_A_IN2_GPIO_Port, MOTOR_A_IN2_Pin,
                          GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, pwm);
        break;
      case MOTOR_ID_B:
        HAL_GPIO_WritePin(MOTOR_B_IN1_GPIO_Port, MOTOR_B_IN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_B_IN2_GPIO_Port, MOTOR_B_IN2_Pin,
                          GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, pwm);
        break;
      case MOTOR_ID_C:
        HAL_GPIO_WritePin(MOTOR_C_IN1_GPIO_Port, MOTOR_C_IN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_C_IN2_GPIO_Port, MOTOR_C_IN2_Pin,
                          GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, pwm);
        break;
      case MOTOR_ID_D:
        HAL_GPIO_WritePin(MOTOR_D_IN1_GPIO_Port, MOTOR_D_IN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_D_IN2_GPIO_Port, MOTOR_D_IN2_Pin,
                          GPIO_PIN_RESET);
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
        HAL_GPIO_WritePin(MOTOR_A_IN1_GPIO_Port, MOTOR_A_IN1_Pin,
                          GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_A_IN2_GPIO_Port, MOTOR_A_IN2_Pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, pwm);
        break;
      case MOTOR_ID_B:
        HAL_GPIO_WritePin(MOTOR_B_IN1_GPIO_Port, MOTOR_B_IN1_Pin,
                          GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_B_IN2_GPIO_Port, MOTOR_B_IN2_Pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, pwm);
        break;
      case MOTOR_ID_C:
        HAL_GPIO_WritePin(MOTOR_C_IN1_GPIO_Port, MOTOR_C_IN1_Pin,
                          GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_C_IN2_GPIO_Port, MOTOR_C_IN2_Pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, pwm);
        break;
      case MOTOR_ID_D:
        HAL_GPIO_WritePin(MOTOR_D_IN1_GPIO_Port, MOTOR_D_IN1_Pin,
                          GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_D_IN2_GPIO_Port, MOTOR_D_IN2_Pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, pwm);
        break;
      default:
        return;
    }
  }
}
