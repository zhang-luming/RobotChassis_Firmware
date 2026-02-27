/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : RobotChassis 主程序
  ******************************************************************************
  * @description
  * 基于 STM32F103xE 的四轮差速驱动机器人底盘主程序
  *
  * 硬件配置：
  * - MCU: STM32F103xE (72MHz, 256KB Flash, 48KB RAM)
  * - 电机: 4x 直流减速电机 + 编码器
  * - IMU: MPU6050 (I2C, DMP姿态融合)
  * - 通信: UART2 (115200)
  * - 调试: USART1
  *
  * 定时器分配：
  * - TIM1: 舵机PWM
  * - TIM2/3/4/5: 编码器输入捕获
  * - TIM6: 系统滴答 (10ms, 100Hz)
  * - TIM7: 微秒时间戳
  * - TIM8: 电机PWM (25kHz)
  *
  * 中断分配：
  * - IMU中断 (100Hz): IMU数据 + 编码器采样 + 电机PID控制
  * - TIM6中断 (100Hz): 主循环触发器
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes -----------------------------------------------------------*/
#include "System/timer.h"
#include "System/retarget.h"
#include "System/debug.h"

#include "led_control.h"
#include "motor_control.h"
#include "comm_protocol.h"
#include "imu.h"
#include "power_management.h"
#include "servo_control.h"
#include "ptp_sync.h"

/* Private define ------------------------------------------------------------*/
/* 主循环计数器 - 每10ms递增 */
uint16_t Run_Times = 0;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/**
  * @brief  主程序入口
  * @retval int
  */
int main(void)
{
    /* ========== HAL库初始化 ========== */
    HAL_Init();
    SystemClock_Config();

    /* ========== 外设初始化 ========== */
    MX_GPIO_Init();
    MX_TIM1_Init();      /* 舵机PWM */
    MX_ADC2_Init();      /* 电源电压ADC */
    MX_I2C1_Init();      /* MPU6050 I2C */
    MX_TIM2_Init();      /* 编码器A */
    MX_TIM3_Init();      /* 编码器B */
    MX_TIM4_Init();      /* 编码器C */
    MX_TIM5_Init();      /* 编码器D */
    MX_TIM8_Init();      /* 电机PWM */
    MX_USART1_UART_Init(); /* 调试串口 */
    MX_USART2_UART_Init(); /* 通信串口 */
    MX_TIM6_Init();      /* 系统滴答 */
    MX_TIM7_Init();      /* 微秒时间戳 */

    /* ========== 系统模块初始化 ========== */

    /* 1. 时间戳模块 (必须最先初始化) */
    HAL_TIM_Base_Start_IT(&htim6);  /* 启动10ms系统滴答 */
    Time_Init();                     /* 初始化微秒时间戳 */

    /* 2. 调试输出 */
    RetargetInit(&huart1);
    DEBUG_INFO("========================================\r\n");
    DEBUG_INFO("RobotChassis Firmware\r\n");
    DEBUG_INFO("初始化开始...\r\n");

    /* 3. LED状态指示 */
    LED_Init();
    DEBUG_INFO("[OK] LED模块\r\n");

    /* 4. 电机控制模块 */
    Motor_Init();
    DEBUG_INFO("[OK] 电机控制模块\r\n");

    /* 5. 电源管理模块 */
    Power_Init();
    DEBUG_INFO("[OK] 电源管理模块\r\n");

    /* 6. 舵机控制模块 */
    Servo_Init();
    DEBUG_INFO("[OK] 舵机控制模块\r\n");

    /* 7. PTP时间同步模块 */
    PTP_Init();
    DEBUG_INFO("[OK] PTP时间同步模块\r\n");

    /* 8. 通信模块 */
    Comm_Init();
    DEBUG_INFO("[OK] 通信模块\r\n");

    /* 9. IMU模块 (最后初始化，可能耗时较长) */
    const uint8_t imu_result = IMU_Init();
    if (imu_result != 0) {
        DEBUG_ERROR("[FAIL] MPU6050初始化失败 (错误码:%d)\r\n", imu_result);
        LED_Error();  /* LED快闪表示错误 */
    } else {
        DEBUG_INFO("[OK] MPU6050初始化成功\r\n");
        LED_Heartbeat();  /* LED心跳表示正常 */

        /* 启用IMU中断 (100Hz数据更新) */
        IMU_SetEnabled(1);
        DEBUG_INFO("[OK] IMU中断已启用\r\n");
    }

    DEBUG_INFO("========================================\r\n");
    DEBUG_INFO("系统运行中...\r\n");

    /* ========== 主循环 ========== */
    while (1)
    {
        /* 检查10ms系统滴答 */
        if (Timer_IsTim6Timeout())
        {
            Run_Times++;

            /* 100Hz 任务 ========== */

            /* 通信协议处理 (接收并分发命令) */
            Comm_Update();

            /* LED状态更新 */
            LED_Update();

            /* 舵机控制更新 */
            Servo_Update();

            /* 5Hz 任务 (每200ms) ========== */
            if (Run_Times % 20 == 0)
            {
                /* 电源电压更新并发送 */
                Power_Update();
                Power_Send();
            }
        }
    }
}

/**
  * @brief 系统时钟配置
  * @note  SYSCLK = 72MHz (HSE 8MHz × PLL9)
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    /* 配置PLL */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    /* 配置系统时钟 */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;       /* HCLK  = 72MHz */
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;        /* APB1  = 36MHz */
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;        /* APB2  = 72MHz */
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

    /* 配置ADC时钟 */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;     /* ADC = 12MHz */
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
}

/**
  * @brief 错误处理函数 (HAL库调用)
  */
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
        /* 错误状态，LED常亮 */
    }
}

#ifdef USE_FULL_ASSERT
/**
  * @brief 断言失败处理函数
  */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* 断言失败时可以在此处添加处理 */
}
#endif
