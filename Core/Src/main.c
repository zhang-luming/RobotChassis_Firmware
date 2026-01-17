/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 主程序主体
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * 此软件采用在 LICENSE 文件中找到的条款进行许可
  * 如果没有随附 LICENSE 文件，则按原样提供
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"

/* 模块头文件 */
#include "comm_protocol.h"
#include "motor_control.h"
#include "power_management.h"
#include "servo_control.h"
#include "imu_data.h"
#include "EXIT.h"
#include "System/retarget.h"
#include "led_control.h"

#define Debug
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* printf重定向已移至 retarget.c */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint16_t Run_Times = 0; /* 循环次数计数器 - 必须初始化为0 */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_TIM1_Init();
    MX_ADC2_Init();
    MX_I2C1_Init();
    MX_TIM2_Init();
    MX_TIM4_Init();
    MX_TIM5_Init();
    MX_TIM8_Init();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();
    MX_TIM3_Init();
    MX_TIM6_Init();
    /* USER CODE BEGIN 2 */

    /* 启动系统定时器 */
    HAL_TIM_Base_Init(&htim6);
    HAL_TIM_Base_Start_IT(&htim6);

    /* 初始化stdio重定向（printf支持） */
    RetargetInit(&huart1);  /* 传入USART1句柄，用于调试日志输出 */

    printf("系统初始化开始...\r\n");

    /* 初始化LED控制模块 */
    LED_Init();

    /* 初始化各功能模块 */
    Motor_Init();         /* 电机控制模块 */
    Power_Init();         /* 电源管理模块 */
    Servo_Init();         /* 舵机控制模块 */
    Comm_Init();          /* 通信模块 */

    /* MPU6050 IMU初始化 */
    if (IMU_Init() != 0) {
        printf("MPU6050初始化失败!!!\r\n");
        LED_SetState(LED_STATE_ERROR);  /* LED快闪表示错误 */
    } else {
        printf("MPU6050初始化成功\r\n");
        LED_SetState(LED_STATE_HEARTBEAT);  /* LED心跳表示正常运行 */
    }

    printf("ROS底板开始运行\r\n");

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        if (Judge_Time_OUT()) /* 判断10ms时间是否到了 */
        {
            Run_Times++;

            /* 更新LED状态 */
            LED_Update();

            /* 处理串口接收的控制数据 */
            Comm_ProcessControlData();

            /* 更新IMU数据 */
            IMU_Update();

            /* 发送IMU数据 */
            {
                int16_t data[3];

                /* 发送欧拉角 */
                IMU_GetEulerAngle(data);
                Comm_SendEulerAngle(data);

                /* 发送陀螺仪 */
                IMU_GetGyro(data);
                Comm_SendGyro(data);

                /* 发送加速度 */
                IMU_GetAccel(data);
                Comm_SendAccel(data);
            }

            /* 编码器速度计算 - 每40毫秒计算一次 */
            if (Run_Times % 4 == 0) {
                int16_t encoder_values[4];

                /* 更新编码器数据 */
                Motor_UpdateEncoderDelta();

                /* 获取编码器数据指针 */
                int16_t *encoder_delta = Motor_GetEncoderDeltaArray();
                int16_t *encoder_target = Motor_GetTargetSpeedArray();

                /* PWM输出计算 */
                int16_t motor_pwm_val = Motor_PIDControl(
                    MOTOR_ID_A, encoder_target[0], encoder_delta[0]);
                Motor_SetSpeed(MOTOR_ID_A, motor_pwm_val);

                motor_pwm_val = Motor_PIDControl(
                    MOTOR_ID_B, encoder_target[1], encoder_delta[1]);
                Motor_SetSpeed(MOTOR_ID_B, motor_pwm_val);

                motor_pwm_val = Motor_PIDControl(
                    MOTOR_ID_C, encoder_target[2], encoder_delta[2]);
                Motor_SetSpeed(MOTOR_ID_C, motor_pwm_val);

                motor_pwm_val = Motor_PIDControl(
                    MOTOR_ID_D, encoder_target[3], encoder_delta[3]);
                Motor_SetSpeed(MOTOR_ID_D, motor_pwm_val);

                printf("期望转速:%d,%d,%d,%d\r\n", encoder_delta[0], encoder_delta[1],
                       encoder_delta[2], encoder_delta[3]);

                /* 发送编码器数据 */
                Motor_GetEncoder(encoder_values);
                Comm_SendEncoder(encoder_values);
            }

            /* 电池电压检测 - 每200毫秒计算一次 */
            if (Run_Times % 20 == 0) {
                /* 更新电池电压采样 */
                Power_UpdateVoltage();

                /* 发送电池电压 */
                Comm_SendBatteryVoltage(Power_GetBatteryVoltage());
            }
        }
    }
    /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* 用户可以添加自己的实现来报告HAL错误返回状态 */
    __disable_irq();
    while (1) {}
    /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line) {
    /* USER CODE BEGIN 6 */
    /* 用户可以添加自己的实现来报告文件名和行号，
       例如: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
