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
#include "imu.h"
#include "System/timer.h"
#include "System/retarget.h"
#include "System/debug.h"
#include "led_control.h"

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
    MX_TIM7_Init();  /* 微秒时间戳定时器 */
    /* USER CODE BEGIN 2 */

    /* 启动系统定时器 */
    HAL_TIM_Base_Init(&htim6);
    HAL_TIM_Base_Start_IT(&htim6);

    /* 初始化微秒时间戳模块 */
    Time_Init();

    /* 初始化stdio重定向（printf支持） */
    RetargetInit(&huart1);  /* 传入USART1句柄，用于调试日志输出 */

    DEBUG_INFO("系统初始化开始...\r\n");

    /* 初始化LED控制模块 */
    LED_Init();

    /* 初始化各功能模块 */
    Motor_Init();         /* 电机控制模块 */
    Power_Init();         /* 电源管理模块 */
    Servo_Init();         /* 舵机控制模块 */
    Comm_Init();          /* 通信模块 */

    /* MPU6050 IMU初始化 */
    const uint8_t imu_result = IMU_Init();
    if (imu_result != 0) {
        DEBUG_ERROR("MPU6050初始化失败 (错误码:%d)\r\n", imu_result);
        switch (imu_result) {
            case 1: DEBUG_ERROR("  -> 传感器设置失败\r\n"); break;
            case 2: DEBUG_ERROR("  -> FIFO配置失败\r\n"); break;
            case 3: DEBUG_ERROR("  -> 采样率设置失败\r\n"); break;
            case 4: DEBUG_ERROR("  -> DMP固件加载失败\r\n"); break;
            case 5: DEBUG_ERROR("  -> 方向矩阵设置失败\r\n"); break;
            case 6: DEBUG_ERROR("  -> DMP功能使能失败\r\n"); break;
            case 7: DEBUG_ERROR("  -> FIFO速率设置失败\r\n"); break;
            case 8: DEBUG_ERROR("  -> 传感器自检失败 (请确保底盘静止且水平放置)\r\n"); break;
            case 9: DEBUG_ERROR("  -> DMP使能失败\r\n"); break;
            case 10: DEBUG_ERROR("  -> MPU初始化失败\r\n"); break;
            default: DEBUG_ERROR("  -> 未知错误\r\n"); break;
        }
        LED_Error();  /* LED快闪表示错误 */
    } else {
        DEBUG_INFO("MPU6050初始化成功\r\n");

        LED_Heartbeat();  /* LED心跳表示正常运行 */
    }

    DEBUG_INFO("ROS底板开始运行\r\n");

    /* 临时禁用IMU中断，避免干扰串口通信调试 */
    DEBUG_INFO("[DEBUG] IMU中断已禁用（用于串口通信调试）\r\n");
    IMU_SetEnabled(0);

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        if (Timer_IsTim6Timeout()) /* 判断10ms时间是否到了 */
        {
            Run_Times++;

            /* ========== 系统更新 ========== */
            Comm_Update();          /* 处理接收数据并分发指令 */

            /* 每50ms执行 */
            if (Run_Times % 5 == 0) {
                Motor_UpdateControl();  // 电机控制
                Servo_Update(); // 舵机控制（预定义接口）
            }

            /* 每100ms执行 - LED状态更新 */
            if (Run_Times % 10 == 0) {
                LED_Update();
            }

            /* 每200ms执行 */
            if (Run_Times % 20 == 0) {
                Power_Update();     /* 更新电池电压 */
                Power_Send();
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
