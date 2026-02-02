/**
 ******************************************************************************
 * @file    debug.h
 * @brief   统一调试输出宏定义
 *
 * 功能说明：
 * - 提供统一的调试输出宏，控制系统中所有日志输出
 * - 支持不同级别的调试信息（INFO、WARN、ERROR）
 * - 通过编译开关控制是否启用调试输出
 *
 * 使用方法：
 * 1. 在需要调试的文件中包含此头文件
 * 2. 使用 DEBUG_PRINTF() 宏代替 printf()
 * 3. 在 user_config.h 中定义 DEBUG_ENABLE 宏启用调试输出
 *
 * 示例:
 *   @code
 *   #include "debug.h"
 *   DEBUG_PRINTF("系统初始化完成\r\n");
 *   DEBUG_PRINTF("温度值: %d\r\n", temperature);
 *   @endcode
 *
 * 注意事项：
 * - DEBUG_PRINTF 在 Release 模式下完全移除，不影响代码大小和性能
 * - 调试输出通过 USART1 发送（需先调用 RetargetInit()）
 ******************************************************************************
 */

#ifndef __DEBUG_H__
#define __DEBUG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include "user_config.h"  /* 必须包含此文件以获取 DEBUG_ENABLE 宏定义 */

/* ==================== 调试级别定义 ==================== */

/**
 * @brief 调试级别枚举
 */
typedef enum {
    DEBUG_LEVEL_INFO = 0,    /* 普通信息 */
    DEBUG_LEVEL_WARN = 1,    /* 警告信息 */
    DEBUG_LEVEL_ERROR = 2,   /* 错误信息 */
    DEBUG_LEVEL_VERBOSE = 3  /* 详细信息（用于频繁打印） */
} DebugLevel_t;

/* ==================== 调试宏配置 ==================== */

/* 检查是否启用了调试输出 */
#if defined(DEBUG_ENABLE)

    /* 支持分级调试输出 */
    #if defined(DEBUG_LEVEL)
        /* 当前调试级别 */
        #define _CURRENT_DEBUG_LEVEL DEBUG_LEVEL
    #else
        /* 默认输出所有级别 */
        #define _CURRENT_DEBUG_LEVEL DEBUG_LEVEL_VERBOSE
    #endif

    /* 判断是否应该输出该级别的调试信息 */
    #define _SHOULD_PRINT(level) (level <= _CURRENT_DEBUG_LEVEL)

    /* INFO 级别打印 */
    #define DEBUG_INFO(fmt, ...) \
        do { \
            if (_SHOULD_PRINT(DEBUG_LEVEL_INFO)) { \
                printf("[INFO] " fmt, ##__VA_ARGS__); \
            } \
        } while(0)

    /* WARN 级别打印 */
    #define DEBUG_WARN(fmt, ...) \
        do { \
            if (_SHOULD_PRINT(DEBUG_LEVEL_WARN)) { \
                printf("[WARN] " fmt, ##__VA_ARGS__); \
            } \
        } while(0)

    /* ERROR 级别打印 */
    #define DEBUG_ERROR(fmt, ...) \
        do { \
            if (_SHOULD_PRINT(DEBUG_LEVEL_ERROR)) { \
                printf("[ERROR] " fmt, ##__VA_ARGS__); \
            } \
        } while(0)

    /* VERBOSE 级别打印（用于频繁打印，如中断统计） */
    #define DEBUG_VERBOSE(fmt, ...) \
        do { \
            if (_SHOULD_PRINT(DEBUG_LEVEL_VERBOSE)) { \
                printf("[VERB] " fmt, ##__VA_ARGS__); \
            } \
        } while(0)

    /* 基础调试打印宏 */
    #define DEBUG_PRINTF(fmt, ...) \
        do { \
            if (_SHOULD_PRINT(DEBUG_LEVEL_INFO)) { \
                printf("[DEBUG] " fmt, ##__VA_ARGS__); \
            } \
        } while(0)

#else
    /* 调试未启用：所有宏定义为空，完全移除代码 */
    #define DEBUG_PRINTF(fmt, ...)            ((void)0)
    #define DEBUG_INFO(fmt, ...)             ((void)0)
    #define DEBUG_WARN(fmt, ...)             ((void)0)
    #define DEBUG_ERROR(fmt, ...)            ((void)0)
    #define DEBUG_VERBOSE(fmt, ...)          ((void)0)
    #define _CURRENT_DEBUG_LEVEL             0
    #define _SHOULD_PRINT(level)             0
#endif

/* ==================== 断言宏 ==================== */

#ifdef DEBUG_ENABLE
    /* 调试模式下的断言 */
    #define DEBUG_ASSERT(expr) \
        do { \
            if (!(expr)) { \
                DEBUG_ERROR("断言失败: %s 在 %s:%d\r\n", #expr, __FILE__, __LINE__); \
                while(1); \
            } \
        } while(0)
#else
    /* Release 模式下移除断言 */
    #define DEBUG_ASSERT(expr) ((void)0)
#endif

/* ==================== 调试辅助宏 ==================== */

/**
 * @brief 打印函数名和行号
 */
#define DEBUG_HERE() \
    DEBUG_INFO("到达位置: %s() Line:%d\r\n", __FUNCTION__, __LINE__)

/**
 * @brief 打印变量值
 */
#define DEBUG_VAR(var) \
    DEBUG_INFO(#var " = %d (0x%X)\r\n", (int)(var), (unsigned int)(var))

/**
 * @brief 打印十六进制数据
 */
#define DEBUG_HEX(data, len) \
    do { \
        if (_SHOULD_PRINT(DEBUG_LEVEL_VERBOSE)) { \
            uint8_t *_p = (uint8_t*)(data); \
            DEBUG_VERBOSE("[%s] HEX: ", __FUNCTION__); \
            for (int _i = 0; _i < (len); _i++) { \
                printf("%02X ", _p[_i]); \
            } \
            printf("\r\n"); \
        } \
    } while(0)

#ifdef __cplusplus
}
#endif

#endif /* __DEBUG_H__ */
