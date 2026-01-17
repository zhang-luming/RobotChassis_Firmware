/**
 ******************************************************************************
 * @file    retarget.h
 * @brief   C标准库I/O重定向头文件
 * @details 本文件提供了将C标准库输入输出函数（printf、scanf等）
 *          重定向到UART串口的接口声明，用于在嵌入式系统中实现
 *          标准输入输出功能，便于调试和日志输出
 *
 *          功能特性：
 *          - 支持printf()输出到UART串口
 *          - 支持scanf()从UART串口输入
 *          - 支持getchar()、putchar()等标准I/O函数
 *          - 自动禁用stdout缓冲，实时输出
 *
 *          使用方法：
 *          1. 在main()函数中调用RetargetInit(&huartX)初始化
 *          2. 之后可直接使用printf()等标准I/O函数
 *          3. 确保已包含stdio.h头文件
 *
 * @note    本实现不支持半主机模式（SEMIHOSTING）
 *          半主机模式会依赖调试器，影响独立运行
 *
 * @author  RobotChassis Team
 * @date    2026-01-17
 ******************************************************************************
 */

#ifndef ROBOTCHASSIS_FIRMWARE_RETARGET_H
#define ROBOTCHASSIS_FIRMWARE_RETARGET_H

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== 头文件包含 ==================== */
#include "stm32f1xx_hal.h"  // STM32 HAL库定义
#include <sys/stat.h>        // 文件状态结构体定义

/* ==================== 函数声明 ==================== */

/**
 * @brief 初始化重定向功能
 * @details 设置用于标准I/O的UART句柄，并禁用stdout缓冲
 *
 * @param huart UART句柄指针，指定使用哪个串口（如&huart1）
 *
 * @note  必须在使用printf()等函数之前调用此函数
 * @note  通常在main()函数初始化UART后立即调用
 *
 * 示例:
 *   @code
 *   MX_USART1_UART_Init();
 *   RetargetInit(&huart1);
 *   printf("System initialized\\n");
 *   @endcode
 */
void RetargetInit(UART_HandleTypeDef *huart);

/**
 * @brief 检查文件描述符是否关联到终端设备
 * @details 用于判断文件描述符是否为交互式终端
 *
 * @param fd 文件描述符（STDIN_FILENO、STDOUT_FILENO、STDERR_FILENO）
 * @return 1-是终端设备，0-不是终端设备
 *
 * @note  标准输入/输出/错误都返回1（表示是终端）
 * @note  其他文件描述符返回0并设置errno为EBADF
 */
int _isatty(int fd);

/**
 * @brief 写数据到文件描述符
 * @details 实现printf()、puts()等输出函数的底层写入功能
 *
 * @param fd  文件描述符（STDOUT_FILENO或STDERR_FILENO）
 * @param ptr 待写入数据的缓冲区指针
 * @param len 待写入数据的长度
 * @return 成功返回写入的字节数，失败返回-1
 *
 * @note  STDOUT_FILENO和STDERR_FILENO都通过UART发送
 * @note  使用HAL_UART_Transmit()阻塞发送，超时时间HAL_MAX_DELAY
 * @note  发送失败返回EIO错误码
 *
 * 工作流程:
 *   1. 检查fd是否为STDOUT_FILENO或STDERR_FILENO
 *   2. 通过UART发送数据
 *   3. 返回实际发送的字节数
 */
int _write(int fd, char *ptr, int len);

/**
 * @brief 关闭文件描述符
 * @details 关闭指定的文件描述符（本实现中为空操作）
 *
 * @param fd 文件描述符
 * @return 成功返回0，失败返回-1
 *
 * @note  标准输入/输出/错误流不支持关闭，返回0
 * @note  其他文件描述符返回错误
 */
int _close(int fd);

/**
 * @brief 移动文件读写位置
 * @details 改变文件的读写位置指针（本实现不支持）
 *
 * @param fd  文件描述符
 * @param ptr 偏移量
 * @param dir 起始位置（SEEK_SET、SEEK_CUR、SEEK_END）
 * @return 成功返回新的文件位置，失败返回-1
 *
 * @note  UART流不支持随机访问，总是返回错误
 * @note  设置errno为EBADF（错误的文件描述符）
 */
int _lseek(int fd, int ptr, int dir);

/**
 * @brief 从文件描述符读取数据
 * @details 实现scanf()、getchar()等输入函数的底层读取功能
 *
 * @param fd  文件描述符（STDIN_FILENO）
 * @param ptr 接收数据的缓冲区指针
 * @param len 期望读取的字节数
 * @return 成功返回读取的字节数，失败返回-1
 *
 * @note  只支持STDIN_FILENO（标准输入）
 * @note  每次只读取1个字符，忽略len参数
 * @note  使用HAL_UART_Receive()阻塞接收，超时时间HAL_MAX_DELAY
 *
 * 工作流程:
 *   1. 检查fd是否为STDIN_FILENO
 *   2. 从UART接收1个字节
 *   3. 返回读取的字节数（总是1）
 *
 * 注意事项:
 *   - 这是一个阻塞函数，会等待UART数据到来
 *   - 不支持多字符读取，len参数被忽略
 */
int _read(int fd, char *ptr, int len);

/**
 * @brief 获取文件状态信息
 * @details 查询文件的属性信息（如文件类型、大小等）
 *
 * @param fd 文件描述符
 * @param st 指向stat结构体的指针，用于返回文件状态
 * @return 成功返回0，失败返回-1
 *
 * @note  标准输入/输出/错误流被标记为字符设备（S_IFCHR）
 * @note  其他文件描述符返回错误
 */
int _fstat(int fd, struct stat *st);

#ifdef __cplusplus
}
#endif

#endif /* ROBOTCHASSIS_FIRMWARE_RETARGET_H */
