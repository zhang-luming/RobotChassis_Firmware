/**
 ******************************************************************************
 * @file    retarget.c
 * @brief   C标准库I/O重定向实现文件
 * @details 本文件实现了将C标准库输入输出函数重定向到UART串口的功能
 *          使得在嵌入式系统中可以使用printf()、scanf()等标准I/O函数
 *          进行调试输出和用户交互
 *
 *          支持的功能：
 *          - printf() → UART发送
 *          - scanf() → UART接收
 *          - putchar()、getchar()
 *          - puts()、gets()
 *
 *          工作原理：
 *          通过重写newlib C库的系统调用接口（以_开头的函数），
 *          将标准I/O操作重定向到UART HAL函数。这是嵌入式开发中
 *          实现标准I/O的常用方法。
 *
 *          性能特性：
 *          - printf()为阻塞发送，直到所有数据发送完成
 *          - scanf()为阻塞接收，直到接收到1个字符
 *          - stdout无缓冲，字符立即发送
 *
 * @note    本实现不支持半主机模式（SEMIHOSTING）
 *          半主机模式需要调试器支持，不适合独立运行
 *
 * @author  RobotChassis Team
 * @date    2026-01-17
 ******************************************************************************
 */

/* ==================== 头文件包含 ==================== */
#include <stdio.h>      // 标准输入输出定义（stdout、stdin、stderr）
#include <_ansi.h>      // newlib标准头文件
#include <_syslist.h>   // 系统调用列表定义
#include <errno.h>      // 错误码定义
#include <sys/time.h>   // 时间相关结构
#include <sys/times.h>  // 进程时间结构
#include "retarget.h"   // 本模块头文件
#include <stdint.h>     // 标准整数类型

/* ==================== 条件编译检查 ==================== */
/**
 * @note OS_USE_SEMIHOSTING未定义时，才编译本实现
 *       如果定义了OS_USE_SEMIHOSTING，则使用半主机模式（依赖调试器）
 */
#if !defined(OS_USE_SEMIHOSTING)

/* ==================== 宏定义 ==================== */
/** @defgroup 文件描述符定义
 *  标准文件描述符编号（POSIX标准）
 */
///@{
#define STDIN_FILENO   0  /**< 标准输入文件描述符（用于scanf、getchar等） */
#define STDOUT_FILENO  1  /**< 标准输出文件描述符（用于printf、puts等） */
#define STDERR_FILENO  2  /**< 标准错误文件描述符（用于perror等） */
///@}

/* ==================== 全局变量 ==================== */
/**
 * @var gHuart
 * @brief 全局UART句柄指针
 * @details 保存用于标准I/O的UART外设句柄
 *
 * @note  由RetargetInit()函数初始化
 * @note  所有I/O操作都通过此UART进行
 */
UART_HandleTypeDef *gHuart;

/* ==================== 函数实现 ==================== */

/**
 * @brief 初始化重定向功能
 * @details 设置用于标准I/O的UART句柄，并配置stdout为无缓冲模式
 *
 * @param huart UART句柄指针，指定使用哪个UART外设
 *
 * 实现细节：
 *   1. 保存UART句柄到全局变量gHuart
 *   2. 调用setvbuf()禁用stdout缓冲
 *   3. 设置为_IONBF（无缓冲）模式
 *
 * @note 禁用缓冲后，printf()的每个字符会立即发送到UART
 *       这会降低性能，但能确保实时性，适合调试输出
 *
 * @warning 必须在使用任何标准I/O函数之前调用
 *
 * 示例:
 *   @code
 *   // main.c
 *   int main(void) {
 *       HAL_Init();
 *       SystemClock_Config();
 *       MX_USART1_UART_Init();  // 初始化UART
 *       RetargetInit(&huart1);  // 初始化重定向
 *
 *       printf("System started!\n");  // 现在可以使用printf了
 *       while(1) {
 *           // 主循环
 *       }
 *   }
 *   @endcode
 */
void RetargetInit(UART_HandleTypeDef *huart)
{
    /* 保存UART句柄到全局变量 */
    gHuart = huart;

    /**
     * @note 禁用stdout的I/O缓冲
     *
     * setvbuf参数说明:
     * - stdout: 标准输出流
     * - NULL: 使用自动分配的缓冲区（此处实际不使用）
     * - _IONBF: 无缓冲模式（No Buffering）
     * - 0: 缓冲区大小（无缓冲模式下无意义）
     *
     * @note _IONBF模式：每次printf()后立即发送，不等待缓冲区满
     * @warning 如果需要提高性能，可以使用行缓冲(_IOLBF)或全缓冲(_IOFBF)
     */
    setvbuf(stdout, NULL, _IONBF, 0);
}

/**
 * @brief 检查文件描述符是否为终端设备
 * @details 判断指定的文件描述符是否关联到交互式终端
 *
 * @param fd 文件描述符（0=stdin, 1=stdout, 2=stderr）
 * @return 1-是终端设备，0-不是终端设备（并设置errno）
 *
 * 实现逻辑：
 *   1. 检查fd是否在0-2范围内（标准I/O范围）
 *   2. 如果是，返回1（表示是终端）
 *   3. 如果不是，设置errno为EBADF并返回0
 *
 * @note EBADF: Bad File Descriptor（错误的文件描述符）
 *
 * 典型应用场景：
 *   - isatty()函数调用此接口判断是否可以交互
 *   - 某些程序根据是否为终端选择不同的输出格式
 */
int _isatty(int fd)
{
    /* 检查文件描述符是否为标准I/O之一 */
    if (fd >= STDIN_FILENO && fd <= STDERR_FILENO)
        return 1;  // 是终端设备

    /* 不是有效的文件描述符 */
    errno = EBADF;
    return 0;
}

/**
 * @brief 写数据到文件描述符
 * @details 实现printf()、puts()、putchar()等输出函数的底层功能
 *
 * @param fd  文件描述符（1=stdout, 2=stderr）
 * @param ptr 待写入数据的缓冲区指针
 * @param len 待写入数据的字节数
 * @return 成功返回写入的字节数，失败返回-1
 *
 * 实现细节：
 *   1. 检查fd是否为stdout或stderr
 *   2. 调用HAL_UART_Transmit()发送数据
 *   3. 返回实际发送的字节数（len）或错误码
 *
 * @note HAL_UART_Transmit()参数：
 *       - gHuart: UART句柄
 *       - ptr: 数据缓冲区
 *       - len: 发送长度
 *       - HAL_MAX_DELAY: 无限等待直到发送完成
 *
 * @warning 这是阻塞函数，会等待所有数据发送完成
 * @warning 如果UART未正确初始化，可能导致系统挂起
 *
 * 性能考虑：
 *   - 115200波特率下，发送1字节约需87us
 *   - printf()大量输出时会占用较多CPU时间
 *   - 建议生产环境减少printf()调用频率
 *
 * 典型调用链：
 *   printf() → vfprintf() → _write() → HAL_UART_Transmit()
 */
int _write(int fd, char *ptr, int len)
{
    HAL_StatusTypeDef hstatus;

    /* 只处理stdout和stderr */
    if (fd == STDOUT_FILENO || fd == STDERR_FILENO)
    {
        /**
         * @note 通过UART发送数据
         * @param gHuart: UART句柄（在RetargetInit中设置）
         * @param ptr: 待发送的数据
         * @param len: 数据长度
         * @param HAL_MAX_DELAY: 超时时间（永远等待）
         */
        hstatus = HAL_UART_Transmit(gHuart, (uint8_t *) ptr, len, HAL_MAX_DELAY);

        if (hstatus == HAL_OK)
            return len;  // 发送成功，返回发送字节数
        else
            return EIO;   // 发送失败，返回I/O错误
    }

    /* 不是有效的文件描述符 */
    errno = EBADF;
    return -1;
}

/**
 * @brief 关闭文件描述符
 * @details 关闭指定的文件描述符（本实现为空操作）
 *
 * @param fd 文件描述符
 * @return 成功返回0，失败返回-1
 *
 * 实现说明：
 *   - 标准I/O流（stdin/stdout/stderr）不支持关闭
 *   - 返回0表示成功（但实际上什么都没做）
 *
 * @note 在某些newlib实现中，这个函数可能需要更复杂的逻辑
 * @note 对于简单的UART重定向，空操作即可
 */
int _close(int fd)
{
    /* 标准I/O流不需要关闭 */
    if (fd >= STDIN_FILENO && fd <= STDERR_FILENO)
        return 0;

    /* 不是有效的文件描述符 */
    errno = EBADF;
    return -1;
}

/**
 * @brief 移动文件读写位置
 * @details 设置文件的读写位置指针（本实现不支持）
 *
 * @param fd  文件描述符
 * @param ptr 偏移量（字节数）
 * @param dir 起始位置（SEEK_SET/SEEK_CUR/SEEK_END）
 * @return 成功返回新的位置，失败返回-1
 *
 * 实现说明：
 *   - UART是流设备，不支持随机访问
 *   - 总是返回错误，表示不支持此操作
 *
 * @note 对于UART流，lseek操作没有意义
 * @note 某些C库函数可能调用此函数，必须提供实现
 */
int _lseek(int fd, int ptr, int dir)
{
    /* 避免未使用参数警告 */
    (void) fd;
    (void) ptr;
    (void) dir;

    /* UART流不支持随机访问 */
    errno = EBADF;
    return -1;
}

/**
 * @brief 从文件描述符读取数据
 * @details 实现scanf()、getchar()等输入函数的底层功能
 *
 * @param fd  文件描述符（0=stdin）
 * @param ptr 接收数据的缓冲区指针
 * @param len 期望读取的字节数（实际被忽略）
 * @return 成功返回读取的字节数，失败返回-1
 *
 * 实现细节：
 *   1. 只处理stdin（标准输入）
 *   2. 每次只读取1个字符，忽略len参数
 *   3. 使用HAL_UART_Receive()阻塞接收
 *   4. 返回1表示成功读取1个字符
 *
 * @note HAL_UART_Receive()参数：
 *       - gHuart: UART句柄
 *       - ptr: 接收缓冲区
 *       - 1: 接收长度（固定为1字节）
 *       - HAL_MAX_DELAY: 无限等待直到收到数据
 *
 * @warning 这是阻塞函数，会一直等待直到收到字符
 * @warning 如果UART没有数据输入，系统会在此处挂起
 *
 * 典型调用链：
 *   scanf() → vfscanf() → _read() → HAL_UART_Receive()
 *   getchar() → _read() → HAL_UART_Receive()
 *
 * 使用注意事项：
 *   - 确保UART有数据输入再调用scanf()
 *   - 或者使用超时机制避免永久阻塞
 *   - 建议在嵌入式系统中谨慎使用scanf()
 */
int _read(int fd, char *ptr, int len)
{
    HAL_StatusTypeDef hstatus;

    /* 只处理stdin */
    if (fd == STDIN_FILENO)
    {
        /**
         * @note 从UART接收1个字符
         * @param gHuart: UART句柄
         * @param ptr: 接收缓冲区
         * @param 1: 接收长度（固定1字节）
         * @param HAL_MAX_DELAY: 超时时间（永远等待）
         *
         * @note len参数被忽略，每次只读取1个字符
         * @warning 这会阻塞程序执行，直到收到数据
         */
        hstatus = HAL_UART_Receive(gHuart, (uint8_t *) ptr, 1, HAL_MAX_DELAY);

        if (hstatus == HAL_OK)
            return 1;  // 接收成功，返回1字节
        else
            return EIO; // 接收失败，返回I/O错误
    }

    /* 不是有效的文件描述符 */
    errno = EBADF;
    return -1;
}

/**
 * @brief 获取文件状态信息
 * @details 查询文件的类型、大小、权限等属性
 *
 * @param fd 文件描述符
 * @param st 指向stat结构体的指针，用于返回文件状态
 * @return 成功返回0，失败返回-1
 *
 * 实现说明：
 *   1. 检查fd是否为标准I/O流
 *   2. 设置st_mode为S_IFCHR（字符设备）
 *   3. 返回0表示成功
 *
 * @note S_IFCHR: Character Special Device（字符设备）
 * @note 字符设备包括终端、串口等流式设备
 * @note 与块设备（如磁盘）相对，不支持随机访问
 *
 * stat结构体主要字段：
 *   - st_mode: 文件类型和权限
 *   - st_size: 文件大小（字节设备为0）
 *   - st_mtime: 最后修改时间
 */
int _fstat(int fd, struct stat *st)
{
    /* 检查是否为标准I/O流 */
    if (fd >= STDIN_FILENO && fd <= STDERR_FILENO)
    {
        /* 标记为字符设备 */
        st->st_mode = S_IFCHR;
        return 0;  // 成功
    }

    /* 不是有效的文件描述符 */
    errno = EBADF;
    return 0;
}

/**
 * @brief 增加程序堆空间（可选实现）
 * @details 用于在程序运行时动态增加堆内存空间
 *
 * @param incr 增加的字节数
 * @return 新的堆顶位置
 *
 * @note 本文件未实现此函数
 *       如果需要，可以在此添加实现
 */

/**
 * @brief 获取当前进程时间（可选实现）
 * @details 返回进程使用的CPU时间
 *
 * @note 本文件未实现此函数
 *       如果需要，可以在此添加实现
 */

/* ==================== 文件结束 ==================== */
#endif //#if !defined(OS_USE_SEMIHOSTING)
