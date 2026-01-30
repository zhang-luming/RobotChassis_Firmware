/**
 ******************************************************************************
 * @file    ptp_sync.h
 * @brief   PTP时间同步模块
 ******************************************************************************
 */

#ifndef __PTP_SYNC_H__
#define __PTP_SYNC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* ==================== PTP协议定义 ==================== */

/* PTP时间同步功能码 */
#define PTP_FUNC_CODE              0x10

/* PTP帧类型 */
typedef enum {
    PTP_MSG_SYNC_REQUEST = 0x01,    /* 同步请求 */
    PTP_MSG_SYNC_RESPONSE = 0x02,   /* 同步响应 */
} PtpMsgType_t;

/* ==================== 数据结构 ==================== */

/**
 * @brief PTP同步状态
 */
typedef struct {
    int64_t  offset_us;             /* 时间偏移量（微秒，有符号） */
    uint64_t local_last_sync;       /* 本地最后同步时间 */
    uint32_t sync_count;            /* 同步次数 */
    uint8_t  synchronized;          /* 是否已同步 */
} PtpSyncState_t;

/* ==================== 核心接口 ==================== */

/**
 * @brief 初始化PTP时间同步模块
 */
void PTP_Init(void);

/**
 * @brief 处理PTP时间同步请求帧
 * @param frame 接收到的帧数据
 * @param frame_len 帧长度
 *
 * @note 收到请求时记录t2，发送响应时记录t3
 */
void PTP_ProcessFrame(uint8_t* frame, uint16_t frame_len);

/**
 * @brief 获取PTP同步状态
 * @return PTP同步状态指针
 */
const PtpSyncState_t* PTP_GetState(void);

/**
 * @brief 将本地时间戳转换为Linux时间戳
 * @param local_us 本地微秒时间戳
 * @return Linux微秒时间戳
 */
int64_t PTP_LocalToLinux(uint64_t local_us);

/**
 * @brief 将Linux时间戳转换为本地时间戳
 * @param linux_us Linux微秒时间戳
 * @return 本地微秒时间戳
 */
uint64_t PTP_LinuxToLocal(int64_t linux_us);

/* ==================== 调试接口 ==================== */

/**
 * @brief 手动设置时间偏移量（用于测试）
 * @param offset_us 偏移量（微秒）
 */
void PTP_SetOffset(int64_t offset_us);

#ifdef __cplusplus
}
#endif

#endif /* __PTP_SYNC_H__ */
