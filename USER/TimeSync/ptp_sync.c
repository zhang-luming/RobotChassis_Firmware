/**
 ******************************************************************************
 * @file    ptp_sync.c
 * @brief   PTP时间同步模块实现
 ******************************************************************************
 */

#include "ptp_sync.h"
#include "System/timer.h"
#include "System/debug.h"
#include "comm_protocol.h"
#include <string.h>

/* ==================== 私有变量 ==================== */

static PtpSyncState_t g_ptp_state = {
    .offset_us = 0,
    .local_last_sync = 0,
    .sync_count = 0,
    .synchronized = 0,
};

/* ==================== 私有函数 ==================== */

/**
 * @brief 发送PTP同步响应帧
 * @param seq_num 序列号
 * @param t2 接收时间戳（微秒）
 * @param t3 发送时间戳（微秒）
 *
 * 数据格式：[msg_type][seq][t2_low32][t3_low32]
 * 注意：为了简化协议，只发送时间戳的低32位
 */
static void send_sync_response(uint8_t seq_num, uint64_t t2, uint64_t t3) {
    /* 发送完整的64位时间戳（需要8个int16_t）
     * 数据布局：
     * [0] = (seq_num << 8) | PTP_MSG_SYNC_RESPONSE
     * [1-4] = t2 (64位，拆分为4个16位)
     * [5-8] = t3 (64位，拆分为4个16位)
     */
    int16_t data[9];

    data[0] = (int16_t)((seq_num << 8) | PTP_MSG_SYNC_RESPONSE);

    /* 将t2拆分为4个int16_t */
    data[1] = (int16_t)(t2 & 0xFFFF);         // t2 [15:0]
    data[2] = (int16_t)((t2 >> 16) & 0xFFFF);  // t2 [31:16]
    data[3] = (int16_t)((t2 >> 32) & 0xFFFF);  // t2 [47:32]
    data[4] = (int16_t)((t2 >> 48) & 0xFFFF);  // t2 [63:48]

    /* 将t3拆分为4个int16_t */
    data[5] = (int16_t)(t3 & 0xFFFF);         // t3 [15:0]
    data[6] = (int16_t)((t3 >> 16) & 0xFFFF);  // t3 [31:16]
    data[7] = (int16_t)((t3 >> 32) & 0xFFFF);  // t3 [47:32]
    data[8] = (int16_t)((t3 >> 48) & 0xFFFF);  // t3 [63:48]

    /* data_len = 9个int16_t */
    Comm_SendDataFrame(PTP_FUNC_CODE, data, 9);

    DEBUG_VERBOSE("[PTP] Resp: seq=%d, t2=%lu, t3=%lu\r\n",
                  seq_num, (unsigned long)t2, (unsigned long)t3);
}

/* ==================== 公共接口实现 ==================== */

void PTP_Init(void) {
    memset(&g_ptp_state, 0, sizeof(PtpSyncState_t));
    DEBUG_INFO("[PTP] Initialized\r\n");
}

void PTP_ProcessFrame(uint8_t* frame, uint16_t frame_len) {
    if (frame_len < 5) {
        DEBUG_WARN("[PTP] Frame too short\r\n");
        return;
    }

    uint8_t msg_type = frame[2];
    uint8_t seq_num = frame[3];

    if (msg_type == PTP_MSG_SYNC_REQUEST) {
        /* 记录接收时间戳 t2 */
        uint64_t t2 = Time_GetUs();

        DEBUG_INFO("[PTP] Sync req seq=%d t2=%lu\r\n", seq_num, (unsigned long)t2);

        /* 立即发送响应，记录发送时间戳 t3 */
        uint64_t t3 = Time_GetUs();

        /* 发送响应（包含t2和t3，offset由上位机计算） */
        send_sync_response(seq_num, t2, t3);

        DEBUG_INFO("[PTP] Resp sent t3=%lu\r\n", (unsigned long)t3);

        /* 更新状态 */
        g_ptp_state.local_last_sync = t2;
        g_ptp_state.sync_count++;
    } else {
        DEBUG_WARN("[PTP] Unknown msg type: 0x%02X\r\n", msg_type);
    }
}

const PtpSyncState_t* PTP_GetState(void) {
    return &g_ptp_state;
}

int64_t PTP_LocalToLinux(uint64_t local_us) {
    if (!g_ptp_state.synchronized) {
        DEBUG_WARN("[PTP] Not synchronized, returning local time\r\n");
        return (int64_t)local_us;
    }

    /* Linux时间 = 本地时间 + offset */
    return (int64_t)local_us + g_ptp_state.offset_us;
}

uint64_t PTP_LinuxToLocal(int64_t linux_us) {
    if (!g_ptp_state.synchronized) {
        DEBUG_WARN("[PTP] Not synchronized, returning Linux time\r\n");
        return (uint64_t)linux_us;
    }

    /* 本地时间 = Linux时间 - offset */
    int64_t local = linux_us - g_ptp_state.offset_us;

    /* 处理溢出 */
    if (local < 0) {
        return 0;
    }

    return (uint64_t)local;
}

void PTP_SetOffset(int64_t offset_us) {
    g_ptp_state.offset_us = offset_us;
    g_ptp_state.synchronized = 1;

    DEBUG_INFO("[PTP] Offset set to %ld us\r\n", (long)offset_us);
}
