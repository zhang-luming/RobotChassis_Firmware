/**
 ******************************************************************************
 * @file    comm_protocol.c
 * @brief   通信协议模块 - 串口通信
 *
 * 功能说明：
 * - 接收控制指令并分发到各模块
 * - 提供数据发送接口
 ******************************************************************************
 */

#include "comm_protocol.h"

#include "System/debug.h"
#include "TimeSync/ptp_sync.h"
#include "motor_control.h"
#include "servo_control.h"
#include "stdio.h"
#include "inttypes.h"
#include "string.h"
#include "tim.h"
#include "usart.h"
#include "user_config.h"

/* ==================== 私有变量 ==================== */

/* 接收缓冲区 - 循环缓冲区设计 */
#define RX_BUFFER_SIZE 256
static uint8_t g_rx_buffer[RX_BUFFER_SIZE];
static volatile uint16_t g_rx_write_idx = 0; /* 写指针（中断中） */
static volatile uint16_t g_rx_read_idx = 0;  /* 读指针（主循环中） */

/* 单字节接收缓冲（用于HAL库） */
static uint8_t g_rx_byte = 0;

/* 串口发送缓冲 */
uint8_t UART_SEND_BUF[128];

/* 调试：上一次打印的字节数 */
static uint16_t g_last_printed_bytes = 0;

/* PTP帧解析状态（用于中断中快速检测） */
typedef enum {
    PTP_STATE_IDLE = 0,         /* 空闲状态 */
    PTP_STATE_GOT_HEADER,       /* 收到帧头0xFC */
    PTP_STATE_GOT_FUNC,         /* 收到功能码 */
    PTP_STATE_GOT_MSG_TYPE,     /* 收到消息类型 */
    PTP_STATE_GOT_SEQ,          /* 收到序列号 */
} PtpParseState_t;

static PtpParseState_t g_ptp_parse_state = PTP_STATE_IDLE;
static uint8_t g_ptp_seq_num = 0;

/* ==================== 公共接口实现 ==================== */

/**
 * @brief 初始化通信模块
 */
void Comm_Init(void) {
  memset((void*)g_rx_buffer, 0, RX_BUFFER_SIZE);
  g_rx_write_idx = 0;
  g_rx_read_idx = 0;
  g_last_printed_bytes = 0;

  /* 启动串口接收中断 */
  HAL_UART_Receive_IT(&huart2, &g_rx_byte, 1);
}

/**
 * @brief 从缓冲区提取一帧数据
 * @param frame 输出帧缓冲区
 * @param max_len 帧缓冲区最大长度
 * @return 帧长度，0表示未找到完整帧
 */
static uint16_t ExtractFrame(uint8_t* frame, uint16_t max_len) {
  uint16_t frame_len = 0;

  /* 在缓冲区中查找帧头FC */
  while (g_rx_read_idx != g_rx_write_idx) {
    /* 读取当前字节 */
    uint8_t byte = g_rx_buffer[g_rx_read_idx];

    /* 查找帧头 */
    if (byte == PROTOCOL_HEADER) {
      /* 找到帧头，检查是否有足够的数据（最小帧：FC + Func + Checksum + DF =
       * 4字节） */
      uint16_t available =
          (g_rx_write_idx >= g_rx_read_idx)
              ? (g_rx_write_idx - g_rx_read_idx)
              : (RX_BUFFER_SIZE - g_rx_read_idx + g_rx_write_idx);

      if (available < 4) {
        /* 数据不足，等待更多数据 */
        return 0;
      }

      /* 读取帧头和功能码 */
      frame[0] = byte;
      frame[1] =
          g_rx_buffer[(g_rx_read_idx + 1) % RX_BUFFER_SIZE];  // 防越界处理

      /* 查找帧尾DF（从索引2开始搜索） */
      uint16_t search_start = 2;
      uint16_t tail_pos = 0;

      for (uint16_t i = search_start; i < available && i < max_len; i++) {
        uint8_t b = g_rx_buffer[(g_rx_read_idx + i) % RX_BUFFER_SIZE];  // 越界处理同上
        frame[i] = b;

        if (b == PROTOCOL_TAIL) {
          /* 找到帧尾 */
          tail_pos = i;
          break;
        }
      }

      if (tail_pos > 0) {
        /* 找到完整帧 */
        frame_len = tail_pos + 1;

        /* 校验和验证（不包括帧尾） */
        uint8_t checksum_len = frame_len - 2;  // 减去校验位和帧尾
        uint8_t calc_checksum = Comm_XORCheck(frame, checksum_len);
        uint8_t recv_checksum = frame[checksum_len];

        if (calc_checksum == recv_checksum) {
          /* 校验通过，移动读指针 */
          g_rx_read_idx = (g_rx_read_idx + frame_len) % RX_BUFFER_SIZE;
          return frame_len;
        } else {
          /* 校验失败，跳过这个帧头，继续搜索 */
          printf("[Frame] 校验失败: Calc=0x%02X, Recv=0x%02X\r\n",
                 calc_checksum, recv_checksum);
          g_rx_read_idx = (g_rx_read_idx + 1) % RX_BUFFER_SIZE;
          return 0;
        }
      } else {
        /* 未找到帧尾，数据不完整，等待更多数据 */
        return 0;
      }
    } else {
      /* 不是帧头，跳过这个字节 */
      g_rx_read_idx = (g_rx_read_idx + 1) % RX_BUFFER_SIZE;
    }
  }

  return 0;
}

/**
 * @brief 处理协议帧
 * @param frame 帧数据
 * @param frame_len 帧长度
 */
static void ProcessFrame(uint8_t* frame, uint16_t frame_len) {
  uint8_t func_code = frame[1];

  printf("[Frame] FC=0x%02X, Len=%d\r\n", func_code, frame_len);

  /* 根据功能码分发 */
  switch (func_code) {
    case FUNC_MOTOR_SPEED: /* 0x06 电机目标速度 */
      Motor_ProcessSpeedFrame(frame, frame_len);
      break;

    case FUNC_PID_PARAM: /* 0x07 PID参数 */
      Motor_ProcessPIDFrame(frame, frame_len);
      break;

    case FUNC_SERVO_CONTROL: /* 0x08 舵机控制 */
      printf("[Servo] 舵机控制 - 待实现\r\n");
      // TODO: 实现舵机控制
      break;

    case FUNC_PTP_SYNC: /* 0x10 PTP时间同步 */
      /* 检查是否为同步请求（已被快速响应处理） */
      if (frame_len >= 3 && frame[2] == PTP_SYNC_REQUEST) {
        /* 同步请求已在快速响应中处理，跳过 */
        DEBUG_VERBOSE("[Frame] PTP sync request already handled in fast path\r\n");
      } else {
        /* 其他PTP消息类型（如设置offset等）正常处理 */
        PTP_ProcessFrame(frame, frame_len);
      }
      break;

    default:
      printf("[Frame] 未知功能码: 0x%02X\r\n", func_code);
      break;
  }
}

/**
 * @brief 更新通信模块（每10ms调用）
 *
 * 功能：
 * - 从缓冲区解析协议帧
 * - 分发到各处理函数
 */
void Comm_Update(void) {
  static uint8_t frame_buffer[64];  // 帧缓冲区

  /* 持续从缓冲区提取帧，直到没有完整帧 */
  while (1) {
    uint16_t frame_len = ExtractFrame(frame_buffer, sizeof(frame_buffer));

    if (frame_len == 0) {
      /* 没有完整帧，退出 */
      break;
    }

    /* 处理帧 */
    ProcessFrame(frame_buffer, frame_len);
  }
}

/**
 * @brief 串口接收中断回调
 * @note 在中断中调用，接收数据并快速检测PTP帧
 */
void Comm_RxCallback(UART_HandleTypeDef* huart) {
  if (huart->Instance == USART2) {
    /* 将接收到的字节写入循环缓冲区 */
    uint16_t next_idx = (g_rx_write_idx + 1) % RX_BUFFER_SIZE;

    /* 缓冲区未满时才写入 */
    if (next_idx != g_rx_read_idx) {
      g_rx_buffer[g_rx_write_idx] = g_rx_byte;
      g_rx_write_idx = next_idx;
    }
    /* 缓冲区满时丢弃新字节 */

    /* PTP快速检测：在中断中识别PTP同步请求 */
    switch (g_ptp_parse_state) {
      case PTP_STATE_IDLE:
        if (g_rx_byte == PROTOCOL_HEADER) {
          g_ptp_parse_state = PTP_STATE_GOT_HEADER;
        }
        break;

      case PTP_STATE_GOT_HEADER:
        if (g_rx_byte == FUNC_PTP_SYNC) {
          g_ptp_parse_state = PTP_STATE_GOT_FUNC;
        } else {
          g_ptp_parse_state = PTP_STATE_IDLE;
        }
        break;

      case PTP_STATE_GOT_FUNC:
        if (g_rx_byte == PTP_SYNC_REQUEST) {
          g_ptp_parse_state = PTP_STATE_GOT_MSG_TYPE;
        } else {
          g_ptp_parse_state = PTP_STATE_IDLE;
        }
        break;

      case PTP_STATE_GOT_MSG_TYPE:
        /* 收到序列号，记录并等待帧尾 */
        g_ptp_seq_num = g_rx_byte;
        g_ptp_parse_state = PTP_STATE_GOT_SEQ;
        break;

      case PTP_STATE_GOT_SEQ:
        /* 等待帧尾0xDF */
        if (g_rx_byte == PROTOCOL_TAIL) {
          /* 完整的PTP同步请求帧，在中断中立即处理 */
          extern uint64_t Time_GetUs(void);

          /* 记录t2时间戳 */
          uint64_t t2 = Time_GetUs();

          /* 构建PTP响应帧（只包含t2，t3由Comm_SendDataFrame自动添加到帧末尾） */
          int16_t data[5];
          data[0] = (int16_t)((g_ptp_seq_num << 8) | 0x02);  /* 响应类型 */

          /* 将t2拆分为4个int16_t */
          data[1] = (int16_t)(t2 & 0xFFFF);
          data[2] = (int16_t)((t2 >> 16) & 0xFFFF);
          data[3] = (int16_t)((t2 >> 32) & 0xFFFF);
          data[4] = (int16_t)((t2 >> 48) & 0xFFFF);

          /* Comm_SendDataFrame会自动添加发送时间戳（tx_timestamp = t3）到帧末尾 */
          Comm_SendDataFrame(FUNC_PTP_SYNC, data, 5);

          g_ptp_parse_state = PTP_STATE_IDLE;

          DEBUG_VERBOSE("[PTP] IRQ resp: seq=%d, t2=%lu\r\n",
                        g_ptp_seq_num, (unsigned long)t2);
        } else if (g_rx_byte == PROTOCOL_HEADER) {
          /* 异常情况：新的帧开始，重置状态 */
          g_ptp_parse_state = PTP_STATE_GOT_HEADER;
        }
        /* 其他字节继续等待帧尾 */
        break;

      default:
        g_ptp_parse_state = PTP_STATE_IDLE;
        break;
    }
  }
}

/**
 * @brief UART接收完成回调（HAL库弱函数）
 * @param huart UART句柄
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
  if (huart->Instance == USART2) {
    /* 调用接收回调 */
    Comm_RxCallback(huart);
  }

  /* 重新启动接收中断 */
  HAL_UART_Receive_IT(&huart2, &g_rx_byte, 1);
}

/* ==================== 发送函数实现 ==================== */

/**
 * @brief 异或校验
 */
uint8_t Comm_XORCheck(uint8_t* a, uint8_t len) {
  uint8_t XOR = 0;
  uint8_t i = 0;
  while (i < len) {
    XOR ^= a[i];
    i++;
  }
  return XOR;
}

/**
 * @brief 串口发送数组函数
 */
void Comm_SendBuf(USART_TypeDef* USART_COM, uint8_t* buf, uint16_t len) {
  while (len--) {
    while ((USART_COM->SR & 0X40) == 0); /* 等待发送缓冲区空 */
    USART_COM->DR = (uint8_t)(*buf++);
    while ((USART_COM->SR & 0X40) == 0); /* 等待发送完成 */
  }
}

/**
 * @brief 公共接口：发送协议数据帧
 * @param func_code 功能码
 * @param data 数据指针（int16_t数组）
 * @param data_len 数据长度（int16_t个数）
 *
 * 帧格式：[FC][Func][Data...][TxTimestamp(8 bytes)][Checksum][DF]
 * - 所有数据统一使用小端序（Little-Endian）
 * - TxTimestamp: 本帧开始发送的时间戳（64位微秒）
 */
void Comm_SendDataFrame(uint8_t func_code, int16_t* data, uint8_t data_len) {
  extern uint64_t Time_GetUs(void);

  /* 构建协议帧 */
  UART_SEND_BUF[0] = PROTOCOL_HEADER;
  UART_SEND_BUF[1] = func_code;

  /* 复制数据（int16_t转uint8_t，小端序） */
  uint8_t data_idx = 2;
  for (uint8_t i = 0; i < data_len; i++) {
    int16_t value = data[i];
    UART_SEND_BUF[data_idx++] = value & 0xFF;          /* 低字节在前 */
    UART_SEND_BUF[data_idx++] = (value >> 8) & 0xFF;  /* 高字节在后 */
  }

  /* 添加发送时间戳（8字节，64位微秒时间戳，小端序） */
  uint64_t tx_timestamp = Time_GetUs();
  for (int i = 0; i < 8; i++) {
    UART_SEND_BUF[data_idx++] = (tx_timestamp >> (i * 8)) & 0xFF;
  }

  /* 计算校验和（包括时间戳） */
  UART_SEND_BUF[data_idx] = Comm_XORCheck(UART_SEND_BUF, data_idx);
  UART_SEND_BUF[data_idx + 1] = PROTOCOL_TAIL;

  /* 发送 */
  Comm_SendBuf(USART2, UART_SEND_BUF, data_idx + 2);
}
