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
#include "motor_control.h"
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
    PTP_STATE_GOT_CHECKSUM,     /* 收到校验和 */
} PtpParseState_t;

static PtpParseState_t g_ptp_parse_state = PTP_STATE_IDLE;

/* ==================== DMA发送队列 ==================== */

/* DMA发送队列（环形缓冲区） */
static DmaTxFrame_t g_dma_tx_queue[DMA_TX_QUEUE_SIZE];

/* 队列读写索引 */
static volatile uint8_t g_queue_write_idx = 0;  /* 写指针（入队） */
static volatile uint8_t g_queue_read_idx = 0;   /* 读指针（出队/DMA发送） */

/* 队列状态 */
static volatile uint8_t g_queue_count = 0;      /* 当前队列中的帧数 */

/**
 * @brief 检查队列是否为空
 */
static inline uint8_t Queue_IsEmpty(void) {
  return g_queue_count == 0;
}

/**
 * @brief 检查队列是否已满
 */
static inline uint8_t Queue_IsFull(void) {
  return g_queue_count >= DMA_TX_QUEUE_SIZE;
}

/**
 * @brief 获取队列中的帧数
 */
static inline uint8_t Queue_GetCount(void) {
  return g_queue_count;
}

/**
 * @brief 启动DMA发送队列中的下一帧
 *
 * 从队列中取出一帧并启动DMA传输，在DMA发送完成中断中自动调用
 */
void DMA_StartNextFrame(void);

/**
 * @brief 构建协议帧
 * @param buf 输出缓冲区
 * @param func_code 功能码
 * @param data 数据指针（int16_t数组）
 * @param data_len 数据长度（int16_t个数）
 * @return 帧长度
 */
static uint8_t BuildDataFrame(uint8_t* buf, uint8_t func_code,
                               int16_t* data, uint8_t data_len);

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
    case FUNC_MOTOR_SPEED: /* 0x04 电机目标速度 */
      Motor_ProcessSpeedFrame(frame, frame_len);
      break;

    case FUNC_PID_PARAM: /* 0x05 PID参数 */
      Motor_ProcessPIDFrame(frame, frame_len);
      break;

    case FUNC_PTP_SYNC: /* 0x10 PTP时间同步 */
      /* PTP同步请求已在UART接收中断中快速处理，此处跳过 */
      DEBUG_VERBOSE("[Frame] PTP sync frame already handled in IRQ\r\n");
      break;

    default:
      printf("[Frame] 未知功能码: 0x%02X\r\n", func_code);
      break;
  }
}

/**
 * @brief 接收并处理通信数据（每10ms调用）
 *
 * 从缓冲区解析协议帧并分发到各处理函数
 */
void Comm_Receive(void) {
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
        /* 收到校验和字节，跳过校验，等待帧尾 */
        g_ptp_parse_state = PTP_STATE_GOT_CHECKSUM;
        break;

      case PTP_STATE_GOT_CHECKSUM:
        /* 等待帧尾0xDF（请求帧格式：[FC][Func][MsgType][Checksum][DF]） */
        if (g_rx_byte == PROTOCOL_TAIL) {
          /* 完整的PTP同步请求帧，在中断中立即处理 */
          extern uint64_t Time_GetUs(void);

          /* 记录t2时间戳 */
          uint64_t t2 = Time_GetUs();

          /* 构建PTP响应帧：仅包含t2（4个int16_t），t3由Comm_SendDataFrame自动添加 */
          int16_t data[4];
          data[0] = (int16_t)(t2 & 0xFFFF);
          data[1] = (int16_t)((t2 >> 16) & 0xFFFF);
          data[2] = (int16_t)((t2 >> 32) & 0xFFFF);
          data[3] = (int16_t)((t2 >> 48) & 0xFFFF);

          /* Comm_SendDataFrame会自动添加发送时间戳（tx_timestamp = t3）到帧末尾 */
          Comm_SendDataFrame(FUNC_PTP_SYNC, data, 4);

          g_ptp_parse_state = PTP_STATE_IDLE;

          DEBUG_VERBOSE("[PTP] IRQ resp: t2=%lu\r\n", (unsigned long)t2);
        } else if (g_rx_byte == PROTOCOL_HEADER) {
          /* 异常情况：新的帧开始，重置状态 */
          g_ptp_parse_state = PTP_STATE_GOT_HEADER;
        } else {
          /* 其他字节：忽略（不符合协议的请求帧） */
          g_ptp_parse_state = PTP_STATE_IDLE;
        }
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
 * @brief 串口发送数组函数（阻塞方式）
 */
void Comm_SendBuf(USART_TypeDef* USART_COM, uint8_t* buf, uint16_t len) {
  while (len--) {
    while ((USART_COM->SR & 0X40) == 0); /* 等待发送缓冲区空 */
    USART_COM->DR = (uint8_t)(*buf++);
    while ((USART_COM->SR & 0X40) == 0); /* 等待发送完成 */
  }
}

/**
 * @brief 发送协议数据帧（阻塞方式）
 * @param func_code 功能码
 * @param data 数据指针（int16_t数组）
 * @param data_len 数据长度（int16_t个数）
 *
 * 帧格式：[FC][Func][Data...][TxTimestamp(8B)][Checksum][DF]
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

/* ==================== DMA发送队列实现 ==================== */

/**
 * @brief 构建协议帧
 * @param buf 输出缓冲区
 * @param func_code 功能码
 * @param data 数据指针（int16_t数组）
 * @param data_len 数据长度（int16_t个数）
 * @return 帧长度
 */
static uint8_t BuildDataFrame(uint8_t* buf, uint8_t func_code,
                               int16_t* data, uint8_t data_len) {
  extern uint64_t Time_GetUs(void);

  /* 构建协议帧 */
  buf[0] = PROTOCOL_HEADER;
  buf[1] = func_code;

  /* 复制数据（int16_t转uint8_t，小端序） */
  uint8_t data_idx = 2;
  for (uint8_t i = 0; i < data_len; i++) {
    int16_t value = data[i];
    buf[data_idx++] = value & 0xFF;          /* 低字节在前 */
    buf[data_idx++] = (value >> 8) & 0xFF;  /* 高字节在后 */
  }

  /* 添加发送时间戳（8字节，64位微秒时间戳，小端序） */
  uint64_t tx_timestamp = Time_GetUs();
  for (int i = 0; i < 8; i++) {
    buf[data_idx++] = (tx_timestamp >> (i * 8)) & 0xFF;
  }

  /* 计算校验和（包括时间戳） */
  buf[data_idx] = Comm_XORCheck(buf, data_idx);
  buf[data_idx + 1] = PROTOCOL_TAIL;

  return data_idx + 2;  /* 返回帧长度 */
}

/**
 * @brief 启动DMA发送队列中的下一帧
 *
 * 从队列中取出一帧并启动DMA传输，在DMA发送完成中断中自动调用
 */
void DMA_StartNextFrame(void) {
  extern void USART2_DMA_SetBusy(uint8_t busy);

  /* 检查队列是否为空 */
  if (Queue_IsEmpty()) {
    /* 队列为空，清除DMA忙碌标志 */
    USART2_DMA_SetBusy(0);
    return;
  }

  /* 从队列中取出一帧 */
  DmaTxFrame_t* frame = &g_dma_tx_queue[g_queue_read_idx];

  /* 启动DMA发送 */
  HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(&huart2, frame->data, frame->len);

  if (status == HAL_OK) {
    /* 发送成功，更新读指针 */
    g_queue_read_idx = (g_queue_read_idx + 1) % DMA_TX_QUEUE_SIZE;
    g_queue_count--;
  } else {
    /* 发送失败，清除忙碌标志，保持帧在队列中 */
    USART2_DMA_SetBusy(0);
    DEBUG_ERROR("[DMA] 启动失败: %d\r\n", status);
  }
}

/**
 * @brief DMA方式发送协议数据帧（非阻塞，使用发送队列）
 * @param func_code 功能码
 * @param data 数据指针（int16_t数组）
 * @param data_len 数据长度（int16_t个数）
 * @return HAL_OK=成功入队，HAL_ERROR=队列已满
 */
HAL_StatusTypeDef Comm_SendDataFrameDMA(uint8_t func_code, int16_t* data, uint8_t data_len) {
  extern uint8_t USART2_DMA_IsBusy(void);

  /* 计算帧长度 */
  uint8_t frame_len = 2 + (data_len * 2) + 8 + 2;  /* 帧头+功能码 + 数据 + 时间戳 + 校验+帧尾 */

  /* 检查缓冲区大小 */
  if (frame_len > DMA_TX_FRAME_SIZE) {
    return HAL_ERROR;
  }

  /* 检查队列是否已满 */
  if (Queue_IsFull()) {
    DEBUG_WARN("[DMA] 队列已满，丢弃帧 (FC=0x%02X)\r\n", func_code);
    return HAL_ERROR;
  }

  /* 构建协议帧到队列缓冲区 */
  DmaTxFrame_t* frame = &g_dma_tx_queue[g_queue_write_idx];
  frame->len = BuildDataFrame(frame->data, func_code, data, data_len);

  /* 更新写指针 */
  g_queue_write_idx = (g_queue_write_idx + 1) % DMA_TX_QUEUE_SIZE;
  g_queue_count++;

  /* 如果DMA空闲，立即启动发送 */
  if (!USART2_DMA_IsBusy()) {
    DMA_StartNextFrame();
  }

  return HAL_OK;
}

