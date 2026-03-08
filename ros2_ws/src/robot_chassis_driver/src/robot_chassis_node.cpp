// Copyright 2026 RobotChassis Driver
// License: MIT

#include "robot_chassis_driver/robot_chassis_node.hpp"
#include <cmath>
#include <algorithm>

using namespace std::chrono_literals;

// ==================== 构造函数 ====================

RobotChassisNode::RobotChassisNode()
    : Node("robot_chassis_node"),
      running_(true),
      g_offset_(0),
      ptp_sync_count_(0),
      ptp_initialized_(false),
      last_t1_(0),
      last_t2_(0),
      ptp_t1_(0),
      ptp_integral_(0.0),
      ptp_converged_(false),
      sensor_frame_count_(0),
      ptp_seq_(0) {

  // 声明参数
  this->declare_parameter("serial_port", "/dev/ttyUSB0");
  this->declare_parameter("serial_baudrate", 921600);
  this->declare_parameter("ptp_interval_ms", 200.0);

  // 获取参数
  port_ = this->get_parameter("serial_port").as_string();
  baudrate_ = this->get_parameter("serial_baudrate").as_int();
  ptp_interval_ms_ = this->get_parameter("ptp_interval_ms").as_double();

  // 初始化串口
  if (!initSerial()) {
    RCLCPP_ERROR(this->get_logger(), "串口初始化失败");
    return;
  }

  // 启动串口读取线程
  read_thread_ = std::thread(&RobotChassisNode::serialReadThread, this);

  RCLCPP_INFO(this->get_logger(), "RobotChassis节点已启动");
  RCLCPP_INFO(this->get_logger(), "串口: %s, 波特率: %d", port_.c_str(), baudrate_);
  RCLCPP_INFO(this->get_logger(), "PTP同步: 启用, 间隔: %.0f ms", ptp_interval_ms_);
}

// ==================== 析构函数 ====================

RobotChassisNode::~RobotChassisNode() {
  running_ = false;
  if (read_thread_.joinable()) {
    read_thread_.join();
  }
  if (ser_.isOpen()) {
    ser_.close();
  }
  RCLCPP_INFO(this->get_logger(), "RobotChassis节点已关闭");
}

// ==================== 串口初始化 ====================

bool RobotChassisNode::initSerial() {
  try {
    ser_.setPort(port_);
    ser_.setBaudrate(baudrate_);
    ser_.setBytesize(serial::eightbits);
    ser_.setParity(serial::parity_none);
    ser_.setStopbits(serial::stopbits_one);
    ser_.setFlowcontrol(serial::flowcontrol_none);
    ser_.setTimeout(serial::Timeout::max(), 1000, 0, 1000, 0);

    ser_.open();
    if (ser_.isOpen()) {
      RCLCPP_INFO(this->get_logger(), "串口已打开: %s", port_.c_str());
      return true;
    }
  } catch (serial::SerialException& e) {
    RCLCPP_ERROR(this->get_logger(), "串口异常: %s", e.what());
  }
  return false;
}

// ==================== 工具函数 ====================

uint8_t RobotChassisNode::checksum(const uint8_t* data, size_t len) {
  uint8_t cs = 0;
  for (size_t i = 0; i < len; i++) {
    cs ^= data[i];
  }
  return cs;
}

int16_t RobotChassisNode::to_int16_le(uint8_t low, uint8_t high) const {
  return static_cast<int16_t>((static_cast<uint16_t>(high) << 8) | low);
}

int64_t RobotChassisNode::to_int64_le(const uint8_t* data) const {
  int64_t value = 0;
  for (int i = 0; i < 8; i++) {
    value |= (static_cast<int64_t>(data[i]) << (i * 8));
  }
  return value;
}

// ==================== 串口读取线程 ====================

void RobotChassisNode::serialReadThread() {
  RCLCPP_INFO(this->get_logger(), "串口读取线程已启动");

  auto last_ptp_time = std::chrono::steady_clock::now();

  while (running_ && rclcpp::ok()) {
    try {
      // ========== PTP同步（定时发送请求）==========
      auto now = std::chrono::steady_clock::now();
      auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_ptp_time).count();
      if (elapsed >= static_cast<int64_t>(ptp_interval_ms_)) {
        sendPTPRequest();
        last_ptp_time = now;
      }

      // ========== 读取串口数据 ==========
      if (ser_.available()) {
        size_t avail = ser_.available();
        std::vector<uint8_t> temp(avail);
        ser_.read(temp.data(), avail);

        std::lock_guard<std::mutex> lock(buffer_mutex_);
        recv_buffer_.insert(recv_buffer_.end(), temp.begin(), temp.end());
      }

      // ========== 解析缓冲区中的帧 ==========
      std::vector<uint8_t> local_buffer;
      {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        local_buffer = recv_buffer_;
      }

      while (local_buffer.size() >= 3) {  // 最小帧长度检查
        // 查找帧头
        if (local_buffer[0] != PROTOCOL_HEADER) {
          local_buffer.erase(local_buffer.begin());
          continue;
        }

        uint8_t func_code = local_buffer[1];
        size_t frame_len = 0;

        // 确定帧长度
        if (func_code == FUNC_SENSOR_MERGED) {
          frame_len = SENSOR_FRAME_SIZE;
        } else if (func_code == FUNC_PTP_SYNC) {
          frame_len = PTP_FRAME_SIZE;
        } else {
          // 未知功能码，跳过
          local_buffer.erase(local_buffer.begin());
          continue;
        }

        // 检查帧是否完整
        if (local_buffer.size() < frame_len) {
          break;
        }

        // 提取完整帧
        std::vector<uint8_t> frame(local_buffer.begin(),
                                    local_buffer.begin() + frame_len);
        local_buffer.erase(local_buffer.begin(), local_buffer.begin() + frame_len);

        // 验证帧尾
        if (frame[frame_len - 1] != PROTOCOL_TAIL) {
          continue;  // 静默跳过无效帧
        }

        // 验证校验和（不包括帧尾）
        uint8_t cs_recv = frame[frame_len - 2];
        uint8_t cs_calc = checksum(frame.data(), frame_len - 2);
        if (cs_recv != cs_calc) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                              "校验和错误: recv=%02X, calc=%02X", cs_recv, cs_calc);
          continue;
        }

        // 验证通过后，立即记录t4（帧接收完成的精确时刻）
        int64_t t4 = this->now().nanoseconds() / 1000;  // 微秒

        // 根据功能码处理帧
        if (func_code == FUNC_PTP_SYNC) {
          handlePTPFrame(frame, t4);
        } else if (func_code == FUNC_SENSOR_MERGED) {
          handleSensorFrame(frame);
        }
      }

      // 更新缓冲区
      {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        recv_buffer_ = local_buffer;
      }

    } catch (...) {
      RCLCPP_ERROR(this->get_logger(), "串口读取错误");
    }
  }

  RCLCPP_INFO(this->get_logger(), "串口读取线程已退出");
}

// ==================== PTP时间同步 ====================

void RobotChassisNode::sendPTPRequest() {
  // PTP请求帧：[0xFC][0x10][0x01][checksum][0xDF] (5字节)
  // 注意：MCU端校验和包括帧头，从frame[0]开始计算
  uint8_t frame_data[3];  // 不含校验和和帧尾
  frame_data[0] = PROTOCOL_HEADER;
  frame_data[1] = FUNC_PTP_SYNC;
  frame_data[2] = 0x01;  // PTP请求

  uint8_t cs = checksum(frame_data, 3);  // 包括帧头

  uint8_t full_frame[5];
  full_frame[0] = frame_data[0];
  full_frame[1] = frame_data[1];
  full_frame[2] = frame_data[2];
  full_frame[3] = cs;
  full_frame[4] = PROTOCOL_TAIL;

  try {
    ser_.write(full_frame, 5);
    ser_.flush();

    // 发送完成后立即记录t1（最接近实际发送时刻）
    ptp_t1_ = this->now().nanoseconds() / 1000;  // 微秒

    RCLCPP_INFO(this->get_logger(), "[PTP] 发送请求 (t1=%ld us)", ptp_t1_);
  } catch (serial::SerialException& e) {
    RCLCPP_ERROR(this->get_logger(), "[PTP] 发送失败: %s", e.what());
  }
}

void RobotChassisNode::handlePTPFrame(const std::vector<uint8_t>& frame,
                                      int64_t t4) {
  // t4是接收完成时刻（检测到帧尾后立即记录）
  // t1是发送完成时刻（在sendPTPRequest中记录）
  int64_t t1 = ptp_t1_;

  // PTP响应帧：[0xFC][0x10][t2(4×int16)][t3(8B)][checksum][0xDF]
  // 总长度：20字节
  // 索引0: 帧头 0xFC
  // 索引1: 功能码 0x10
  // 索引2-9: t2（4个uint16_t小端序）
  // 索引10-17: t3（tx_timestamp，8字节小端序）
  // 索引18: 校验和（包括帧头，frame[0:18]）
  // 索引19: 帧尾 0xDF

  // 解析t2（4个uint16_t小端序组成64位，无符号）
  uint64_t t2_parts[4];
  for (int i = 0; i < 4; i++) {
    uint8_t low = frame[2 + i * 2];
    uint8_t high = frame[2 + i * 2 + 1];
    t2_parts[i] = static_cast<uint64_t>((static_cast<uint16_t>(high) << 8) | low);
  }
  uint64_t t2_mcu = (t2_parts[3] << 48) | (t2_parts[2] << 32) |
                    (t2_parts[1] << 16) | t2_parts[0];

  // 解析t3（tx_timestamp，8字节小端序，索引10-17）
  uint64_t t3_mcu = static_cast<uint64_t>(to_int64_le(&frame[10]));

  // ===== 首次同步：建立时域映射 =====
  if (!ptp_initialized_) {
    g_offset_ = t1 - static_cast<int64_t>(t2_mcu);
    ptp_initialized_ = true;
    ptp_integral_ = 0.0;

    RCLCPP_INFO(this->get_logger(), "[PTP] 时域映射建立");
    RCLCPP_INFO(this->get_logger(), "[PTP]   g_offset = %ld us", g_offset_);
    RCLCPP_INFO(this->get_logger(), "[PTP]   MCU运行时间: %.3f 秒", t2_mcu / 1e6);

    last_t1_ = t1;
    last_t2_ = static_cast<int64_t>(t2_mcu);
    ptp_sync_count_++;
    return;
  }

  // ===== 将MCU时间转换到Linux时间域 =====
  int64_t t2_unix = static_cast<int64_t>(t2_mcu) + g_offset_;
  int64_t t3_unix = static_cast<int64_t>(t3_mcu) + g_offset_;

  // ===== 计算PTP offset和delay =====
  // offset: 时间偏移
  double offset = ((static_cast<double>(t2_unix) - t1) -
                   (static_cast<double>(t4) - t3_unix)) / 2.0;
  // delay: 往返延迟
  double delay = ((static_cast<double>(t4) - t1) +
                  (static_cast<double>(t3_unix) - t2_unix)) / 2.0;

  // 检测异常offset
  if (std::abs(offset) > 5000.0) {
    RCLCPP_WARN(this->get_logger(), "[PTP] #%d 异常偏差值: %+.1f us，跳过本次",
                ptp_sync_count_ + 1, offset);
    last_t1_ = t1;
    last_t2_ = static_cast<int64_t>(t2_mcu);
    ptp_sync_count_++;
    return;
  }

  // ===== PI控制器 =====
  double p_term = offset * PTP_KP;
  ptp_integral_ += offset;
  // 限幅
  ptp_integral_ = std::max(std::min(ptp_integral_, 10000.0), -10000.0);
  double i_term = ptp_integral_ * PTP_KI;

  double correction = p_term + i_term;
  // 限幅
  correction = std::max(std::min(correction, 500.0), -500.0);

  // 更新g_offset
  g_offset_ -= static_cast<int64_t>(correction);

  // 检测收敛状态
  if (!ptp_converged_ && std::abs(offset) < 500.0) {
    ptp_converged_ = true;
    RCLCPP_INFO(this->get_logger(), "[PTP] 已达到收敛状态");
  }

  // ===== 计算时间间隔 =====
  int64_t dt_mcu = static_cast<int64_t>(t3_mcu - t2_mcu);  // MCU处理时间 (t3 - t2)
  int64_t dt_linux = t4 - t1;                              // 整体往返时间 (t4 - t1)

  // 更新历史记录
  last_t1_ = t1;
  last_t2_ = static_cast<int64_t>(t2_mcu);
  ptp_sync_count_++;

  // 打印PTP同步信息
  RCLCPP_INFO(this->get_logger(),
              "[PTP] #%d offset=%+.1f us | delay=%.1f us | correction=%.1f us | "
              "g_offset=%ld us | Δt_MCU=%ld us | Δt_Linux=%ld us",
              ptp_sync_count_, offset, delay, correction, g_offset_, dt_mcu, dt_linux);
}

// ==================== 传感器数据处理 ====================

void RobotChassisNode::handleSensorFrame(const std::vector<uint8_t>& frame) {
  // 传感器帧：[0xFC][0x20][数据26B][时间戳8B][校验][0xDF]
  // 数据格式：13个int16_t（小端序）
  //   [0-3]: 编码器位置
  //   [4-6]: 欧拉角（pitch, roll, yaw），单位：0.01度
  //   [7-9]: 陀螺仪，单位：0.01弧度/s
  //   [10-12]: 加速度，单位：0.01g
  // 时间戳在索引28-35（13*2=26字节数据后）

  RawSensorData sensor_data;

  // 解析时间戳（8字节，小端序，索引28-35）
  sensor_data.timestamp_us = to_int64_le(&frame[28]);

  // 解析编码器数据（4个int16，小端序，索引2-9）
  for (int i = 0; i < 4; i++) {
    sensor_data.encoders[i] = to_int16_le(frame[2 + i*2], frame[2 + i*2 + 1]);
  }

  // 解析欧拉角（3个int16，小端序，索引10-15），单位：0.01度
  for (int i = 0; i < 3; i++) {
    sensor_data.euler[i] = to_int16_le(frame[10 + i*2], frame[10 + i*2 + 1]);
  }

  // 解析陀螺仪（3个int16，小端序，索引16-21），单位：0.01弧度/s
  for (int i = 0; i < 3; i++) {
    sensor_data.gyro[i] = to_int16_le(frame[16 + i*2], frame[16 + i*2 + 1]);
  }

  // 解析加速度（3个int16，小端序，索引22-27），单位：0.01g
  for (int i = 0; i < 3; i++) {
    sensor_data.accel[i] = to_int16_le(frame[22 + i*2], frame[22 + i*2 + 1]);
  }

  sensor_frame_count_++;

  // 每50帧打印一次传感器数据（100Hz → 2Hz日志频率）
  if (sensor_frame_count_ % 50 == 0) {
    RCLCPP_INFO(this->get_logger(), "[传感器 #%d]", sensor_frame_count_);
    RCLCPP_INFO(this->get_logger(), "  时间戳: %ld us", sensor_data.timestamp_us);
    RCLCPP_INFO(this->get_logger(), "  编码器: [%d, %d, %d, %d]",
                sensor_data.encoders[0], sensor_data.encoders[1],
                sensor_data.encoders[2], sensor_data.encoders[3]);
    RCLCPP_INFO(this->get_logger(), "  欧拉角(°): [%.2f, %.2f, %.2f]",
                sensor_data.euler[0] / 100.0, sensor_data.euler[1] / 100.0,
                sensor_data.euler[2] / 100.0);
    RCLCPP_INFO(this->get_logger(), "  陀螺仪(°/s): [%.2f, %.2f, %.2f]",
                sensor_data.gyro[0] / 100.0, sensor_data.gyro[1] / 100.0,
                sensor_data.gyro[2] / 100.0);
    RCLCPP_INFO(this->get_logger(), "  加速度(g): [%.2f, %.2f, %.2f]",
                sensor_data.accel[0] / 100.0, sensor_data.accel[1] / 100.0,
                sensor_data.accel[2] / 100.0);
  }
}
