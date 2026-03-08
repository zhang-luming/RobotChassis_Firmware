// Copyright 2026 RobotChassis Driver
// License: MIT

#ifndef ROBOT_CHASSIS_DRIVER__ROBOT_CHASSIS_NODE_HPP_
#define ROBOT_CHASSIS_DRIVER__ROBOT_CHASSIS_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>
#include <vector>
#include <mutex>
#include <thread>
#include <cstring>
#include <cstdint>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ==================== 协议常量 ====================

#define PROTOCOL_HEADER 0xFC
#define PROTOCOL_TAIL   0xDF

// MCU → PC 功能码
#define FUNC_PTP_SYNC       0x10  // PTP时间同步
#define FUNC_SENSOR_MERGED  0x20  // 传感器数据（编码器4 + 欧拉角3 + 陀螺仪3 + 加速度3）

// PC → MCU 功能码
#define FUNC_MOTOR_SPEED    0x31  // 电机目标速度（4个int16_t，CPS）

// 帧长度定义
#define SENSOR_FRAME_SIZE 38      // 传感器帧：1+1+26(13*int16)+8+1+1 = 38
#define PTP_FRAME_SIZE     20      // PTP响应帧：1+1+8+8+1+1 = 20
#define MOTOR_FRAME_SIZE   12      // 电机速度帧：1+1+8(4*int16)+1+1 = 12

// ==================== 数据结构 ====================

/**
 * @brief 原始传感器数据结构
 *
 * 数据来源：MCU 0x20功能码，13个int16_t
 * - 索引0-3: 编码器位置（4个int16）
 * - 索引4-6: 欧拉角（pitch, roll, yaw），单位：0.01度
 * - 索引7-9: 陀螺仪，单位：0.01弧度/s
 * - 索引10-12: 加速度，单位：0.01g
 */
struct RawSensorData {
  int64_t timestamp_us;     // MCU时间戳（微秒）
  int16_t encoders[4];      // 编码器位置
  int16_t euler[3];         // 欧拉角（俯仰、横滚、航向）
  int16_t gyro[3];          // 陀螺仪
  int16_t accel[3];         // 加速度
};

// ==================== 主节点类 ====================

/**
 * @brief RobotChassis ROS2驱动节点
 *
 * 功能：
 * 1. 通过串口与MCU通信
 * 2. PTP时间同步
 * 3. 解析传感器数据（编码器 + IMU）
 * 4. 订阅cmd_vel并控制电机速度
 */
class RobotChassisNode : public rclcpp::Node {
 public:
  RobotChassisNode();
  ~RobotChassisNode();

 private:
  // ========== 串口相关 ==========
  serial::Serial ser_;
  std::string port_;
  int baudrate_;
  std::thread read_thread_;
  std::vector<uint8_t> recv_buffer_;
  std::mutex buffer_mutex_;
  bool running_;

  bool initSerial();
  uint8_t checksum(const uint8_t* data, size_t len);

  // ========== 串口读取线程 ==========
  void serialReadThread();

  // ========== PTP时间同步相关 ==========
  int64_t g_offset_;             // MCU时间 → ROS时间的时域映射偏移（微秒）
  int ptp_sync_count_;           // 同步次数计数
  bool ptp_initialized_;         // 是否已建立时域映射
  int64_t last_t1_;              // 上次t1时间戳（ROS发送时刻）
  int64_t last_t2_;              // 上次t2时间戳（MCU接收时刻）
  int64_t ptp_t1_;               // 当前PTP请求的t1时间戳

  // PI控制器（硬编码，无需调参）
  static constexpr double PTP_KP = 0.5;   // 比例系数
  static constexpr double PTP_KI = 0.01;  // 积分系数
  double ptp_integral_;          // 积分累积
  bool ptp_converged_;           // 是否已收敛

  void handlePTPFrame(const std::vector<uint8_t>& frame, int64_t t4);
  void sendPTPRequest();

  // ========== 传感器数据处理相关 ==========
  int sensor_frame_count_;

  void handleSensorFrame(const std::vector<uint8_t>& frame);
  int16_t to_int16_le(uint8_t low, uint8_t high) const;
  int64_t to_int64_le(const uint8_t* data) const;

  // ========== 电机控制相关 ==========
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  std::mutex serial_mutex_;  // 串口发送互斥锁

  // 机械参数（从配置文件读取）
  double encoder_ppr_;       // 编码器每圈脉冲数
  double wheel_radius_;      // 轮半径（米）
  double wheelbase_;         // 轮距（米）

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void sendMotorSpeedCommand(const std::vector<int16_t>& speeds);
  std::vector<int16_t> twistToMotorSpeeds(double linear_x, double angular_z);

  // ========== 参数相关 ==========
  double ptp_interval_ms_;   // PTP同步间隔（毫秒）
  int ptp_seq_;              // PTP序列号
};

#endif  // ROBOT_CHASSIS_DRIVER__ROBOT_CHASSIS_NODE_HPP_
