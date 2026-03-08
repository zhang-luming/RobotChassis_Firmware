// Copyright 2026 RobotChassis Driver
// License: MIT

#include "robot_chassis_driver/robot_chassis_node.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <algorithm>
#include <vector>
#include <thread>

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
      last_mcu_timestamp_(0),
      encoders_initialized_(false),
      odom_x_(0.0),
      odom_y_(0.0),
      odom_theta_(0.0),
      last_path_x_(0.0),
      last_path_y_(0.0),
      last_path_theta_(0.0),
      ptp_seq_(0) {

  // 声明参数
  this->declare_parameter("serial_port", "/dev/ttyUSB0");
  this->declare_parameter("serial_baudrate", 921600);
  this->declare_parameter("ptp_interval_ms", 200.0);
  this->declare_parameter("encoder_ppr", 1000.0);
  this->declare_parameter("wheel_radius", 0.1);
  this->declare_parameter("wheelbase", 0.5);
  this->declare_parameter("wheel_odom_frame_id", "wheel_odom");
  this->declare_parameter("wheel_base_frame_id", "wheel_base_link");
  this->declare_parameter("imu_frame_id", "imu_link");
  this->declare_parameter("path_distance_threshold", 0.05);
  this->declare_parameter("path_angle_threshold", 0.1);
  this->declare_parameter("cmd_vel_topic", "/cmd_vel");
  this->declare_parameter("wheel_odom_topic", "/wheel_odom");
  this->declare_parameter("wheel_odom_path_topic", "/wheel_odom_path");
  this->declare_parameter("imu_topic", "/imu");
  this->declare_parameter("imu_orientation_covariance", std::vector<double>{0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01});
  this->declare_parameter("imu_angular_velocity_covariance", std::vector<double>{0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001});
  this->declare_parameter("imu_linear_acceleration_covariance", std::vector<double>{0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01});
  this->declare_parameter("odom_pose_covariance", std::vector<double>{
    0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.1
  });
  this->declare_parameter("odom_twist_covariance", std::vector<double>{
    0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.001, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.001, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.01
  });

  // 获取参数
  port_ = this->get_parameter("serial_port").as_string();
  baudrate_ = this->get_parameter("serial_baudrate").as_int();
  ptp_interval_ms_ = this->get_parameter("ptp_interval_ms").as_double();
  encoder_ppr_ = this->get_parameter("encoder_ppr").as_double();
  wheel_radius_ = this->get_parameter("wheel_radius").as_double();
  wheelbase_ = this->get_parameter("wheelbase").as_double();
  wheel_odom_frame_id_ = this->get_parameter("wheel_odom_frame_id").as_string();
  wheel_base_frame_id_ = this->get_parameter("wheel_base_frame_id").as_string();
  imu_frame_id_ = this->get_parameter("imu_frame_id").as_string();
  path_distance_threshold_ = this->get_parameter("path_distance_threshold").as_double();
  path_angle_threshold_ = this->get_parameter("path_angle_threshold").as_double();
  cmd_vel_topic_ = this->get_parameter("cmd_vel_topic").as_string();
  wheel_odom_topic_ = this->get_parameter("wheel_odom_topic").as_string();
  wheel_odom_path_topic_ = this->get_parameter("wheel_odom_path_topic").as_string();
  imu_topic_ = this->get_parameter("imu_topic").as_string();
  imu_orientation_covariance_ = this->get_parameter("imu_orientation_covariance").as_double_array();
  imu_angular_velocity_covariance_ = this->get_parameter("imu_angular_velocity_covariance").as_double_array();
  imu_linear_acceleration_covariance_ = this->get_parameter("imu_linear_acceleration_covariance").as_double_array();
  odom_pose_covariance_ = this->get_parameter("odom_pose_covariance").as_double_array();
  odom_twist_covariance_ = this->get_parameter("odom_twist_covariance").as_double_array();

  // 初始化串口
  if (!initSerial()) {
    RCLCPP_ERROR(this->get_logger(), "串口初始化失败");
    return;
  }

  // 创建cmd_vel订阅
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, 10,
      std::bind(&RobotChassisNode::cmdVelCallback, this, std::placeholders::_1));

  // 创建里程计发布者
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(wheel_odom_topic_, 10);
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>(wheel_odom_path_topic_, 10);
  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic_, 10);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  // 初始化编码器历史
  for (int i = 0; i < 4; i++) {
    last_encoders_[i] = 0;
  }

  // 初始化轨迹消息
  odom_path_.header.frame_id = wheel_odom_frame_id_;

  // 启动串口读取线程
  read_thread_ = std::thread(&RobotChassisNode::serialReadThread, this);

  // 启动消息发布线程
  publish_thread_ = std::thread(&RobotChassisNode::publishThread, this);

  RCLCPP_INFO(this->get_logger(), "RobotChassis节点已启动");
  RCLCPP_INFO(this->get_logger(), "串口: %s, 波特率: %d", port_.c_str(), baudrate_);
  RCLCPP_INFO(this->get_logger(), "PTP同步: 启用, 间隔: %.0f ms", ptp_interval_ms_);
  RCLCPP_INFO(this->get_logger(), "机械参数: 编码器%.0f PPR, 轮径%.3fm, 轮距%.3fm",
              encoder_ppr_, wheel_radius_, wheelbase_);
  RCLCPP_INFO(this->get_logger(), "话题: %s, %s, %s, %s",
              cmd_vel_topic_.c_str(), wheel_odom_topic_.c_str(),
              wheel_odom_path_topic_.c_str(), imu_topic_.c_str());
}

// ==================== 析构函数 ====================

RobotChassisNode::~RobotChassisNode() {
  running_ = false;
  if (read_thread_.joinable()) {
    read_thread_.join();
  }
  if (publish_thread_.joinable()) {
    publish_thread_.join();
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

    // RCLCPP_INFO(this->get_logger(), "[PTP] 发送请求 (t1=%ld us)", ptp_t1_);
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

    // RCLCPP_INFO(this->get_logger(), "[PTP] 时域映射建立");
    // RCLCPP_INFO(this->get_logger(), "[PTP]   g_offset = %ld us", g_offset_);
    // RCLCPP_INFO(this->get_logger(), "[PTP]   MCU运行时间: %.3f 秒", t2_mcu / 1e6);

    last_t1_ = t1;
    last_t2_ = static_cast<int64_t>(t2_mcu);
    ptp_sync_count_++;
    return;
  }

  // ===== 将MCU时间转换到Linux时间域 =====
  int64_t t2_unix = static_cast<int64_t>(t2_mcu) + g_offset_;
  int64_t t3_unix = static_cast<int64_t>(t3_mcu) + g_offset_;

  // ===== 计算PTP offset =====
  // offset: 时间偏移
  double offset = ((static_cast<double>(t2_unix) - t1) -
                   (static_cast<double>(t4) - t3_unix)) / 2.0;
  // delay: 往返延迟（未使用）
  // double delay = ((static_cast<double>(t4) - t1) +
  //                 (static_cast<double>(t3_unix) - t2_unix)) / 2.0;

  // 检测异常offset
  if (std::abs(offset) > 5000.0) {
    // RCLCPP_WARN(this->get_logger(), "[PTP] #%d 异常偏差值: %+.1f us，跳过本次",
    //             ptp_sync_count_ + 1, offset);
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
    // RCLCPP_INFO(this->get_logger(), "[PTP] 已达到收敛状态");
  }

  // ===== 计算时间间隔（未使用）=====
  // int64_t dt_mcu = static_cast<int64_t>(t3_mcu - t2_mcu);  // MCU处理时间 (t3 - t2)
  // int64_t dt_linux = t4 - t1;                              // 整体往返时间 (t4 - t1)

  // 更新历史记录
  last_t1_ = t1;
  last_t2_ = static_cast<int64_t>(t2_mcu);
  ptp_sync_count_++;

  // 打印PTP同步信息
  // RCLCPP_INFO(this->get_logger(),
  //             "[PTP] #%d offset=%+.1f us | delay=%.1f us | correction=%.1f us | "
  //             "g_offset=%ld us | Δt_MCU=%ld us | Δt_Linux=%ld us",
  //             ptp_sync_count_, offset, delay, correction, g_offset_, dt_mcu, dt_linux);
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
  for (int i = 0; i < 6; i++) {
    sensor_data.accel[i] = to_int16_le(frame[22 + i*2], frame[22 + i*2 + 1]);
  }

  sensor_frame_count_++;

  // 推入队列，供发布线程消费
  sensor_queue_.push(sensor_data);
}

// ==================== 电机控制 ====================

void RobotChassisNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  // 打印原始控制指令
  RCLCPP_INFO(this->get_logger(), "[控制指令] linear.x=%.3f m/s, angular.z=%.3f rad/s",
              msg->linear.x, msg->angular.z);

  // 将Twist消息转换为4个电机速度
  std::vector<int16_t> motor_speeds = twistToMotorSpeeds(msg->linear.x, msg->angular.z);

  // 打印计算出的电机速度
  RCLCPP_INFO(this->get_logger(), "[电机速度] A:%d, B:%d, C:%d, D:%d CPS",
              motor_speeds[0], motor_speeds[1], motor_speeds[2], motor_speeds[3]);

  // 发送速度控制指令
  sendMotorSpeedCommand(motor_speeds);
}

std::vector<int16_t> RobotChassisNode::twistToMotorSpeeds(double linear_x, double angular_z) {
  // 差速驱动运动学解算
  // 四轮底盘：左侧轮A、C，右侧轮B、D

  // 计算左右轮线速度
  double v_left = linear_x - (angular_z * wheelbase_ / 2.0);
  double v_right = linear_x + (angular_z * wheelbase_ / 2.0);

  // 转换为编码器速度
  // ticks_per_second = velocity_m_s * encoder_ppr / (2 * π * wheel_radius)
  double conversion_factor = encoder_ppr_ / (2.0 * M_PI * wheel_radius_);

  double left_cps = v_left * conversion_factor;
  double right_cps = v_right * conversion_factor;

  // 限制速度范围（int16_t: -32768 ~ 32767）
  left_cps = std::max(std::min(left_cps, 30000.0), -30000.0);
  right_cps = std::max(std::min(right_cps, 30000.0), -30000.0);

  // 返回4个电机速度 [A, B, C, D]
  // 只有A、B电机连接，C、D置零
  std::vector<int16_t> speeds(4);
  speeds[0] = static_cast<int16_t>(right_cps);  // 电机A（右）
  speeds[1] = static_cast<int16_t>(left_cps);   // 电机B（左）
  speeds[2] = 0;                                 // 电机C（未连接）
  speeds[3] = 0;                                 // 电机D（未连接）

  return speeds;
}

void RobotChassisNode::sendMotorSpeedCommand(const std::vector<int16_t>& speeds) {
  if (speeds.size() != 4) {
    RCLCPP_ERROR(this->get_logger(), "电机速度数组长度错误: %zu", speeds.size());
    return;
  }

  // 构建电机速度控制帧
  // 帧格式：[0xFC][0x31][速度A_L][速度A_H]...[速度D_L][速度D_H][checksum][0xDF]
  std::vector<uint8_t> frame;
  frame.reserve(MOTOR_FRAME_SIZE);

  frame.push_back(PROTOCOL_HEADER);
  frame.push_back(FUNC_MOTOR_SPEED);

  // 4个int16_t速度值（小端序）
  for (int i = 0; i < 4; i++) {
    frame.push_back(static_cast<uint8_t>(speeds[i] & 0xFF));        // 低字节
    frame.push_back(static_cast<uint8_t>((speeds[i] >> 8) & 0xFF)); // 高字节
  }

  // 计算校验和
  uint8_t cs = checksum(frame.data(), frame.size());
  frame.push_back(cs);
  frame.push_back(PROTOCOL_TAIL);

  // 发送（加锁保护）
  try {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    ser_.write(frame.data(), frame.size());
    ser_.flush();
  } catch (serial::SerialException& e) {
    RCLCPP_ERROR(this->get_logger(), "发送电机指令失败: %s", e.what());
  }
}

// ==================== 消息发布线程 ====================

void RobotChassisNode::publishThread() {
  RCLCPP_INFO(this->get_logger(), "消息发布线程已启动");

  RawSensorData sensor_data;

  while (running_ && rclcpp::ok()) {
    // 从队列中取出传感器数据
    if (sensor_queue_.pop(sensor_data)) {
      // 发布里程计数据
      publishOdometry(sensor_data);

      // 发布IMU数据
      publishIMU(sensor_data);
    } else {
      // 队列为空，休眠1ms
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  RCLCPP_INFO(this->get_logger(), "消息发布线程已退出");
}

// ==================== 里程计发布 ====================

void RobotChassisNode::publishOdometry(const RawSensorData& sensor_data) {
  // 第一次数据：初始化编码器基准值
  if (!encoders_initialized_) {
    for (int i = 0; i < 4; i++) {
      last_encoders_[i] = sensor_data.encoders[i];
    }
    last_mcu_timestamp_ = sensor_data.timestamp_us;
    encoders_initialized_ = true;
    return;  // 跳过第一次数据
  }

  // 计算时间差（秒）
  int64_t delta_time_us = sensor_data.timestamp_us - last_mcu_timestamp_;
  if (delta_time_us <= 0) {
    // 时间戳异常，跳过本次更新
    return;
  }
  double dt = delta_time_us / 1000000.0;  // 微秒转秒
  last_mcu_timestamp_ = sensor_data.timestamp_us;

  // 计算编码器增量（处理16位溢出）
  int32_t delta_encoders[4];
  for (int i = 0; i < 4; i++) {
    int16_t delta = sensor_data.encoders[i] - last_encoders_[i];
    // 处理溢出（int16_t范围：-32768~32767）
    if (delta < -32767) {
      delta += 65536;
    }
    delta_encoders[i] = delta;
    last_encoders_[i] = sensor_data.encoders[i];
  }

  // A、B电机连接，A是右轮，B是左轮
  // 注意：B电机编码器方向相反，需要取反
  double right_counts = static_cast<double>(delta_encoders[0]);   // 电机A（右轮）
  double left_counts = -static_cast<double>(delta_encoders[1]);  // 电机B（左轮，取反）

  // 计算左右轮线位移增量
  // 距离增量 = counts / PPR × 轮周长
  double wheel_circumference = 2.0 * M_PI * wheel_radius_;
  double right_delta = right_counts / encoder_ppr_ * wheel_circumference;
  double left_delta = left_counts / encoder_ppr_ * wheel_circumference;

  // 差速驱动里程计计算
  double delta_distance = (left_delta + right_delta) / 2.0;
  double delta_theta = (right_delta - left_delta) / wheelbase_;

  // 更新位姿
  odom_x_ += delta_distance * std::cos(odom_theta_);
  odom_y_ += delta_distance * std::sin(odom_theta_);
  odom_theta_ += delta_theta;

  // 归一化角度到[-π, π]
  while (odom_theta_ > M_PI) odom_theta_ -= 2.0 * M_PI;
  while (odom_theta_ < -M_PI) odom_theta_ += 2.0 * M_PI;

  // 将MCU时间戳转换为ROS时间
  rclcpp::Time ros_time = rclcpp::Time(
      (sensor_data.timestamp_us + g_offset_) * 1000);  // 微秒转纳秒

  // 构造里程计消息
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = ros_time;
  odom_msg.header.frame_id = wheel_odom_frame_id_;
  odom_msg.child_frame_id = wheel_base_frame_id_;

  // 位置
  odom_msg.pose.pose.position.x = odom_x_;
  odom_msg.pose.pose.position.y = odom_y_;
  odom_msg.pose.pose.position.z = 0.0;

  // 姿态（从yaw角构造四元数）
  tf2::Quaternion q;
  q.setRPY(0, 0, odom_theta_);
  odom_msg.pose.pose.orientation.x = q.x();
  odom_msg.pose.pose.orientation.y = q.y();
  odom_msg.pose.pose.orientation.z = q.z();
  odom_msg.pose.pose.orientation.w = q.w();

  // 速度（使用实际时间差计算）
  odom_msg.twist.twist.linear.x = delta_distance / dt;
  odom_msg.twist.twist.linear.y = 0.0;
  odom_msg.twist.twist.angular.z = delta_theta / dt;

  // 设置协方差
  for (int i = 0; i < 36; i++) {
    odom_msg.pose.covariance[i] = odom_pose_covariance_[i];
    odom_msg.twist.covariance[i] = odom_twist_covariance_[i];
  }

  // 发布里程计消息
  odom_pub_->publish(odom_msg);

  // 发布TF变换（wheel_odom → wheel_base_link）
  geometry_msgs::msg::TransformStamped odom_tf;
  odom_tf.header.stamp = ros_time;
  odom_tf.header.frame_id = wheel_odom_frame_id_;
  odom_tf.child_frame_id = wheel_base_frame_id_;

  odom_tf.transform.translation.x = odom_x_;
  odom_tf.transform.translation.y = odom_y_;
  odom_tf.transform.translation.z = 0.0;

  odom_tf.transform.rotation.x = q.x();
  odom_tf.transform.rotation.y = q.y();
  odom_tf.transform.rotation.z = q.z();
  odom_tf.transform.rotation.w = q.w();

  tf_broadcaster_->sendTransform(odom_tf);

  // 发布轨迹（根据距离和角度阈值）
  double dx = odom_x_ - last_path_x_;
  double dy = odom_y_ - last_path_y_;
  double distance = std::sqrt(dx * dx + dy * dy);

  // 归一化角度差到[-π, π]
  double dtheta = odom_theta_ - last_path_theta_;
  while (dtheta > M_PI) dtheta -= 2.0 * M_PI;
  while (dtheta < -M_PI) dtheta += 2.0 * M_PI;
  double angle_diff = std::abs(dtheta);

  // 判断是否需要添加新的轨迹点
  if (distance > path_distance_threshold_ || angle_diff > path_angle_threshold_) {
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros_time;
    pose_stamped.header.frame_id = wheel_odom_frame_id_;
    pose_stamped.pose.position.x = odom_x_;
    pose_stamped.pose.position.y = odom_y_;
    pose_stamped.pose.position.z = 0.0;
    pose_stamped.pose.orientation = odom_msg.pose.pose.orientation;

    odom_path_.poses.push_back(pose_stamped);
    odom_path_.header.stamp = ros_time;

    // 发布轨迹
    path_pub_->publish(odom_path_);

    // 更新上次轨迹点
    last_path_x_ = odom_x_;
    last_path_y_ = odom_y_;
    last_path_theta_ = odom_theta_;
  }
}

void RobotChassisNode::publishIMU(const RawSensorData& sensor_data) {
  // 将MCU时间戳转换为ROS时间
  rclcpp::Time ros_time = rclcpp::Time(
      (sensor_data.timestamp_us + g_offset_) * 1000);  // 微秒转纳秒

  // 构造IMU消息
  sensor_msgs::msg::Imu imu_msg;
  imu_msg.header.stamp = ros_time;
  imu_msg.header.frame_id = imu_frame_id_;

  // 欧拉角转四元数（roll, pitch, yaw）
  // 注意：MCU上报的是[pitch, roll, yaw]，单位0.01度
  double roll = sensor_data.euler[1] / 100.0 * M_PI / 180.0;   // 弧度
  double pitch = sensor_data.euler[0] / 100.0 * M_PI / 180.0;  // 弧度
  double yaw = sensor_data.euler[2] / 100.0 * M_PI / 180.0;    // 弧度

  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  imu_msg.orientation.x = q.x();
  imu_msg.orientation.y = q.y();
  imu_msg.orientation.z = q.z();
  imu_msg.orientation.w = q.w();

  // 陀螺仪数据（单位：0.01弧度/s → 弧度/s）
  imu_msg.angular_velocity.x = sensor_data.gyro[0] / 100.0;
  imu_msg.angular_velocity.y = sensor_data.gyro[1] / 100.0;
  imu_msg.angular_velocity.z = sensor_data.gyro[2] / 100.0;

  // 加速度数据（单位：0.01g → m/s²）
  // g = 9.80665 m/s²
  constexpr double G_TO_ACCEL = 9.80665;
  imu_msg.linear_acceleration.x = sensor_data.accel[0] / 100.0 * G_TO_ACCEL;
  imu_msg.linear_acceleration.y = sensor_data.accel[1] / 100.0 * G_TO_ACCEL;
  imu_msg.linear_acceleration.z = sensor_data.accel[2] / 100.0 * G_TO_ACCEL;

  // 设置协方差（从配置文件读取）
  for (int i = 0; i < 9; i++) {
    imu_msg.orientation_covariance[i] = imu_orientation_covariance_[i];
    imu_msg.angular_velocity_covariance[i] = imu_angular_velocity_covariance_[i];
    imu_msg.linear_acceleration_covariance[i] = imu_linear_acceleration_covariance_[i];
  }

  // 发布IMU消息
  imu_pub_->publish(imu_msg);
}
