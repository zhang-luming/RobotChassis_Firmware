#ifndef ROBOT_CHASSIS_DRIVER__ROBOT_CHASSIS_NODE_HPP_
#define ROBOT_CHASSIS_DRIVER__ROBOT_CHASSIS_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <chrono>
#include <vector>
#include <mutex>
#include <thread>
#include <queue>
#include <condition_variable>
#include <cstring>
#include <cstdint>
#include <cmath>
#include <cstddef>

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

// ==================== 无锁环形队列 ====================

/**
 * @brief 无锁环形队列（单生产者单消费者）
 */
template<typename T, size_t Size>
class LockFreeRingBuffer {
 public:
  LockFreeRingBuffer() : write_pos_(0), read_pos_(0) {}

  /**
   * @brief 写入数据（生产者）
   * @param item 数据项
   * @return true=成功，false=队列满（丢弃最旧数据）
   */
  bool push(const T& item) {
    size_t next_write = (write_pos_ + 1) % Size;

    // 检查队列是否满
    if (next_write == read_pos_) {
      // 队列满，移动读指针丢弃最旧数据
      read_pos_ = (read_pos_ + 1) % Size;
    }

    buffer_[write_pos_] = item;
    write_pos_ = next_write;
    return true;
  }

  /**
   * @brief 读取数据（消费者）
   * @param item 输出数据项
   * @return true=成功，false=队列空
   */
  bool pop(T& item) {
    if (read_pos_ == write_pos_) {
      return false;  // 队列空
    }

    item = buffer_[read_pos_];
    read_pos_ = (read_pos_ + 1) % Size;
    return true;
  }

  /**
   * @brief 检查队列是否为空
   */
  bool empty() const {
    return read_pos_ == write_pos_;
  }

  /**
   * @brief 获取当前队列长度
   */
  size_t size() const {
    if (write_pos_ >= read_pos_) {
      return write_pos_ - read_pos_;
    } else {
      return Size - read_pos_ + write_pos_;
    }
  }

 private:
  T buffer_[Size];
  size_t write_pos_;  // 写指针
  size_t read_pos_;   // 读指针
};

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
 * 3. 解析传感器数据并推入队列（生产者）
 * 4. 从队列消费数据并发布ROS消息（消费者）：odom、imu等
 * 5. 订阅cmd_vel并控制电机速度
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
  LockFreeRingBuffer<RawSensorData, 10> sensor_queue_;  // 传感器数据队列（10帧）
  int sensor_frame_count_;

  void handleSensorFrame(const std::vector<uint8_t>& frame);
  int16_t to_int16_le(uint8_t low, uint8_t high) const;
  int64_t to_int64_le(const uint8_t* data) const;

  // ========== 消息发布相关 ==========
  std::thread publish_thread_;    // 消息发布线程
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // 里程计状态
  int16_t last_encoders_[4];     // 上次编码器位置
  int64_t last_mcu_timestamp_;   // 上次MCU时间戳（微秒）
  bool encoders_initialized_;     // 编码器是否已初始化
  double odom_x_, odom_y_, odom_theta_;  // 里程计位姿

  // 轨迹发布
  nav_msgs::msg::Path odom_path_;  // 里程计轨迹
  double last_path_x_, last_path_y_, last_path_theta_;  // 上次轨迹点位置

  // ========== EKF里程计轨迹发布 ==========
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ekf_odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr ekf_trajectory_pub_;
  nav_msgs::msg::Path ekf_trajectory_;  // EKF融合后的轨迹
  double last_ekf_x_, last_ekf_y_, last_ekf_theta_;  // 上次EKF轨迹点位置

  void publishThread();          // 消息发布线程函数
  void publishOdometry(const RawSensorData& sensor_data);
  void publishIMU(const RawSensorData& sensor_data);  // 预留接口
  void ekfOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);  // EKF里程计回调

  // ========== 电机控制相关 ==========
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  std::mutex serial_mutex_;  // 串口发送互斥锁

  // 话题名（从配置文件读取）
  std::string cmd_vel_topic_;
  std::string wheel_odom_topic_;
  std::string imu_topic_;

  // 机械参数（从配置文件读取）
  double encoder_ppr_;       // 编码器每圈脉冲数
  double wheel_radius_;      // 轮半径（米）
  double wheelbase_;         // 轮距（米）

  // 轮速计校准系数（从配置文件读取）
  double odometry_linear_scale_;   // 线速度校准系数
  double odometry_angular_scale_;  // 角速度校准系数

  // 坐标系参数（从配置文件读取）
  std::string wheel_odom_frame_id_;
  std::string wheel_base_frame_id_;
  std::string imu_frame_id_;

  // 轨迹发布参数（从配置文件读取）
  bool publish_trajectory_;               // 是否发布轨迹（统一开关）
  double trajectory_distance_threshold_;  // 轨迹距离阈值（统一参数）
  double trajectory_angle_threshold_;     // 轨迹角度阈值（统一参数）

  // 轨迹话题名（从配置文件读取）
  std::string wheel_odom_path_topic_;     // 轮速里程计轨迹话题
  std::string ekf_trajectory_topic_;      // EKF融合轨迹话题

  // 协方差参数（从配置文件读取）
  std::vector<double> imu_orientation_covariance_;
  std::vector<double> imu_angular_velocity_covariance_;
  std::vector<double> imu_linear_acceleration_covariance_;
  std::vector<double> odom_pose_covariance_;
  std::vector<double> odom_twist_covariance_;

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void sendMotorSpeedCommand(const std::vector<int16_t>& speeds);
  std::vector<int16_t> twistToMotorSpeeds(double linear_x, double angular_z);

  // ========== 参数相关 ==========
  double ptp_interval_ms_;   // PTP同步间隔（毫秒）
  int ptp_seq_;              // PTP序列号

  // ========== IMU零偏校准相关 ==========
  struct ChannelStats {
    double mean = 0.0;       // 均值（零偏）
    double m2 = 0.0;         // 平方差累计（用于计算方差）
    int count = 0;           // 样本计数
    double deadzone = 0.0;   // 死区（标准差 × 倍数）
    bool calibrated = false; // 是否已校准
  };

  ChannelStats gyro_stats_[3];      // 陀螺仪三轴统计 [x, y, z]
  ChannelStats accel_stats_[3];     // 加速度三轴统计 [x, y, z]

  int calibration_frames_;          // 校准需要的帧数
  double deadzone_sigma_;           // 死区标准差倍数
  bool imu_calibrated_;             // IMU是否已完成校准

  void updateIMUCalibration(const RawSensorData& sensor_data);
  double applyCalibration(double value, ChannelStats& stats);

  // ========== 轮速约束IMU零偏相关 ==========
  struct VelocitySample {
    double linear_x;        // 线速度 m/s
    double angular_z;       // 角速度 rad/s
    int64_t timestamp_us;   // 时间戳（微秒）
  };

  std::vector<VelocitySample> velocity_history_;  // 轮速历史记录
  double velocity_threshold_;         // 轮速阈值（m/s）
  double static_time_window_;         // 静止判断时间窗口（秒）
  int64_t static_start_time_us_;      // 开始静止的时刻（微秒）
  bool is_static_;                    // 当前是否静止
  bool was_static_;                   // 上一次是否静止（用于检测状态变化）

  void updateVelocityHistory(double linear_x, double angular_z, int64_t timestamp_us);
  bool checkStaticState(int64_t current_timestamp_us);
};

#endif  // ROBOT_CHASSIS_DRIVER__ROBOT_CHASSIS_NODE_HPP_
