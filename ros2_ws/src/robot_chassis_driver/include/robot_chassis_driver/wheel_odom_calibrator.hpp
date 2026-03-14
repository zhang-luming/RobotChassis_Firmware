// 轮速里程计校准工具 - 以IMU为基准进行自动标定

#ifndef ROBOT_CHASSIS_DRIVER__WHEEL_ODOM_CALIBRATOR_HPP_
#define ROBOT_CHASSIS_DRIVER__WHEEL_ODOM_CALIBRATOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <deque>
#include <vector>
#include <fstream>
#include <cmath>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <numeric>

/**
 * @brief 轮速里程计校准工具节点
 *
 * 功能：
 * 1. 自动检测机器人旋转运动（原地旋转）
 * 2. 以IMU为基准计算角速度校准系数 (odometry_angular_scale)
 * 3. 输出统计结果和95%置信区间
 *
 * 使用方法：
 * 1. 启动机器人和校准工具
 * 2. 手动控制机器人原地旋转（建议每次旋转90度以上）
 * 3. 工具自动检测运动开始/停止并记录数据
 * 4. 完成指定次数后输出最终结果
 *
 * 注意：
 * - 仅进行原地旋转，避免直行运动
 * - 旋转速度应保持均匀
 * - 每次旋转后确保机器人完全停止
 */
class WheelOdomCalibrator : public rclcpp::Node {
public:
  WheelOdomCalibrator();
  ~WheelOdomCalibrator();

private:
  // 状态枚举
  enum State {
    IDLE,           // 空闲状态，等待运动
    CALIBRATING,    // 标定中，累积角度变化
  };

  // IMU数据结构
  struct ImuData {
    rclcpp::Time timestamp;
    double yaw;              // 偏航角（弧度）

    // 用于排序和查找
    bool operator<(const rclcpp::Time& time) const {
      return timestamp < time;
    }
  };

  // 角速度标定结果结构
  struct AngularCalibrationResult {
    double wheel_angle;       // 轮速里程计累积角度（度）
    double imu_angle;         // IMU累积角度（度）
    double scale_factor;      // 校准系数比值（IMU/轮速计）
    rclcpp::Time timestamp;   // 时间戳
  };

  // 回调函数
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void wheelOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  // 工具函数
  double getYawFromQuaternion(const geometry_msgs::msg::Quaternion& quat);
  bool findClosestImuData(const rclcpp::Time& target_time,
                          double& imu_yaw,
                          double& time_diff);
  void saveCalibrationResult(double wheel_angle, double imu_angle);
  void printCalibrationSummary();
  void saveFinalResults();

  // ROS2订阅者
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_sub_;

  // IMU数据缓存（用于时间戳对齐）
  std::deque<ImuData> imu_buffer_;
  size_t max_imu_buffer_size_;

  // 角速度标定结果存储
  std::vector<AngularCalibrationResult> calibration_results_;

  // 状态机
  State state_;
  int stop_count_;

  // 累积角度（本次标定周期内）
  double accumulated_wheel_yaw_;
  double accumulated_imu_yaw_;

  // 上一次的值（用于计算增量）
  double prev_wheel_yaw_;
  double prev_imu_yaw_;

  // 初始化标志
  bool initialized_;

  // 参数
  double min_rotation_angle_;           // 最小旋转角度（度）
  double motion_velocity_threshold_;    // 运动检测角速度阈值（弧度/秒）
  double stop_velocity_threshold_;      // 静止检测角速度阈值（弧度/秒）
  int stop_count_threshold_;            // 静止检测连续次数
  int max_calibrations_;                // 最大标定次数

  // 日志文件
  std::ofstream log_file_;
  std::string log_file_path_;
  bool log_file_opened_;

  // 轮距参数（用于判断是否为纯旋转）
  double wheelbase_;
};

#endif  // ROBOT_CHASSIS_DRIVER__WHEEL_ODOM_CALIBRATOR_HPP_
