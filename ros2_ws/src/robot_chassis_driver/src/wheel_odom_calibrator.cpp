// Copyright 2026 RobotChassis Project
// Licensed under MIT License
// 轮速里程计校准工具 - 以IMU为基准进行自动标定

#include "robot_chassis_driver/wheel_odom_calibrator.hpp"
#include <ctime>

using namespace std::chrono_literals;

WheelOdomCalibrator::WheelOdomCalibrator()
    : Node("wheel_odom_calibrator"),
      state_(IDLE),
      stop_count_(0),
      log_file_opened_(false),
      initialized_(false),
      prev_wheel_yaw_(0.0),
      prev_imu_yaw_(0.0) {

  // 声明并获取参数
  this->declare_parameter("min_rotation_angle", 90.0);
  this->declare_parameter("motion_velocity_threshold", 0.3);
  this->declare_parameter("stop_velocity_threshold", 0.1);
  this->declare_parameter("stop_count_threshold", 100);
  this->declare_parameter("max_calibrations", 10);
  this->declare_parameter("imu_buffer_size", 500);
  this->declare_parameter("wheelbase", 0.148);

  this->get_parameter("min_rotation_angle", min_rotation_angle_);
  this->get_parameter("motion_velocity_threshold", motion_velocity_threshold_);
  this->get_parameter("stop_velocity_threshold", stop_velocity_threshold_);
  this->get_parameter("stop_count_threshold", stop_count_threshold_);
  this->get_parameter("max_calibrations", max_calibrations_);
  this->get_parameter("imu_buffer_size", max_imu_buffer_size_);
  this->get_parameter("wheelbase", wheelbase_);

  // 创建订阅者
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu", 10,
      std::bind(&WheelOdomCalibrator::imuCallback, this, std::placeholders::_1));

  wheel_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/wheel_odom", 10,
      std::bind(&WheelOdomCalibrator::wheelOdomCallback, this, std::placeholders::_1));

  // 生成日志文件名
  auto now = std::chrono::system_clock::now();
  auto time_t_now = std::chrono::system_clock::to_time_t(now);
  std::stringstream ss;
  ss << "wheel_odom_angular_calibration_" << std::put_time(std::localtime(&time_t_now), "%Y%m%d_%H%M%S") << ".log";
  log_file_path_ = ss.str();

  // 打开日志文件
  log_file_.open(log_file_path_, std::ios::out);
  if (log_file_.is_open()) {
    log_file_opened_ = true;
    log_file_ << "========================================" << std::endl;
    log_file_ << "     轮速里程计角速度校准工具" << std::endl;
    log_file_ << "========================================" << std::endl;
    log_file_ << "开始时间: " << std::put_time(std::localtime(&time_t_now), "%Y-%m-%d %H:%M:%S") << std::endl;
    log_file_ << "最小旋转角度: " << min_rotation_angle_ << " 度" << std::endl;
    log_file_ << "运动检测角速度阈值: " << motion_velocity_threshold_ << " rad/s" << std::endl;
    log_file_ << "静止检测角速度阈值: " << stop_velocity_threshold_ << " rad/s" << std::endl;
    log_file_ << "静止检测连续次数: " << stop_count_threshold_ << std::endl;
    log_file_ << "最大标定次数: " << max_calibrations_ << std::endl;
    log_file_ << "轮距: " << wheelbase_ << " m" << std::endl;
    log_file_ << "========================================" << std::endl;
    log_file_.flush();
  }

  RCLCPP_INFO(this->get_logger(), "========================================");
  RCLCPP_INFO(this->get_logger(), "     轮速里程计角速度校准工具");
  RCLCPP_INFO(this->get_logger(), "========================================");
  RCLCPP_INFO(this->get_logger(), "最小旋转角度: %.1f 度", min_rotation_angle_);
  RCLCPP_INFO(this->get_logger(), "运动检测角速度阈值: %.2f rad/s", motion_velocity_threshold_);
  RCLCPP_INFO(this->get_logger(), "静止检测角速度阈值: %.2f rad/s", stop_velocity_threshold_);
  RCLCPP_INFO(this->get_logger(), "静止检测连续次数: %d", stop_count_threshold_);
  RCLCPP_INFO(this->get_logger(), "最大标定次数: %d", max_calibrations_);
  RCLCPP_INFO(this->get_logger(), "轮距: %.3f m", wheelbase_);
  RCLCPP_INFO(this->get_logger(), "日志文件: %s", log_file_path_.c_str());
  RCLCPP_INFO(this->get_logger(), "========================================");
  RCLCPP_INFO(this->get_logger(), "使用说明：");
  RCLCPP_INFO(this->get_logger(), "  1. 请手动控制机器人原地旋转至少 %.1f 度", min_rotation_angle_);
  RCLCPP_INFO(this->get_logger(), "  2. 工具将自动检测运动开始/停止并记录数据");
  RCLCPP_INFO(this->get_logger(), "  3. 完成 %d 次有效标定后自动输出结果", max_calibrations_);
  RCLCPP_INFO(this->get_logger(), "  4. 原地旋转，避免直行运动");
  RCLCPP_INFO(this->get_logger(), "========================================");
}

WheelOdomCalibrator::~WheelOdomCalibrator() {
  // 输出最终结果
  if (!calibration_results_.empty()) {
    printCalibrationSummary();
    saveFinalResults();
  }

  // 关闭日志文件
  if (log_file_.is_open()) {
    log_file_ << "========================================" << std::endl;
    log_file_ << "标定结束" << std::endl;
    log_file_ << "========================================" << std::endl;
    log_file_.close();
  }
}

void WheelOdomCalibrator::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  // 提取偏航角
  double yaw = getYawFromQuaternion(msg->orientation);

  // 添加到缓存
  ImuData imu_data;
  imu_data.timestamp = msg->header.stamp;
  imu_data.yaw = yaw;

  imu_buffer_.push_back(imu_data);

  // 限制缓存大小
  while (imu_buffer_.size() > max_imu_buffer_size_) {
    imu_buffer_.pop_front();
  }
}

void WheelOdomCalibrator::wheelOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  // 提取轮速里程计偏航角和角速度
  double wheel_yaw = getYawFromQuaternion(msg->pose.pose.orientation);
  double angular_velocity = msg->twist.twist.angular.z;
  double linear_velocity = std::abs(msg->twist.twist.linear.x);

  // 时间戳对齐: 找到对应的IMU数据
  double current_imu_yaw;
  double imu_time_diff;
  if (!findClosestImuData(msg->header.stamp, current_imu_yaw, imu_time_diff)) {
    return;  // 没有找到对应的IMU数据
  }

  // 初始化
  if (!initialized_) {
    prev_wheel_yaw_ = wheel_yaw;
    prev_imu_yaw_ = current_imu_yaw;
    initialized_ = true;
    RCLCPP_INFO(this->get_logger(), "系统初始化完成，等待旋转运动...");
    return;
  }

  // 计算相邻帧的角度变化（用于累积）
  double wheel_delta = wheel_yaw - prev_wheel_yaw_;
  double imu_delta = current_imu_yaw - prev_imu_yaw_;

  // 处理角度跳变（相邻帧之间的跳变，不会超过180度）
  if (wheel_delta > M_PI) wheel_delta -= 2 * M_PI;
  if (wheel_delta < -M_PI) wheel_delta += 2 * M_PI;
  if (imu_delta > M_PI) imu_delta -= 2 * M_PI;
  if (imu_delta < -M_PI) imu_delta += 2 * M_PI;

  // 状态机处理
  switch (state_) {
    case IDLE: {
      // 检测旋转运动开始（角速度超过阈值，线速度较低）
      bool is_rotating = std::abs(angular_velocity) > motion_velocity_threshold_;
      bool is_pure_rotation = linear_velocity < 0.05;  // 线速度小于0.05m/s认为是纯旋转

      if (is_rotating && is_pure_rotation) {
        state_ = CALIBRATING;
        stop_count_ = 0;
        accumulated_wheel_yaw_ = 0.0;
        accumulated_imu_yaw_ = 0.0;

        RCLCPP_INFO(this->get_logger(), "【标定开始】轮速里程计: %.2f°, IMU: %.2f°",
                    wheel_yaw * 180.0 / M_PI, current_imu_yaw * 180.0 / M_PI);
      }
      break;
    }

    case CALIBRATING: {
      // 累积角度变化
      accumulated_wheel_yaw_ += wheel_delta;
      accumulated_imu_yaw_ += imu_delta;

      // 检测运动停止（连续N次角速度低于阈值）
      if (std::abs(angular_velocity) < stop_velocity_threshold_) {
        stop_count_++;

        // 达到静止判定阈值
        if (stop_count_ >= stop_count_threshold_) {
          double wheel_angle_deg = std::abs(accumulated_wheel_yaw_) * 180.0 / M_PI;
          double imu_angle_deg = std::abs(accumulated_imu_yaw_) * 180.0 / M_PI;

          // 检查是否满足最小旋转角度
          if (wheel_angle_deg >= min_rotation_angle_) {
            // 保存标定结果
            saveCalibrationResult(accumulated_wheel_yaw_, accumulated_imu_yaw_);

            double scale_factor = imu_angle_deg / wheel_angle_deg;
            RCLCPP_INFO(this->get_logger(), "【标定完成 #%zu】轮速里程计: %.2f°, IMU: %.2f°, 校准系数: %.4f",
                        calibration_results_.size(), wheel_angle_deg, imu_angle_deg, scale_factor);
          } else {
            RCLCPP_WARN(this->get_logger(), "【角度不足】轮速里程计: %.2f° < %.1f°，忽略本次测量",
                        wheel_angle_deg, min_rotation_angle_);
          }

          // 回到空闲状态
          state_ = IDLE;
          stop_count_ = 0;

          size_t current_count = calibration_results_.size();
          RCLCPP_INFO(this->get_logger(), "【等待下一次旋转】已完成 %zu/%d 次标定",
                      current_count, max_calibrations_);

          // 检查是否达到最大标定次数
          if (current_count >= static_cast<size_t>(max_calibrations_)) {
            RCLCPP_INFO(this->get_logger(), "========================================");
            RCLCPP_INFO(this->get_logger(), "已完成 %d 次标定，输出最终结果", max_calibrations_);
            RCLCPP_INFO(this->get_logger(), "========================================");
            printCalibrationSummary();
            saveFinalResults();
          }
        }
      } else {
        // 运动中，重置计数器
        stop_count_ = 0;
      }
      break;
    }
  }

  // 更新上一次的值
  prev_wheel_yaw_ = wheel_yaw;
  prev_imu_yaw_ = current_imu_yaw;
}

double WheelOdomCalibrator::getYawFromQuaternion(const geometry_msgs::msg::Quaternion& quat) {
  tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

bool WheelOdomCalibrator::findClosestImuData(const rclcpp::Time& target_time,
                                             double& imu_yaw,
                                             double& time_diff) {
  if (imu_buffer_.empty()) {
    return false;
  }

  // 二分查找最接近的时间戳
  auto it = std::lower_bound(imu_buffer_.begin(), imu_buffer_.end(),
                             target_time,
                             [](const ImuData& data, const rclcpp::Time& time) {
                               return data.timestamp < time;
                             });

  if (it == imu_buffer_.end()) {
    // 目标时间在所有数据之后
    imu_yaw = imu_buffer_.back().yaw;
    time_diff = (target_time - imu_buffer_.back().timestamp).seconds();
    return true;
  }

  if (it == imu_buffer_.begin()) {
    // 目标时间在所有数据之前
    imu_yaw = imu_buffer_.front().yaw;
    time_diff = (imu_buffer_.front().timestamp - target_time).seconds();
    return true;
  }

  // 比较前后两个数据点，选择更接近的
  auto prev_it = it - 1;
  double diff_next = (it->timestamp - target_time).seconds();
  double diff_prev = (target_time - prev_it->timestamp).seconds();

  if (diff_next < diff_prev) {
    imu_yaw = it->yaw;
    time_diff = diff_next;
  } else {
    imu_yaw = prev_it->yaw;
    time_diff = diff_prev;
  }

  return true;
}

void WheelOdomCalibrator::saveCalibrationResult(double wheel_angle, double imu_angle) {
  AngularCalibrationResult result;
  result.wheel_angle = std::abs(wheel_angle) * 180.0 / M_PI;
  result.imu_angle = std::abs(imu_angle) * 180.0 / M_PI;
  result.scale_factor = result.imu_angle / result.wheel_angle;
  result.timestamp = this->now();

  calibration_results_.push_back(result);

  // 记录到日志文件
  if (log_file_.is_open()) {
    size_t count = calibration_results_.size();
    log_file_ << "【第 " << count << " 次标定】" << std::endl;
    log_file_ << "轮速里程计角度: " << result.wheel_angle << " 度" << std::endl;
    log_file_ << "IMU角度: " << result.imu_angle << " 度" << std::endl;
    log_file_ << "校准系数(IMU/轮速计): " << result.scale_factor << std::endl;
    log_file_.flush();
  }
}

void WheelOdomCalibrator::printCalibrationSummary() {
  if (calibration_results_.empty()) {
    RCLCPP_WARN(this->get_logger(), "没有标定数据");
    return;
  }

  // 计算统计数据
  std::vector<double> scale_factors;
  for (const auto& result : calibration_results_) {
    scale_factors.push_back(result.scale_factor);
  }

  double mean = std::accumulate(scale_factors.begin(), scale_factors.end(), 0.0) / scale_factors.size();
  double variance = 0.0;
  for (double factor : scale_factors) {
    variance += (factor - mean) * (factor - mean);
  }
  variance /= scale_factors.size();
  double stddev = std::sqrt(variance);

  auto min_it = std::min_element(scale_factors.begin(), scale_factors.end());
  auto max_it = std::max_element(scale_factors.begin(), scale_factors.end());

  RCLCPP_INFO(this->get_logger(), "========================================");
  RCLCPP_INFO(this->get_logger(), "     轮速里程计角速度标定结果");
  RCLCPP_INFO(this->get_logger(), "========================================");
  RCLCPP_INFO(this->get_logger(), "标定次数: %zu", calibration_results_.size());
  RCLCPP_INFO(this->get_logger(), "----------------------------------------");
  RCLCPP_INFO(this->get_logger(), "校准系数 (odometry_angular_scale):");
  RCLCPP_INFO(this->get_logger(), "  最佳值: %.6f", mean);
  RCLCPP_INFO(this->get_logger(), "  标准差: %.6f", stddev);
  RCLCPP_INFO(this->get_logger(), "  最小值: %.6f", *min_it);
  RCLCPP_INFO(this->get_logger(), "  最大值: %.6f", *max_it);
  RCLCPP_INFO(this->get_logger(), "  变异系数: %.2f%%", (stddev / mean) * 100.0);
  RCLCPP_INFO(this->get_logger(), "----------------------------------------");
  RCLCPP_INFO(this->get_logger(), "95%%置信区间: [%.6f, %.6f]",
              mean - 1.96 * stddev, mean + 1.96 * stddev);
  RCLCPP_INFO(this->get_logger(), "99%%置信区间: [%.6f, %.6f]",
              mean - 2.576 * stddev, mean + 2.576 * stddev);
  RCLCPP_INFO(this->get_logger(), "========================================");
  RCLCPP_INFO(this->get_logger(), "应用方法：");
  RCLCPP_INFO(this->get_logger(), "在 base.yaml 中设置：");
  RCLCPP_INFO(this->get_logger(), "  odometry_angular_scale: %.6f", mean);
  RCLCPP_INFO(this->get_logger(), "========================================");

  // 输出每次标定的详细信息
  RCLCPP_INFO(this->get_logger(), "详细数据:");
  for (size_t i = 0; i < calibration_results_.size(); ++i) {
    const auto& result = calibration_results_[i];
    double deviation = (result.scale_factor - mean) / mean * 100.0;
    RCLCPP_INFO(this->get_logger(), "  [%2zu] 轮速计: %6.2f°, IMU: %6.2f°, 系数: %.4f (偏差: %+.2f%%)",
                i + 1, result.wheel_angle, result.imu_angle, result.scale_factor, deviation);
  }
  RCLCPP_INFO(this->get_logger(), "========================================");
}

void WheelOdomCalibrator::saveFinalResults() {
  if (!log_file_.is_open() || calibration_results_.empty()) {
    return;
  }

  // 计算统计数据
  std::vector<double> scale_factors;
  for (const auto& result : calibration_results_) {
    scale_factors.push_back(result.scale_factor);
  }

  double mean = std::accumulate(scale_factors.begin(), scale_factors.end(), 0.0) / scale_factors.size();
  double variance = 0.0;
  for (double factor : scale_factors) {
    variance += (factor - mean) * (factor - mean);
  }
  variance /= scale_factors.size();
  double stddev = std::sqrt(variance);

  auto min_it = std::min_element(scale_factors.begin(), scale_factors.end());
  auto max_it = std::max_element(scale_factors.begin(), scale_factors.end());

  // 写入文件
  log_file_ << "========================================" << std::endl;
  log_file_ << "【最终标定结果】" << std::endl;
  log_file_ << "标定次数: " << calibration_results_.size() << std::endl;
  log_file_ << std::fixed << std::setprecision(6);
  log_file_ << "校准系数统计:" << std::endl;
  log_file_ << "  最佳值: " << mean << std::endl;
  log_file_ << "  标准差: " << stddev << std::endl;
  log_file_ << "  最小值: " << *min_it << std::endl;
  log_file_ << "  最大值: " << *max_it << std::endl;
  log_file_ << "  变异系数: " << (stddev / mean) * 100.0 << "%" << std::endl;
  log_file_ << std::endl;
  log_file_ << "置信区间:" << std::endl;
  log_file_ << "  95%置信区间: [" << (mean - 1.96 * stddev) << ", " << (mean + 1.96 * stddev) << "]" << std::endl;
  log_file_ << "  99%置信区间: [" << (mean - 2.576 * stddev) << ", " << (mean + 2.576 * stddev) << "]" << std::endl;
  log_file_ << std::endl;
  log_file_ << "========================================" << std::endl;
  log_file_ << "应用方法：" << std::endl;
  log_file_ << "在 base.yaml 配置文件中添加或修改：" << std::endl;
  log_file_ << std::setprecision(6);
  log_file_ << "  odometry_angular_scale: " << mean << std::endl;
  log_file_ << "========================================" << std::endl;
  log_file_ << std::endl;
  log_file_ << "详细数据:" << std::endl;
  for (size_t i = 0; i < calibration_results_.size(); ++i) {
    const auto& result = calibration_results_[i];
    double deviation = (result.scale_factor - mean) / mean * 100.0;
    log_file_ << "  [" << std::setw(2) << (i + 1) << "] "
               << "轮速计: " << std::setprecision(2) << std::setw(6) << result.wheel_angle << "°, "
               << "IMU: " << std::setw(6) << result.imu_angle << "°, "
               << "系数: " << std::setprecision(4) << result.scale_factor
               << " (偏差: " << std::showpos << deviation << std::noshowpos << "%)"
               << std::endl;
  }
  log_file_ << "========================================" << std::endl;
  log_file_.flush();
}

// 主函数
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelOdomCalibrator>());
  rclcpp::shutdown();
  return 0;
}
