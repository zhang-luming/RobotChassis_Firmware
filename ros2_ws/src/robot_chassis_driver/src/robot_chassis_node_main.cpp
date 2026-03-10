#include "robot_chassis_driver/robot_chassis_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<RobotChassisNode>();

  RCLCPP_INFO(node->get_logger(), "RobotChassis驱动节点已启动");

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
