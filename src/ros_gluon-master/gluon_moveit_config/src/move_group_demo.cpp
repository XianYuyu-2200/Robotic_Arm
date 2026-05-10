#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("gluon_move_group_demo");
  
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  static const std::string PLANNING_GROUP = "gluon";
  moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

  move_group.setMaxVelocityScalingFactor(0.5);
  move_group.setMaxAccelerationScalingFactor(0.5);

  RCLCPP_INFO(node->get_logger(), "Moving to home_pose");
  move_group.setNamedTarget("home_pose");
  move_group.move();

  rclcpp::sleep_for(std::chrono::seconds(1));

  RCLCPP_INFO(node->get_logger(), "Moving to test_pose-1");
  move_group.setNamedTarget("test_pose-1");
  move_group.move();

  rclcpp::sleep_for(std::chrono::seconds(1));

  RCLCPP_INFO(node->get_logger(), "Moving to test_pose-2");
  move_group.setNamedTarget("test_pose-2");
  move_group.move();

  rclcpp::sleep_for(std::chrono::seconds(1));

  RCLCPP_INFO(node->get_logger(), "Moving back to home_pose");
  move_group.setNamedTarget("home_pose");
  move_group.move();

  rclcpp::shutdown();
  return 0;
}
