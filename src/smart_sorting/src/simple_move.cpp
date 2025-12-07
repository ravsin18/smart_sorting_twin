#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <thread> // Needed for sleep

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("simple_move_cpp");
  
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "panda_arm");

  // We will loop 3 times
  for(int i=0; i<3; i++) {
      RCLCPP_INFO(node->get_logger(), "Starting Loop %d...", i+1);

      // 1. Go to 'Ready' Pose (Standing up)
      RCLCPP_INFO(node->get_logger(), "Moving to READY pose...");
      move_group_interface.setNamedTarget("ready");
      move_group_interface.move();
      std::this_thread::sleep_for(std::chrono::seconds(1)); // Pause for 1 second

      // 2. Go to 'Extended' Pose (Reaching out)
      RCLCPP_INFO(node->get_logger(), "Moving to EXTENDED pose...");
      move_group_interface.setNamedTarget("extended");
      move_group_interface.move();
      std::this_thread::sleep_for(std::chrono::seconds(1)); // Pause for 1 second
  }

  RCLCPP_INFO(node->get_logger(), "Dance finished!");
  rclcpp::shutdown();
  return 0;
}