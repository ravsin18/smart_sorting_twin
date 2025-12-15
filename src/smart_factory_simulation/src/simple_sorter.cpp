#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("simple_sorter");

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("simple_sorter", node_options);

  using moveit::planning_interface::MoveGroupInterface;
  auto move_group = MoveGroupInterface(node, "panda_arm");
  auto hand_group = MoveGroupInterface(node, "hand");

  move_group.setMaxVelocityScalingFactor(0.5);
  move_group.setMaxAccelerationScalingFactor(0.5);
  
  // CRITICAL: Tell MoveIt we are using World Coordinates
  move_group.setPoseReferenceFrame("world");

  RCLCPP_INFO(LOGGER, "--- ROBOT READY. STARTING HIGH-TABLE SORTING ---");

  // 1. OPEN GRIPPER
  RCLCPP_INFO(LOGGER, "1. Opening Gripper...");
  hand_group.setNamedTarget("open");
  hand_group.move();

  // 2. APPROACH COKE CAN (HIGH TABLE COORDINATES)
  geometry_msgs::msg::Pose target_pose;
  // Orientation: Pointing down
  target_pose.orientation.x = 0.924; 
  target_pose.orientation.y = -0.382;
  target_pose.orientation.z = 0.0;
  target_pose.orientation.w = 0.0;
  
  // Position: Coke Can is at x=0.6, y=-0.2, z=1.05
  target_pose.position.x = 0.6;
  target_pose.position.y = -0.2;
  target_pose.position.z = 1.25; // Hover 20cm ABOVE the table (z=1.25)

  RCLCPP_INFO(LOGGER, "2. Moving to Pre-Grasp (High)...");
  move_group.setPoseTarget(target_pose);
  
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  if(move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      move_group.execute(my_plan);
  } else {
      RCLCPP_ERROR(LOGGER, "Failed to plan approach! Check collisions.");
      return 1;
  }

  // 3. DESCEND TO GRASP
  RCLCPP_INFO(LOGGER, "3. Descending...");
  target_pose.position.z = 1.35; // Exact object height
  move_group.setPoseTarget(target_pose);
  move_group.move();

  // 4. GRASP
  RCLCPP_INFO(LOGGER, "4. Grasping...");
  hand_group.setNamedTarget("close"); 
  hand_group.move();
  rclcpp::sleep_for(std::chrono::seconds(1));

  // 5. LIFT
  RCLCPP_INFO(LOGGER, "5. Lifting...");
  target_pose.position.z = 1.30; // Lift high to clear bins
  move_group.setPoseTarget(target_pose);
  move_group.move();

  // 6. MOVE TO RED BIN (Left side)
  // Bin is at x=-0.5, y=-0.6, z=0.8 (Top edge is higher)
  RCLCPP_INFO(LOGGER, "6. Moving to Bin...");
  target_pose.position.x = -0.5;
  target_pose.position.y = -0.6; 
  target_pose.position.z = 1.30; // Stay high!
  move_group.setPoseTarget(target_pose);
  move_group.move();

  // 7. DROP
  RCLCPP_INFO(LOGGER, "7. Dropping...");
  hand_group.setNamedTarget("open");
  hand_group.move();

  RCLCPP_INFO(LOGGER, "--- MISSION COMPLETE ---");
  rclcpp::shutdown();
  return 0;
}