#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

// Global storage for the target
double target_x = 0.0;
double target_y = 0.0;
double target_z = 0.0;
bool target_found = false;

// Callback: Listen to the Vision Node
void target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    // Only accept targets if we aren't already busy
    if (!target_found && msg->pose.position.x != 0.0) {
        target_x = msg->pose.position.x;
        target_y = msg->pose.position.y;
        target_z = msg->pose.position.z; // Trust the Vision Z!
        target_found = true;
        RCLCPP_INFO(rclcpp::get_logger("sorter"), "TARGET RECEIVED: X=%.2f, Y=%.2f, Z=%.2f", target_x, target_y, target_z);
    }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("simple_sorter");

  // Subscribe to Vision
  auto sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/target_object", 10, target_callback);

  using moveit::planning_interface::MoveGroupInterface;
  auto move_group = MoveGroupInterface(node, "panda_arm");
  auto hand_group = MoveGroupInterface(node, "hand");

  // SAFETY SETTINGS
  move_group.setMaxVelocityScalingFactor(0.4); // Move slow to avoid smashing
  move_group.setMaxAccelerationScalingFactor(0.3);
  move_group.setPoseReferenceFrame("world");

  RCLCPP_INFO(node->get_logger(), "--- SORTER READY: WAITING FOR TARGET ---");

  // 1. MAIN LOOP
  while (rclcpp::ok()) {
      // Wait for target
      while(rclcpp::ok() && !target_found) {
          rclcpp::spin_some(node);
          rclcpp::sleep_for(std::chrono::milliseconds(100));
      }
      
      RCLCPP_INFO(node->get_logger(), "EXECUTING PICK SEQUENCE...");

      // 2. PREPARE GRIPPER
      hand_group.setNamedTarget("open");
      hand_group.move();

      // 3. PHASE 1: THE HOVER (Safe Approach)
      // Go to a point 15cm ABOVE the object. This prevents lateral collisions.
      geometry_msgs::msg::Pose target_pose;
      target_pose.orientation.x = 1.0; 
      target_pose.orientation.y = 0.0;
      target_pose.orientation.z = 0.0;
      target_pose.orientation.w = 0.0;
      
      target_pose.position.x = target_x;
      target_pose.position.y = target_y;
      target_pose.position.z = target_z + 0.10; // Hover 20cm above
      
      move_group.setPoseTarget(target_pose);
      auto success = (move_group.move() == moveit::core::MoveItErrorCode::SUCCESS);
      
      if(success) {
          // 4. PHASE 2: LINEAR DESCENT (The "Elevator" Move)
          // We use Waypoints to force a straight line down
          std::vector<geometry_msgs::msg::Pose> waypoints;
          
          // Start where we are
          waypoints.push_back(target_pose);
          
          // Go Down to Grasp Height
          // Tip: The Vision Z is usually the LID. We want to grab slightly BELOW the lid.
          // BUT we must not crash the palm. 
          // Let's aim for Z + 0.05 (Fingers surround object, palm doesn't touch).
          double grasp_z = target_z + 0.06; 
          if (grasp_z < 0.12) {
              grasp_z = 0.12;
              RCLCPP_WARN(node->get_logger(), "Target too low! Engaging safety floor at Z=0.12");
          }
          
          target_pose.position.z = grasp_z;
          waypoints.push_back(target_pose);

          moveit_msgs::msg::RobotTrajectory trajectory;
          const double jump_threshold = 0.0;
          const double eef_step = 0.01; // 1cm steps
          
          double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
          
          if(fraction > 0.9) { // If path is valid
              move_group.execute(trajectory);
              rclcpp::sleep_for(std::chrono::milliseconds(1000));
              
              // 5. PHASE 3: GRASP
              rclcpp::sleep_for(std::chrono::milliseconds(500)); // Pause to settle
              hand_group.setNamedTarget("close");
              hand_group.move();
              
              // 6. PHASE 4: LIFT (Linear Move Up)
              waypoints.clear();
              waypoints.push_back(target_pose); // Current low position
              target_pose.position.z = target_z + 0.20; // Back to Hover
              waypoints.push_back(target_pose);
              
              move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
              move_group.execute(trajectory);
              
              // 7. PHASE 5: DROP OFF
              // Move to Bin (Hardcoded for now, or use TF)
              target_pose.position.x = 0.0;
              target_pose.position.y = -0.5;
              target_pose.position.z = 0.5;
              move_group.setPoseTarget(target_pose);
              move_group.move();
              
              hand_group.setNamedTarget("open");
              hand_group.move();
          } else {
              RCLCPP_ERROR(node->get_logger(), "Cartesian Path Failed! (Unreachable)");
          }
      } else {
          RCLCPP_ERROR(node->get_logger(), "Hover Approach Failed!");
      }

      // Reset for next object
      target_found = false;
      
      // Return to Home/Ready Position
      target_pose.position.x = 0.5;
      target_pose.position.y = 0.0;
      target_pose.position.z = 0.5;
      move_group.setPoseTarget(target_pose);
      move_group.move();
  }

  rclcpp::shutdown();
  return 0;
}