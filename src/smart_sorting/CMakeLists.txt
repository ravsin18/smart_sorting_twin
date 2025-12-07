cmake_minimum_required(VERSION 3.8)
project(smart_sorting)

# 1. Find the libraries we need
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_move_group REQUIRED)

# 2. Define the executable (our node)
add_executable(simple_move_node src/simple_move.cpp)

# 3. Link the libraries to the executable
ament_target_dependencies(simple_move_node
  rclcpp
  moveit_ros_planning_interface
  moveit_ros_move_group
)

# 4. Install the executable so 'ros2 run' can find it
install(TARGETS
  simple_move_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()