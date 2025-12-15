cmake_minimum_required(VERSION 3.8)
project(smart_factory_simulation)

# Default to C++14 or 17 (MoveIt often requires new standards)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. FIND PACKAGES (Critical Step!)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_move_group REQUIRED)
# You likely also need these for the message types (Pose, etc.)
find_package(geometry_msgs REQUIRED) 
find_package(moveit_msgs REQUIRED)

# 2. BUILD THE NODE
add_executable(simple_sorter src/simple_sorter.cpp)
ament_target_dependencies(simple_sorter
  rclcpp
  moveit_ros_planning_interface
  moveit_ros_move_group
  geometry_msgs
  moveit_msgs
)

# 3. INSTALL RULES
# Install the Executable
install(TARGETS
  simple_sorter
  DESTINATION lib/${PROJECT_NAME}
)

# Install the Assets (Launch, Worlds, URDF, Config, Meshes)
install(DIRECTORY launch worlds urdf config meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()