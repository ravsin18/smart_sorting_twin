import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    
    # 1. Get the path to YOUR Custom Robot Description (Pedestal Version)
    # We must use the same description as Gazebo, or the math will be wrong
    pkg_smart_factory = get_package_share_directory('smart_factory_simulation')
    xacro_file = os.path.join(pkg_smart_factory, 'urdf', 'smart_panda.urdf.xacro')

    # 2. Load MoveIt Configs with the Custom URDF
    moveit_config = MoveItConfigsBuilder("moveit_resources_panda") \
        .robot_description(file_path=xacro_file) \
        .to_moveit_configs()

    # 3. Launch the C++ Sorter Node
    sorter_node = Node(
        package='smart_factory_simulation',
        executable='simple_sorter',
        output='screen',
        parameters=[
            moveit_config.to_dict(), # Pass URDF/SRDF to the node
            {"use_sim_time": True},  # Sync with Gazebo clock
        ],
    )

    return LaunchDescription([
        sorter_node
    ])