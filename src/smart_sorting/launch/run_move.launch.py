from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 1. Load the Robot Description (URDF/SRDF) automatically
    moveit_config = MoveItConfigsBuilder("panda", package_name="moveit_resources_panda_moveit_config").to_moveit_configs()

    # 2. Configure the Node
    move_node = Node(
        package="smart_sorting",
        executable="simple_move_node",
        output="screen",
        # This is the magic line: Pass the description to the node
        parameters=[moveit_config.to_dict()]
    )

    return LaunchDescription([move_node])