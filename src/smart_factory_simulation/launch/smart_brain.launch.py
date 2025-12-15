import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import xacro

def generate_launch_description():
    
    # 1. Get the path to YOUR Custom Robot Description (with Pedestal)
    pkg_smart_factory = get_package_share_directory('smart_factory_simulation')
    xacro_file = os.path.join(pkg_smart_factory, 'urdf', 'smart_panda.urdf.xacro')

    # 2. Tell MoveIt to use YOUR file, not the default one
    moveit_config = MoveItConfigsBuilder("moveit_resources_panda") \
        .robot_description(file_path=xacro_file) \
        .to_moveit_configs()

    # 3. Get Controller Config
    controller_config = os.path.join(pkg_smart_factory, 'config', 'moveit_controllers.yaml')

    # 4. Start the Move Group Node (The Brain)
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
            {"trajectory_execution.allowed_start_tolerance": 0.05},
            {"trajectory_execution.allowed_execution_duration_scaling": 2.0},
            {"trajectory_execution.allow_trajectory_execution": True},
            {"moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager"},
            controller_config,
        ],
    )

    # 5. Start RViz (The Visualizer)
    run_rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", str(moveit_config.package_path / "launch/moveit.rviz")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": True}
        ],
    )

    return LaunchDescription([
        run_move_group_node,
        run_rviz_node
    ])