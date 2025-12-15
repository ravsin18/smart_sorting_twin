import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_smart_factory = get_package_share_directory('smart_factory_simulation')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    # --- CRITICAL FIX: FIND THE ROBOT MESHES ---
    # Gazebo needs to know where 'moveit_resources_panda_description' is.
    # We find the package, then get its PARENT directory (usually 'share')
    # and add that to GAZEBO_MODEL_PATH.
    try:
        pkg_panda_description = get_package_share_directory('moveit_resources_panda_description')
        model_path = os.path.dirname(pkg_panda_description) # Points to .../share
        
        # Also add our own models (bins, etc.)
        my_models = os.path.join(pkg_smart_factory, 'models')
        
        # Combine paths
        if 'GAZEBO_MODEL_PATH' in os.environ:
            model_path += os.pathsep + my_models + os.pathsep + os.environ['GAZEBO_MODEL_PATH']
        else:
            model_path += os.pathsep + my_models
            
        print(f"DEBUG: Setting GAZEBO_MODEL_PATH to {model_path}")
    except Exception as e:
        print(f"Error finding panda description: {e}")
        model_path = ""

    # 1. Gazebo World
    world_path = os.path.join(pkg_smart_factory, 'worlds', 'factory.world')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_path}.items(),
    )

    # 2. Robot Description (Xacro -> URDF)
    xacro_file = os.path.join(pkg_smart_factory, 'urdf', 'smart_panda.urdf.xacro')
    doc = xacro.process_file(xacro_file)
    robot_description = {'robot_description': doc.toxml()}

    # 3. Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # 4. Spawn The Robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'panda'],
        output='screen'
    )

    # 5. Load Controllers
    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    load_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["panda_arm_controller"],
        output="screen",
    )

    load_hand_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["panda_hand_controller"],
        output="screen",
    )

    return LaunchDescription([
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path),
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_arm_controller, load_hand_controller],
            )
        ),
    ])