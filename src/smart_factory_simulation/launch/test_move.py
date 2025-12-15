import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def main(args=None):
    rclpy.init(args=args)
    node = Node('test_mover')
    
    # Create a publisher to talk DIRECTLY to the Gazebo controller
    publisher = node.create_publisher(JointTrajectory, '/panda_arm_controller/joint_trajectory', 10)
    
    msg = JointTrajectory()
    msg.joint_names = [
        'panda_joint1', 'panda_joint2', 'panda_joint3', 
        'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7'
    ]
    
    point = JointTrajectoryPoint()
    # Move Joint 1 to 1.57 (90 degrees) and Joint 4 to -1.57
    point.positions = [1.57, 0.0, 0.0, -1.57, 0.0, 0.0, 0.0]
    point.time_from_start.sec = 2  # Take 2 seconds to get there
    
    msg.points.append(point)
    
    print("Sending Direct Command to Gazebo...")
    # Send multiple times to make sure it catches it
    for i in range(5):
        publisher.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.5)

    print("Command Sent!")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()