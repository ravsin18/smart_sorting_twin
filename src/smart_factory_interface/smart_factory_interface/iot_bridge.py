#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import paho.mqtt.client as mqtt
import json
from sensor_msgs.msg import JointState

# CONFIGURATION
MQTT_BROKER = "localhost" 
MQTT_PORT = 1883
MQTT_TOPIC = "robot/telemetry"

class IoTBridge(Node):
    def __init__(self):
        super().__init__('iot_bridge_node')
        
        # 1. Connect to MQTT Broker
        self.mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, "ROS2_Bridge")
        try:
            self.mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
            self.mqtt_client.loop_start() # Run in background
            self.get_logger().info(f"Connected to MQTT Broker at {MQTT_BROKER}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to MQTT: {e}")

        # 2. Subscribe to Robot Joints
        self.create_subscription(
            JointState,
            '/joint_states', # The topic where the robot broadcasts its position
            self.joint_callback,
            10
        )

    def joint_callback(self, msg):
        # 3. Convert ROS Data to JSON
        # We take the first 3 joints just for the dashboard
        payload = {
            "robot_id": "panda_1",
            "joint_positions": list(msg.position)[:4], 
            "timestamp": self.get_clock().now().nanoseconds
        }

        # 4. Publish to Digital Twin
        json_str = json.dumps(payload)
        self.mqtt_client.publish(MQTT_TOPIC, json_str)
        self.get_logger().info(f"Published: {json_str[:50]}...")

def main(args=None):
    rclpy.init(args=args)
    node = IoTBridge()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()