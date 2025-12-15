#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import numpy as np

class SmartVision(Node):
    def __init__(self):
        super().__init__('smart_vision_node')
        
        # 1. Subscribe to the Robot's Camera
        self.subscription = self.create_subscription(
            Image,
            '/smart_camera/image_raw',
            self.image_callback,
            10)
        
        # 2. Setup Tools
        self.bridge = CvBridge()
        # Load the smallest standard YOLO model (downloads automatically)
        self.model = YOLO("yolov8n.pt") 
        
        self.get_logger().info("--- AI VISION SYSTEM INITIALIZED ---")

    def image_callback(self, msg):
        try:
            # 1. Convert ROS Image -> OpenCV Image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # 2. Run YOLO Inference (The AI Brain)
            # classes=[39, 41] filters for 'bottle' and 'cup' (Cans often detect as cups/bottles)
            results = self.model.predict(cv_image, verbose=False, conf=0.5)

            # 3. Draw Detections
            for result in results:
                for box in result.boxes:
                    # Get coordinates
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    conf = float(box.conf[0])
                    cls = int(box.cls[0])
                    label = self.model.names[cls]

                    # Draw Box
                    cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(cv_image, f"{label} {conf:.2f}", (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    
                    # Log what we see
                    center_x = (x1 + x2) // 2
                    center_y = (y1 + y2) // 2
                    self.get_logger().info(f"Detected {label} at Pixel: ({center_x}, {center_y})")

            # 4. Show the "Robot's Eye View"
            cv2.imshow("Robot Vision", cv_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error in vision loop: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SmartVision()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()