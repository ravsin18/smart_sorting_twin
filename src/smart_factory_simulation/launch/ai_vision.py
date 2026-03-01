#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import open3d as o3d
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
import pyransac3d as pyrsc
import torch

from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs

class RGBDVision3D(Node):
    def __init__(self):
        super().__init__('rgbd_vision_3d_node')
        
        self.color_sub = self.create_subscription(Image, '/smart_depth_camera/image_raw', self.color_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/smart_depth_camera/depth/image_raw', self.depth_callback, 10)
        self.info_sub = self.create_subscription(CameraInfo, '/smart_depth_camera/camera_info', self.info_callback, 10)
        self.pc_pub = self.create_publisher(PointCloud2, '/isolated_object_cloud', 10)
        self.target_pub = self.create_publisher(PoseStamped, '/target_object', 10)
        
        self.bridge = CvBridge()
        self.model = YOLO("yolov8n.pt") 
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.current_depth_image = None
        self.fx = self.fy = self.cx = self.cy = None
        
        self.get_logger().info("--- 3D POINT CLOUD VISION READY ---")

    def info_callback(self, msg):
        if self.fx is None:
            self.fx = msg.k[0]
            self.cx = msg.k[2]
            self.fy = msg.k[4]
            self.cy = msg.k[5]

    def depth_callback(self, msg):
        try:
            self.current_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except Exception as e:
            pass

    def color_callback(self, msg):
            if self.current_depth_image is None or self.fx is None:
                return

            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            results = self.model.predict(cv_image, verbose=False, conf=0.20)

            # --- NEW: TRACK THE CLOSEST OBJECT ---
            closest_dist = float('inf')
            best_centroid = None
            best_points = None
            best_stance = None
            best_box = None
            best_label = None

            for result in results:
                for box in result.boxes:
                    label = self.model.names[int(box.cls[0])]
                    if label not in ['bottle', 'cup', 'can']:
                        continue

                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    h, w = self.current_depth_image.shape
                    x1, y1 = max(0, x1), max(0, y1)
                    x2, y2 = min(w, x2), min(h, y2)

                    # Extract Depth ROI & Convert to 3D Points
                    depth_roi = self.current_depth_image[y1:y2, x1:x2]
                    uu, vv = np.meshgrid(np.arange(x1, x2), np.arange(y1, y2))
                    uu, vv, zz = uu.flatten(), vv.flatten(), depth_roi.flatten()

                    valid = (zz > 0.1) & (zz < 1.5) & (~np.isnan(zz))
                    uu, vv, zz = uu[valid], vv[valid], zz[valid]

                    if len(zz) < 50:
                        continue

                    # Background Masking
                    median_z = np.median(zz)
                    object_mask = (zz > median_z - 0.05) & (zz < median_z + 0.05)
                    uu, vv, zz = uu[object_mask], vv[object_mask], zz[object_mask]

                    if len(zz) < 20:
                        continue

                    xx = (uu - self.cx) * zz / self.fx
                    yy = (vv - self.cy) * zz / self.fy
                    points_3d = np.vstack((xx, yy, zz)).T

                    # Open3D Magic
                    pcd = o3d.geometry.PointCloud()
                    pcd.points = o3d.utility.Vector3dVector(points_3d)
                    pcd = pcd.voxel_down_sample(voxel_size=0.005)
                    pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

                    if len(pcd.points) < 20:
                        continue

                    obb = pcd.get_oriented_bounding_box()
                    centroid = obb.get_center()

                    # --- 3D PYTHAGOREAN DISTANCE ---
                    # Calculate how far this specific object is from the camera lens
                    dist = np.sqrt(centroid[0]**2 + centroid[1]**2 + centroid[2]**2)

                    # If this is the closest object we've seen so far, save it!
                    if dist < closest_dist:
                        closest_dist = dist
                        best_centroid = centroid
                        best_points = np.asarray(pcd.points)
                        best_box = (x1, y1, x2, y2)
                        best_label = label
                        
                        pixel_width, pixel_height = (x2 - x1), (y2 - y1)
                        best_stance = "STANDING" if pixel_height > (pixel_width * 1.1) else "LYING FLAT"

            # --- AFTER THE LOOP: ONLY PUBLISH THE CLOSEST TARGET ---
# --- AFTER THE LOOP: ONLY PUBLISH THE CLOSEST TARGET ---
# --- AFTER THE LOOP: ONLY PUBLISH THE CLOSEST TARGET ---
# --- AFTER THE LOOP: ONLY PUBLISH THE CLOSEST TARGET ---
            if best_centroid is not None:
                
                # 1. PYTORCH GPU INTEGRATION (Send points to the AI)
                try:
                    points_tensor = torch.tensor(best_points, dtype=torch.float32).to('cuda')
                    self.get_logger().info(f"AI Ready: Sent {points_tensor.shape[0]} points to {points_tensor.device}")
                except Exception as e:
                    self.get_logger().error(f"Failed to load Tensor to GPU: {e}")

                # 2. RANSAC CYLINDER FITTING (This calculates target_x, target_y, target_z!)
                try:
                    cyl = pyrsc.Cylinder()
                    center, direction, radius, inliers = cyl.fit(best_points, thresh=0.002, maxIteration=2000)
                    
                    if radius > 0.05 or radius < 0.015:
                        raise ValueError(f"Hallucinated radius: {radius*100:.1f}cm")
                    
                    target_x = float(center[0])
                    target_y = float(best_centroid[1]) 
                    target_z = float(center[2])
                    self.get_logger().info(f"RANSAC SUCCESS: Radius {radius*100:.1f}cm")

                except Exception as e:
                    # Fallback
                    self.get_logger().warn(f"RANSAC Rejected ({str(e)}). Using 3cm offset.")
                    target_x = float(best_centroid[0])
                    target_y = float(best_centroid[1])
                    target_z = float(best_centroid[2]) + 0.03
                    
                # 3. PUBLISH COORDINATE TO C++ NODE
                cam_pose = PoseStamped()
                cam_pose.header.frame_id = "camera_link_optical"
                cam_pose.header.stamp = self.get_clock().now().to_msg()
                
                cam_pose.pose.position.x = target_x
                cam_pose.pose.position.y = target_y
                cam_pose.pose.position.z = target_z
                
                # The Stance Flag Bridge
                if best_stance == "STANDING":
                    cam_pose.pose.orientation.w = 2.0  
                else:
                    cam_pose.pose.orientation.w = 1.0  

                try:
                    if self.tf_buffer.can_transform("world", "camera_link_optical", rclpy.time.Time()):
                        world_pose = self.tf_buffer.transform(cam_pose, "world")
                        self.target_pub.publish(world_pose)
                except Exception as e:
                    pass

                # 4. PUBLISH POINT CLOUD & DRAW UI 
                header = Header()
                header.stamp = self.get_clock().now().to_msg()
                header.frame_id = "camera_link_optical"
                pc_msg = pc2.create_cloud_xyz32(header, best_points)
                self.pc_pub.publish(pc_msg)

                bx1, by1, bx2, by2 = best_box
                cv2.rectangle(cv_image, (bx1, by1), (bx2, by2), (0, 255, 0), 3)
                cv2.putText(cv_image, f"TARGET: {best_label} ({best_stance})", (bx1, by1-10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            cv2.imshow("3D Point Cloud Vision", cv_image)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = RGBDVision3D()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()