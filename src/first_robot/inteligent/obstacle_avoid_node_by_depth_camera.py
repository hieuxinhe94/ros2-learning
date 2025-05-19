#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from enum import Enum
import random
import time
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


class PatrolState(Enum):
    FORWARD = 1
    TURN_LEFT = 2
    TURN_RIGHT = 3
    STOP = 4

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')
        self.last_image_time = self.get_clock().now()
        
        self.publisher = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        # MobilenetSSD detect obstacle by depth camera
        # self.subscription = self.create_subscription(Image, '/camera', self.depth_image_callback_mobilenetssd, 10)
        self.subscription_rgb = self.create_subscription(Image, '/camera/color/image_raw', self.rgb_callback, 10)
        self.subscription_depth = self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, 10)

        self.bridge = CvBridge()
        self.latest_rgb = None
        self.latest_depth = None
        
        self.state = PatrolState.FORWARD
        self.state_timer = 0.0
        self.state_duration = 1.0

        self.obstacle_detected = False
        self.timer = self.create_timer(2, self.update_callback) # 0.5 giây một lần
 
    def load_mobilenet_ssd(self):
        pkg_path = get_package_share_directory("first_robot")
        model_path = os.path.join(pkg_path, "ai_models", "MobileNetSSD_deploy.caffemodel")
        prototxt_path = os.path.join(pkg_path, "ai_models", "MobileNetSSD_deploy.prototxt")

        self.net = cv2.dnn.readNetFromCaffe(prototxt_path, model_path)
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

    def rgb_callback(self, msg):
        self.get_logger().info("depth_callback - using MobileNetSSD")        
        try:
            self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.process_obstacle()
        except Exception as e:
            self.get_logger().error(f"RGB callback error: {e}")
            
    def depth_callback(self, msg):
        self.get_logger().info("depth_callback - using MobileNetSSD")
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Depth callback error: {e}")                    
 
    def depth_image_callback_mobilenetssd(self, msg):
        now = self.get_clock().now()
        time_diff = (now - self.last_image_time).nanoseconds * 1e-9  # giây

        if time_diff < 1.0:  # chỉ xử lý ảnh mỗi 1 giây
            return
        self.last_image_time = now
        self.get_logger().info(f"depth_image_callback_mobilenetssd self.latest_rgb is None: {self.latest_rgb is None } self.latest_depth is None: {self.latest_depth is None}")        
        if self.latest_rgb is None or self.latest_depth is None:
            return
    
        try:
            # cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            frame = self.latest_rgb.copy()
            
            # Load MobileNet SSD model if not already loaded
            if not hasattr(self, 'net'):
                self.get_logger().info("image_callback - loading MobileNetSSD for first time")
                self.load_mobilenet_ssd()
            self.get_logger().info("image_callback - Resize + normalize input for MobileNetSSD")
            # Resize + normalize input for MobileNetSSD
            blob = cv2.dnn.blobFromImage(frame, 0.007843, (300, 300), 127.5)
           
            self.net.setInput(blob)
            detections = self.net.forward()

            height, width = frame.shape[:2]
            self.obstacle_detected = False
            steer_direction = "forward"
            
            for i in range(detections.shape[2]):
                confidence = detections[0, 0, i, 2]
                class_id = int(detections[0, 0, i, 1]) # class_id detected is mobilenetssd classes 
                
                if confidence > 0.5:  # Ngưỡng confidence
                    box = detections[0, 0, i, 3:7] * np.array([width, height, width, height])
                    (x1, y1, x2, y2) = box.astype("int")
                    
                    center_x = (x1 + x2) // 2
                    center_y = (y1 + y2) // 2
                    
                    # Lấy khoảng cách từ depth image
                    if center_y < self.latest_depth.shape[0] and center_x < self.latest_depth.shape[1]:
                        distance = self.latest_depth[center_y, center_x] / 1000.0  # mm -> m
                        if 0.2 < distance < 1.5 and y2 > height * 0.6:  # chỉ xử lý vật cản gần
                            self.obstacle_detected = True
                            steer_direction = "left" if center_x > width // 2 else "right"
                            break

                    # Optionally draw the box:
                    cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

                    # Check if the object is in ROI (vùng phía trước robot)
            self.steer_direction = steer_direction
            self.get_logger().info(f"[SSD] Obstacle: {self.obstacle_detected} | Steer: {self.steer_direction}")
            # Optionally: hiển thị ảnh debug
            # cv2.imshow("SSD Output", cv_image)
            # cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error in image_callback: {e}")
        
        
    def update_callback(self):
        self.get_logger().info("update_callback")
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()

        self.state_timer += 1

        # Nếu phát hiện vật cản → chuyển sang rẽ trái/phải
        if self.obstacle_detected:
            self.get_logger().info(f"obstacle_detected: self.state {self.state} steer_direction {self.steer_direction}")
            if self.state not in [PatrolState.TURN_LEFT, PatrolState.TURN_RIGHT]:
                self.get_logger().info("Nếu phát hiện vật cản → chuyển sang rẽ trái/phải 3s")
                if self.steer_direction == "left":
                    self.state = PatrolState.TURN_RIGHT
                elif self.steer_direction == "right":
                    self.state = PatrolState.TURN_LEFT
                elif self.steer_direction == "forward":
                    self.state = PatrolState.FORWARD
                self.state_timer = 0.0
                self.state_duration = 3.0
        else:
            # Khi rẽ xong mà không còn vật cản → đi thẳng tiếp
            if self.state in [PatrolState.TURN_LEFT, PatrolState.TURN_RIGHT] and self.state_timer >= self.state_duration:
                self.get_logger().info("Khi rẽ xong mà không còn vật cản → đi thẳng tiếp 2s")
                self.state = PatrolState.FORWARD
                self.state_timer = 3.0
                self.state_duration = 5.0

        # Tạo lệnh tương ứng với trạng thái
        self.get_logger().info(f"PatrolState action: {self.state}")
        if self.state == PatrolState.FORWARD:
            msg.twist.linear.x = 0.1
            msg.twist.angular.z = 0.0
        elif self.state == PatrolState.TURN_LEFT:
            msg.twist.linear.x = 0.0
            msg.twist.angular.z = 0.4
        elif self.state == PatrolState.TURN_RIGHT:
            msg.twist.linear.x = 0.0
            msg.twist.angular.z = -0.4
        elif self.state == PatrolState.STOP:
            msg.twist.linear.x = 0.0
            msg.twist.angular.z = 0.0

        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
