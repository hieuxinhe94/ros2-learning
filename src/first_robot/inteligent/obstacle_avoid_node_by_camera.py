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
        
        # OPEN_CV detect obstacle
        #self.subscription = self.create_subscription(Image, '/camera', self.image_callback_opencv, 10)
        
        # MobilenetSSD detect obstacle
        self.subscription = self.create_subscription(Image, '/camera', self.depth_image_callback_mobilenetssd, 10)

        self.bridge = CvBridge()
        self.state = PatrolState.FORWARD
        self.state_timer = 0.0
        self.state_duration = 1.0

        self.obstacle_detected = False
        self.timer = self.create_timer(2, self.update_callback) # 0.5 giây một lần
 
    def image_callback_opencv(self, msg):
        now = self.get_clock().now()
        time_diff = (now - self.last_image_time).nanoseconds * 1e-9  # giây

        if time_diff < 1.0:  # chỉ xử lý ảnh mỗi 1 giây
            return
        self.last_image_time = now
        
        try:
            self.get_logger().info("image_callback")
            self.get_logger().info("camera_sensor/image analysis processing")

            # Chuyển đổi từ ROS Image message sang OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            _, thresh = cv2.threshold(blurred, 50, 255, cv2.THRESH_BINARY_INV)

            height, width = thresh.shape
            roi = thresh[int(height * 0.6):, int(width * 0.3):int(width * 0.7)]

            obstacle_ratio = np.sum(roi == 255) / roi.size
            self.obstacle_detected = obstacle_ratio > 0.2

            self.get_logger().info(f"obstacle_ratio: {obstacle_ratio:.2f}, obstacle_detected: {self.obstacle_detected}")

        except Exception as e:
            self.get_logger().error(f"Image conversion error: {str(e)}")
            
    def image_callback_mobilenetssd(self, msg):
        now = self.get_clock().now()
        time_diff = (now - self.last_image_time).nanoseconds * 1e-9  # giây

        if time_diff < 1.0:  # chỉ xử lý ảnh mỗi 1 giây
            return
        self.last_image_time = now
        
        
        try:
            self.get_logger().info("image_callback - using MobileNetSSD")
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Load MobileNet SSD model if not already loaded
            if not hasattr(self, 'net'):
                self.get_logger().info("image_callback - loading MobileNetSSD for first time")
                pkg_path = get_package_share_directory("first_robot") 
                
                model_path = os.path.join(pkg_path, "ai_models", "MobileNetSSD_deploy.caffemodel")
                prototxt_path = os.path.join(pkg_path, "ai_models", "MobileNetSSD_deploy.prototxt")

                self.get_logger().info(f"model_path - {model_path}")
                self.get_logger().info(f"prototxt_path - {prototxt_path}")
                
                self.net = cv2.dnn.readNetFromCaffe(prototxt_path, model_path)
                self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
                self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

            # Resize + normalize input for MobileNetSSD
            blob = cv2.dnn.blobFromImage(cv_image, 0.007843, (300, 300), 127.5)
            self.net.setInput(blob)
            detections = self.net.forward()

            height, width = cv_image.shape[:2]
            self.obstacle_detected = False

            for i in range(detections.shape[2]):
                confidence = detections[0, 0, i, 2]
                class_id = int(detections[0, 0, i, 1])
                
                if confidence > 0.5:  # Ngưỡng confidence
                    box = detections[0, 0, i, 3:7] * np.array([width, height, width, height])
                    (x1, y1, x2, y2) = box.astype("int")
                    
                    center_x = (x1 + x2) // 2
                    # Phân tích vị trí ngang
                    if center_x < width / 3:
                        direction = "right"
                    elif center_x > width * 2 / 3:
                        direction = "left"
                    else:
                        direction = "back_or_random"

                    # Optionally draw the box:
                    cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

                    # Check if the object is in ROI (vùng phía trước robot)
                    if y2 > height * 0.6:
                        self.obstacle_detected = True
                        self.obstacle_detected_side = direction
                        break

            self.get_logger().info(f"[SSD] Obstacle Detected: {self.obstacle_detected}")

            # Optionally: hiển thị ảnh debug
            # cv2.imshow("SSD Output", cv_image)
            # cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error in image_callback: {e}")
    
    def depth_image_callback_mobilenetssd(self, msg):
        now = self.get_clock().now()
        time_diff = (now - self.last_image_time).nanoseconds * 1e-9  # giây

        if time_diff < 1.0:  # chỉ xử lý ảnh mỗi 1 giây
            return
        self.last_image_time = now
        
        
        try:
            self.get_logger().info("image_callback - using MobileNetSSD")
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Load MobileNet SSD model if not already loaded
            if not hasattr(self, 'net'):
                self.get_logger().info("image_callback - loading MobileNetSSD for first time")
                pkg_path = get_package_share_directory("first_robot") 
                
                model_path = os.path.join(pkg_path, "ai_models", "MobileNetSSD_deploy.caffemodel")
                prototxt_path = os.path.join(pkg_path, "ai_models", "MobileNetSSD_deploy.prototxt")

                self.get_logger().info(f"model_path - {model_path}")
                self.get_logger().info(f"prototxt_path - {prototxt_path}")
                
                self.net = cv2.dnn.readNetFromCaffe(prototxt_path, model_path)
                self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
                self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

            # Resize + normalize input for MobileNetSSD
            blob = cv2.dnn.blobFromImage(cv_image, 0.007843, (300, 300), 127.5)
            self.net.setInput(blob)
            detections = self.net.forward()

            height, width = cv_image.shape[:2]
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
                    if center_y < self.depth_image.shape[0] and center_x < self.depth_image.shape[1]:
                        distance = self.depth_image[center_y, center_x] / 1000.0  # mm -> m
                        if 0.2 < distance < 1.5:  # chỉ xử lý vật cản gần
                            obstacle_detected = True
                            steer_direction = "left" if center_x > width // 2 else "right"
                            cv2.rectangle(self.rgb_image, (x1, y1), (x2, y2), (0, 0, 255), 2)
                            cv2.putText(self.rgb_image, f"{self.classes[class_id]} {distance:.2f}m", 
                                        (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                    
                    # Optionally draw the box:
                    cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

                    # Check if the object is in ROI (vùng phía trước robot)
                    if y2 > height * 0.6:
                        self.obstacle_detected = True
                        self.steer_direction = steer_direction
                        break

            self.get_logger().info(f"[SSD] Obstacle Detected: {self.obstacle_detected}")

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
            self.get_logger().info(f"obstacle_detected: self.state {self.state} obstacle_detected_side {self.obstacle_detected_side}")
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
            msg.twist.linear.x = 0.05
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
