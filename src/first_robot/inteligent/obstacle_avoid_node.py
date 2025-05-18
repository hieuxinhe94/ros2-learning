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
        self.subscription = self.create_subscription(Image, '/camera', self.image_callback, 10)

        self.bridge = CvBridge()
        self.state = PatrolState.FORWARD
        self.state_timer = 0.0
        self.state_duration = 1.0

        self.obstacle_detected = False
        self.timer = self.create_timer(0.2, self.update_callback) # 0.5 giây một lần
 
    def image_callback(self, msg):
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
        
    def update_callback(self):
        self.get_logger().info("update_callback")
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()

        self.state_timer += 1

        # Nếu phát hiện vật cản → chuyển sang rẽ trái/phải
        if self.obstacle_detected:
            self.get_logger().info(f"obstacle_detected: self.state {self.state}")
            if self.state not in [PatrolState.TURN_LEFT, PatrolState.TURN_RIGHT]:
                self.get_logger().info("Nếu phát hiện vật cản → chuyển sang rẽ trái/phải")
                self.state = random.choice([PatrolState.TURN_LEFT, PatrolState.TURN_RIGHT])
                self.state_timer = 0.0
                self.state_duration = 3.0
        else:
            # Khi rẽ xong mà không còn vật cản → đi thẳng tiếp
            if self.state in [PatrolState.TURN_LEFT, PatrolState.TURN_RIGHT] and self.state_timer >= self.state_duration:
                self.get_logger().info("Khi rẽ xong mà không còn vật cản → đi thẳng tiếp")
                self.state = PatrolState.FORWARD
                self.state_timer = 3.0
                self.state_duration = 5.0

        # Tạo lệnh tương ứng với trạng thái
        self.get_logger().info(f"PatrolState action: {self.state}")
        if self.state == PatrolState.FORWARD:
            msg.twist.linear.x = 0.15
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
