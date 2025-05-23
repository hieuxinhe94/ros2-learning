#!/usr/bin/env python3


import os
from ament_index_python import get_package_share_directory
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

import cv2
from cv_bridge import CvBridge
import numpy as np
import math
import time
from enum import Enum
import random


class PatrolState(Enum):
    FORWARD = 1
    TURN_LEFT = 2
    TURN_RIGHT = 3
    STOP = 4
    BACKWARD = 5
    TURN_LEFT_BACK = 6
    TURN_RIGHT_BACK = 7
 

class SemanticExplorer(Node):
    def __init__(self):
        super().__init__('SemanticExplorer')
        self.get_logger().info("[SemanticExplorer] SmartMove============================================")
        self.bridge = CvBridge()
        self.state = PatrolState.FORWARD
        self.state_timer = 0.0
        self.state_duration = 1.0

        # Load the MobileNet SSD model
        self.load_mobilenet_ssd()
        # Initialize the camera_subscription
        self.get_logger().info("[SemanticExplorer] Initialize the camera_subscription by /camera/image_raw topic of rgb camera")
        self.camera_subscription = self.create_subscription(Image,'camera',self.camera_image_callback,10)
        
        self.get_logger().info("[SemanticExplorer] Initialize the lidar2d_subscription by /scan topic of lidar2d")
        self.lidar2d_subscription = self.create_subscription(LaserScan,'/scan',self.lidar_scan_callback,10)
        
        self.timer = self.create_timer(1.0, self.processing_loop)  # 1Hz: mỗi 1s xử lý
        
        self.rgb_image = None
        self.scan_data = None
        self.goal_sent = False
        
        self.nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.cancel_bt_srv = self.create_client(Empty, '/bt_navigator/cancel')
        

    def processing_loop(self):
        if self.rgb_image is None or self.scan_data is None:
            return

        # Thực hiện xử lý semantic + gửi goal
        self.get_logger().info("[SemanticExplorer] Processing loop each second")
        self.process_semantic_detection(self.rgb_image, self.scan_data)

    def lidar_scan_callback(self, msg):
        self.scan_data = msg
        
    def camera_image_callback(self, msg):
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')        
        
        
    def process_semantic_detection(self, rgb_image, scan_data):
        if self.goal_sent:
            return

        frame = rgb_image
        (h, w) = frame.shape[:2]

        blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)),
                                     0.007843, (300, 300), 127.5)
        self.net.setInput(blob)
        detections = self.net.forward()

        for i in range(detections.shape[2]):
            confidence = detections[0, 0, i, 2]

            if confidence > 0.5:
                idx = int(detections[0, 0, i, 1])
                label = self.classes[idx]
                
                self.get_logger().info(f"[SemanticExplorer] 🛑 label: {label} - confidence: {confidence}")       
                if label == "person":  # 🎯 Target class
                    box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                    (startX, startY, endX, endY) = box.astype("int")
                    center_x = int((startX + endX) / 2)
                    angle = self.calculate_angle(center_x, w)
                    self.get_logger().info(f"[SemanticExplorer] 🛑 angle: {angle} - center_x: {center_x} - width: {w} - confidence: {confidence} - label: {label}")         
                    if scan_data:
                        distance = self.get_distance(angle)
                        self.get_logger().info(f"[SemanticExplorer] 🛑 distance: {distance} - angle: {angle} - center_x: {center_x} - width: {w} - confidence: {confidence} - label: {label}")
                        if distance > 0:
                            x = math.cos(angle) * distance
                            y = math.sin(angle) * distance
                            self.get_logger().info(f"[SemanticExplorer]  send_goal  x: {x} - y: {y} confidence: {confidence} - label: {label}")
                            self.send_goal(x, y)
                            break        


    def load_mobilenet_ssd(self):
        pkg_path = get_package_share_directory("first_robot")
        model_path = os.path.join(pkg_path, "ai_models", "MobileNetSSD_deploy.caffemodel")
        prototxt_path = os.path.join(pkg_path, "ai_models", "MobileNetSSD_deploy.prototxt")
        self.classes = ["background", "aeroplane", "bicycle", "bird", "boat",
                        "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
                        "dog", "horse", "motorbike", "person", "pottedplant", "sheep",
                        "sofa", "train", "tvmonitor"]

        self.net = cv2.dnn.readNetFromCaffe(prototxt_path, model_path)
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU) 
       

    def calculate_angle(self, pixel_x, width):
        fov = math.radians(60.0)
        center_offset = pixel_x - width / 2.0
        angle = center_offset / (width / 2.0) * (fov / 2.0)
        return angle
    
    def get_distance(self, angle_rad):
        if not self.scan_data:
            return -1
        index = int((angle_rad - self.scan_data.angle_min) / self.scan_data.angle_increment)
        if 0 <= index < len(self.scan_data.ranges):
            return self.scan_data.ranges[index]
        return -1

    def cancel_bt_nav(self):
        if not self.cancel_bt_srv.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("[SemanticExplorer] ⚠️ Cannot cancel Nav2 BT behavior, service not available.")
            return

        request = Empty.Request()
        future = self.cancel_bt_srv.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info("[SemanticExplorer] 🛑 Canceled current Nav2 behavior tree")         
  
def main(args=None):
    rclpy.init(args=args)
    node = SemanticExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
