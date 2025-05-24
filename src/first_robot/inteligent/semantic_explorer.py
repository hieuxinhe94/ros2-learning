#!/usr/bin/env python3


import os
from ament_index_python import get_package_share_directory

from action_scheduler import ActionScheduler
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
from geometry_msgs.msg import  TwistStamped
import datetime
import copy

class MoveTemplate(Enum):
    FORWARD = 1
    TURN_LEFT = 2
    TURN_RIGHT = 3
    STOP = 4
    BACKWARD = 5
    TURN_LEFT_BACK = 6
    TURN_RIGHT_BACK = 7
    TURN_LEFT_AND_FORWARD = 10
    TURN_RIGHT_AND_FORWARD = 11
    TURN_LEFT_BACK_AND_FORWARD = 20
    TURN_RIGHT_BACK_AND_FORWARD = 21
 

class SemanticExplorer(Node):
    def __init__(self):
        super().__init__('SemanticExplorerNode')
        self.get_logger().info("[SemanticExplorer] SmartMove============================================")
        self.bridge = CvBridge()
        self.state = MoveTemplate.FORWARD
        self.state_timer = 0.0
        self.state_duration = 1.0

        # Load the MobileNet SSD model
        self.load_mobilenet_ssd()
        # Initialize the camera_subscription
        self.get_logger().info("[SemanticExplorer] Initialize the camera_subscription by /camera/image_raw topic of rgb camera")
        self.camera_subscription = self.create_subscription(Image,'camera',self.camera_image_callback,10)
        
        self.get_logger().info("[SemanticExplorer] Initialize the lidar2d_subscription by /scan topic of lidar2d")
        self.lidar2d_subscription = self.create_subscription(LaserScan,'/scan',self.lidar_scan_callback,10)
        
        self.nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.cancel_bt_srv = self.create_client(Empty, '/bt_navigator/cancel')
        
        self.rgb_image = None
        self.scan_data = None
        self.goal_sent = False
        self.last_process_time = self.get_clock().now().nanoseconds
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel_stamped', 10)
        self.state_start_time = self.get_clock().now()
        self.state_duration = 0.0
        self.last_state_duration = 0.0
        self.state = MoveTemplate.STOP
        self.last_state = MoveTemplate.STOP

        self.scheduler_node = ActionScheduler(self)
        self.scheduler_node.run_scheduler()
        
        #
        self.timer = self.create_timer(1.0, self.processing_loop)  # 1Hz: mỗi 1s xử lý

    def processing_loop(self):
        
        # Tính thời gian giữa 2 lần xử lý
        now = self.get_clock().now().nanoseconds
        if now - self.last_process_time < 1e9:  # 1s = 1e9 nanoseconds
            return
        self.last_process_time = now
        
        self.get_logger().info("[SemanticExplorer] ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^")
        self.get_logger().info("[SemanticExplorer] Processing loop each second")
        
        if self.rgb_image is None or self.scan_data is None:
            return

        if self.goal_sent:
            return  # Đang tới mục tiêu

        found_target = self.process_semantic_detection(self.rgb_image, self.scan_data)
        if not found_target:
            self.get_logger().info("[SemanticExplorer] random exploration")
            self.random_exploration()
        else: 
            self.get_logger().info("[SemanticExplorer] go to target")            
        # Thực hiện xử lý semantic + gửi goal
        

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
  
    def random_exploration(self):
        min_distance = min(self.scan_data.ranges)
        angle_range = len(self.scan_data.ranges)
        front_index = angle_range // 2
        front_distance = self.scan_data.ranges[front_index]

        self.get_logger().info(f"[SemanticExplorer] front_distance = {front_distance}")           
        if math.isinf(front_distance) or front_distance > 0.7:
            self.get_logger().info(f"[SemanticExplorer] {self.get_current_time()} Tiếp tục đi thẳng 1s")
            self.state = random.choice([MoveTemplate.FORWARD, MoveTemplate.STOP])
            self.get_logger().info(f"[SemanticExplorer]  {self.get_current_time()} random di chuyển {self.state.name} ")           
            self.state_duration = 2.0 
            
            # test
            # node = ActionScheduler(self)
            # msg = TwistStamped()
            # msg.header.stamp = self.get_clock().now().to_msg()      
            # # Tiến 1s
            # msg.twist.linear.x = 0.1
            # msg.twist.angular.z = 0.0
            # self.get_logger().info(f"[SemanticExplorer] {self.get_current_time()} Tiến lên 1s sau đó dừng lại")           
            # node.set_timeout(0.0, copy.deepcopy(msg), note="Tiến lên 1s sau đó dừng lại")
            # Sau 1s dừng lại
            # msg = TwistStamped()
            # msg.header.stamp = self.get_clock().now().to_msg()            
            # msg.twist.linear.x = 0.10
            # msg.twist.angular.z = 0.0
            # node.set_timeout(1.0, copy.deepcopy(msg), note="")            

        else:
            # Nếu bị chắn phía trước
            self.get_logger().info(f"[SemanticExplorer] {self.get_current_time()} Nếu bị chắn phía trước front_distance = {front_distance}")
            check_time_now = self.get_clock().now().nanoseconds           
            if front_distance < 0.3:
                if self.state in [MoveTemplate.TURN_LEFT_BACK_AND_FORWARD, MoveTemplate.TURN_RIGHT_BACK_AND_FORWARD] and check_time_now - self.state_start_time.nanoseconds >= 1e9:
                    self.get_logger().info(f"[SemanticExplorer] {self.get_current_time()} State hiện tại {self.state} còn {check_time_now - self.state_start_time.nanoseconds}s đang khớp với dự định 1, bỏ qua = {front_distance}")           
                    return
                self.state = random.choice([MoveTemplate.TURN_LEFT_BACK_AND_FORWARD, MoveTemplate.TURN_RIGHT_BACK_AND_FORWARD])
                self.state_duration = 10.0
            else:
                if self.state in [MoveTemplate.TURN_LEFT_AND_FORWARD, MoveTemplate.TURN_RIGHT_AND_FORWARD] and check_time_now - self.state_start_time.nanoseconds >= 1e9:
                    self.get_logger().info(f"[SemanticExplorer] {self.get_current_time()} State hiện tại {self.state} còn {check_time_now - self.state_start_time.nanoseconds}s  đang khớp với dự định 2, bỏ qua = {front_distance}")                               
                    return                
                self.state = random.choice([MoveTemplate.TURN_LEFT_AND_FORWARD, MoveTemplate.TURN_RIGHT_AND_FORWARD])
                self.state_duration = 7.0

        self.execute_movement(self.state, self.state_duration)  
        
    def execute_movement(self, state, duration):
        self.get_logger().info(f"[Explorer] {self.get_current_time()} Đang yêu cầu 🤖 State: {state.name}, Duration: {duration:.2f}s")

        # đếm thời gian hành động cũ đã làm xong chưa
        now = self.get_clock().now()
        elapsed = (now - self.state_start_time).nanoseconds / 1e9  # giây
        if elapsed < self.last_state_duration:
            self.get_logger().info(f"[Explorer] {self.get_current_time()} 🤖 Hành động cũ {self.last_state.name} trong {self.last_state_duration}s chưa xong, bỏ qua yêu cầu di chuyển")
            return
        
        self.state_start_time = now  # Bắt đầu lại đồng hồ cho hành động mới
        self.last_state_duration = self.state_duration 
        self.last_state = self.state
        
        self.get_logger().info(f"[Explorer] {self.get_current_time()} Thực hiện hành động {state.name}, Duration: {duration:.2f}s")
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()

        self.state_timer += 1                
        if self.state == MoveTemplate.FORWARD:
            msg.twist.linear.x = 0.1
            msg.twist.angular.z = 0.0
        elif self.state == MoveTemplate.TURN_LEFT:
            msg.twist.linear.x = 0.0
            msg.twist.angular.z = 0.4
        elif self.state == MoveTemplate.TURN_RIGHT:
            msg.twist.linear.x = 0.0
            msg.twist.angular.z = -0.4
        elif self.state == MoveTemplate.STOP:
            msg.twist.linear.x = 0.0
            msg.twist.angular.z = 0.0
            
        elif state == MoveTemplate.TURN_LEFT_BACK:
            
            msg = TwistStamped()
            # Lùi
            msg.twist.linear.x = -0.1
            msg.twist.angular.z = 0.0
            self.scheduler_node.set_timeout(0.0, copy.deepcopy(msg), note="Lùi 2s")
            #time = angle / speed = π/2 / 0.4 ≈ 3.93s ≈ 4s quay tại chỗ mới đạt ~90°
            # Quay trái 5s
            msg.twist.linear.x = 0.0
            msg.twist.angular.z = 0.4
            self.scheduler_node.set_timeout(2.0, copy.deepcopy(msg), note="Quay trái 5s")
            # Đi thẳng            
            msg.twist.linear.x = 0.1
            msg.twist.angular.z = 0.4
            self.scheduler_node.set_timeout(7.0, copy.deepcopy(msg), note="Đi thẳng 3s")
           

        elif state == MoveTemplate.TURN_RIGHT_BACK:
            
            msg = TwistStamped()
            # Lùi
            msg.twist.linear.x = -0.1
            msg.twist.angular.z = 0.0
            self.scheduler_node.set_timeout(0.0, copy.deepcopy(msg), note="Lùi 2s")
            # Quay phải
            msg.twist.linear.x = 0.0
            msg.twist.angular.z = -0.4
            self.scheduler_node.set_timeout(2.0, copy.deepcopy(msg), note="Quay phải 5s")
            # Đi thẳng            
            msg.twist.linear.x = 0.1
            msg.twist.angular.z = 0.4
            self.scheduler_node.set_timeout(7.0, copy.deepcopy(msg), note="Đi thẳng 3s")

        elif state == MoveTemplate.TURN_LEFT_AND_FORWARD:
            
            msg = TwistStamped()
            # Quay trái
            msg.twist.linear.x = 0.0
            msg.twist.angular.z = 0.4
            self.scheduler_node.set_timeout(0.0, copy.deepcopy(msg), note="Quay trái 5s")
            # Đi thẳng
            msg.twist.linear.x = 0.1
            msg.twist.angular.z = 0.0
            self.scheduler_node.set_timeout(5.0, copy.deepcopy(msg), note="Đi thẳng 3s")
 

        elif state == MoveTemplate.TURN_RIGHT_AND_FORWARD:
            
            msg = TwistStamped()
            # Quay phải
            msg.twist.linear.x = 0.0
            msg.twist.angular.z = -0.4
            self.scheduler_node.set_timeout(0.0, copy.deepcopy(msg), note="Quay phải 5s")
            # Đi thẳng
            msg.twist.linear.x = 0.1
            msg.twist.angular.z = 0.0
            self.scheduler_node.set_timeout(5.0, copy.deepcopy(msg), note="Đi thẳng 3s")

        elif state == MoveTemplate.TURN_LEFT_BACK_AND_FORWARD:
            
            msg = TwistStamped()
            # Lùi
            msg.twist.linear.x = -0.1
            msg.twist.angular.z = 0.0
            self.scheduler_node.set_timeout(0.0, copy.deepcopy(msg), note="Lùi 2s")
            # Quay trái
            msg.twist.linear.x = 0.0
            msg.twist.angular.z = 0.5
            self.scheduler_node.set_timeout(2.0, copy.deepcopy(msg), note="Quay trái 5s")
            # Đi thẳng
            msg.twist.linear.x = 0.1
            msg.twist.angular.z = 0.0
            self.scheduler_node.set_timeout(7.0, copy.deepcopy(msg), note="Đi thẳng 3s")

        elif state == MoveTemplate.TURN_RIGHT_BACK_AND_FORWARD:
            
            msg = TwistStamped()
            # Lùi
            msg.twist.linear.x = -0.1
            msg.twist.angular.z = 0.0
            self.scheduler_node.set_timeout(0.0, copy.deepcopy(msg), note="Lùi 2s")
            # Quay phải
            msg.twist.linear.x = 0.0
            msg.twist.angular.z = -0.5
            self.scheduler_node.set_timeout(2.0, copy.deepcopy(msg), note="Quay phải 5s")
            # Đi thẳng
            msg.twist.linear.x = 0.1
            msg.twist.angular.z = 0.0
            self.scheduler_node.set_timeout(7.0, copy.deepcopy(msg),    note="Đi thẳng 3s") 
                               
        elif state == MoveTemplate.STOP:
            msg.twist.linear.x = 0.0
            msg.twist.angular.z = 0.0  
                                      
        self.get_logger().info(f"execute_movement direction action: {self.state}")
        self.pub.publish(msg)
        pass

    def get_current_time(self): 
        now_ros = self.get_clock().now()
        now_ns = now_ros.nanoseconds  # Thời gian tính từ epoch (1970) tính bằng nano giây
        now_dt = datetime.datetime.fromtimestamp(now_ns / 1e9)

        # Hiển thị dạng dễ đọc
        formatted_time = now_dt.strftime("%H:%M:%S.%f")[:-3]  # bỏ bớt 3 số cuối (từ micro thành mili)
        return formatted_time
  
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
