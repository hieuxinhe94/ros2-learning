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

class SmartMove(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')
        self.get_logger().info("[SmartMove] SmartMove============================================")
        self.last_image_time = self.get_clock().now()
        
       
 
  
def main(args=None):
    rclpy.init(args=args)
    node = SmartMove()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
