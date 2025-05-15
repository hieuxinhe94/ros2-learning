#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from enum import Enum

class PatrolState(Enum):
    FORWARD = 1
    TURN_LEFT = 2
    TURN_RIGHT = 3

class PatrolMover(Node):
    def __init__(self):
        super().__init__('patrol_mover')
        self.publisher = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        self.state = PatrolState.FORWARD
        self.state_duration = 3.0
        self.state_timer = 0.0

        self.timer = self.create_timer(0.1, self.update_callback)

    def update_callback(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()

        self.state_timer += 0.1

        if self.state == PatrolState.FORWARD:
            msg.twist.linear.x = 0.4
            msg.twist.angular.z = 0.0
        elif self.state == PatrolState.TURN_LEFT:
            msg.twist.linear.x = 0.0
            msg.twist.angular.z = 0.6
        elif self.state == PatrolState.TURN_RIGHT:
            msg.twist.linear.x = 0.0
            msg.twist.angular.z = -0.6

        self.publisher.publish(msg)

        if self.state_timer > self.state_duration:
            self.state_timer = 0.0
            self.next_state()

    def next_state(self):
        if self.state == PatrolState.FORWARD:
            self.state = PatrolState.TURN_LEFT
            self.state_duration = 1.5
        elif self.state == PatrolState.TURN_LEFT:
            self.state = PatrolState.FORWARD
            self.state_duration = 3.0
        elif self.state == PatrolState.TURN_RIGHT:
            self.state = PatrolState.FORWARD
            self.state_duration = 3.0

def main(args=None):
    rclpy.init(args=args)
    node = PatrolMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()