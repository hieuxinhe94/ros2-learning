#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Header

class TwistToTwistStamped(Node):
    def __init__(self):
        super().__init__('twist_to_twiststamped')

        self.get_logger().info(f'[twist_to_twiststamped_node] ============= INIT ============================')

        self.declare_parameter('input_topic', '/cmd_vel')              # nơi Nav2AI xuất Twist
        self.declare_parameter('output_topic', '/cmd_vel_stamped')      # nơi robotcontroller cần TwistStamped
        self.declare_parameter('frame_id', 'base_link')

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        self.sub = self.create_subscription(Twist, input_topic, self.twist_callback, 10)
        self.pub = self.create_publisher(TwistStamped, output_topic, 10)

        self.get_logger().info(f'Forwarding {input_topic} (Twist) -> {output_topic} (TwistStamped)')

    def twist_callback(self, msg: Twist):
        self.get_logger().info(f'[twist_to_twiststamped_node] using custom node coverting (Twist) -> (TwistStamped).......')
        ts = TwistStamped()
        ts.header = Header()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.header.frame_id = self.frame_id
        ts.twist = msg
        self.pub.publish(ts)

def main(args=None):
    rclpy.init(args=args)
    node = TwistToTwistStamped()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

