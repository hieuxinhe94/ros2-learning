#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from builtin_interfaces.msg import Time

class PatrolNode(Node):
    def __init__(self):
        super().__init__("patrol_node")

        # TF Buffer + Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Biến theo dõi
        self.last_check_time = self.get_clock().now()
        self.last_position = None  # (x, y)
        self.check_interval = 1.0  # giây

        # Timer mỗi 0.5s để check
        self.create_timer(0.5, self.check_position)

        # Giả lập state
        self.state = "FORWARD"
        self.lag = False

    def get_current_position(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                target_frame="map",
                source_frame="base_link",
                time=rclpy.time.Time()
            )

            x = trans.transform.translation.x
            y = trans.transform.translation.y
            return (x, y)

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"Không lấy được TF: {str(e)}")
            return None

    def check_position(self):
        now = self.get_clock().now()

        if (now - self.last_check_time).nanoseconds < self.check_interval * 1e9:
            return  # chưa đến thời gian kiểm tra

        current_position = self.get_current_position()
        if current_position is None:
            return

        if self.last_position is not None:
            dx = current_position[0] - self.last_position[0]
            dy = current_position[1] - self.last_position[1]
            distance = math.sqrt(dx**2 + dy**2)

            self.get_logger().info(f"[TF] Distance moved = {distance:.4f} m")

            if distance < 0.01:
                self.get_logger().warn(f"[TF] Robot CÓ THỂ đang bị kẹt!")

                if self.state == "FORWARD":
                    self.get_logger().info("==> Robot không tiến được! Đổi sang STOP.")
                    self.state = "STOP"
                    self.lag = True
            else:
                self.lag = False

        self.last_position = current_position
        self.last_check_time = now

def main(args=None):
    rclpy.init(args=args)
    node = PatrolNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
