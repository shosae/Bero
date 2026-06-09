#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, String


class MockDoorStatusServer(Node):
    """door_monitoring_enable topic을 받아 duration_sec 이후 opened 상태를 발행하는 Mock 서버."""

    def __init__(self):
        super().__init__('mock_door_status_server')

        self.declare_parameter('duration_sec', 3.0)
        self.duration_sec = self.get_parameter('duration_sec').value
        self.open_timer = None

        self.status_pub = self.create_publisher(String, '/elevator/door_status', 10)
        self.monitor_enable_sub = self.create_subscription(
            Bool,
            '/elevator/door_monitoring_enable',
            self.monitor_enable_cb,
            10,
        )
        self.get_logger().info('Mock door status server ready')

    def monitor_enable_cb(self, msg: Bool):
        """Enable 신호를 받아서 duration_sec 뒤 opened 상태를 발행한다."""
        if msg.data:
            if self.open_timer is not None:
                self.destroy_timer(self.open_timer)
            self.open_timer = self.create_timer(self.duration_sec, self.publish_opened)
            self.get_logger().info('Door monitoring enabled')
        else:
            if self.open_timer is not None:
                self.destroy_timer(self.open_timer)
                self.open_timer = None
            self.get_logger().info('Door monitoring disabled')

    def publish_opened(self):
        """Publish opened state."""
        if self.open_timer is not None:
            self.destroy_timer(self.open_timer)
            self.open_timer = None
        status_msg = String()
        status_msg.data = 'opened'
        self.status_pub.publish(status_msg)
        self.get_logger().info('Published opened state')


def main(args=None):
    rclpy.init(args=args)
    node = MockDoorStatusServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
