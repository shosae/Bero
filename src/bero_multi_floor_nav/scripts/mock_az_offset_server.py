#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class MockAzOffsetServer(Node):
    """calibrate_az_offset service에 즉시 success를 반환하는 Mock 서버."""

    def __init__(self):
        super().__init__('mock_az_offset_calibrator')
        self.srv = self.create_service(Trigger, 'calibrate_az_offset', self.handle_calibrate)
        self.get_logger().info('[MockAzOffsetServer] Mock Az Offset Server ready')

    def handle_calibrate(self, request, response):
        self.get_logger().info('[MockAzOffsetServer] Calibration requested → success')
        response.success = True
        response.message = 'Mock calibration done'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MockAzOffsetServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
