#!/usr/bin/env python3
import os
import sys

import rclpy
from rclpy.executors import MultiThreadedExecutor

from bero_msgs.action import PressButton

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from mock_action_server import MockActionServer  # noqa: E402


class MockPressButtonServer(MockActionServer):
    """press_button action에 duration_sec 이후 success를 반환하는 Mock 서버."""

    def __init__(self):
        super().__init__(
            node_name='press_button_arm',
            action_type=PressButton,
            action_name='press_button',
            manual_status_topic='/press_button/mock_status',
            target_extractor=lambda req: req.button,
            expected_msg_generator=lambda target: f"pressed {target}",
            result_generator=self.create_result
        )

    def create_result(self, target, success, message):
        result = PressButton.Result()
        result.success = success
        result.message = f"Button {target}: {message}"
        return result


def main(args=None):
    rclpy.init(args=args)
    node = MockPressButtonServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
