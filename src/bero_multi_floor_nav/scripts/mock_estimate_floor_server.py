#!/usr/bin/env python3
import os
import sys

import rclpy
from rclpy.executors import MultiThreadedExecutor

from bero_msgs.action import EstimateFloor

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from mock_action_server import MockActionServer  # noqa: E402


class MockEstimateFloorServer(MockActionServer):
    """estimate_floor action에 duration_sec 이후 success를 반환하는 Mock 서버."""

    def __init__(self):
        super().__init__(
            node_name='floor_estimator',
            action_type=EstimateFloor,
            action_name='estimate_floor',
            manual_status_topic='/floor_estimator/mock_status',
            target_extractor=lambda req: req.target_floor,
            expected_msg_generator=lambda target: f"getoff {target}",
            result_generator=self.create_result
        )

    def create_result(self, target, success, message):
        result = EstimateFloor.Result()
        result.success = success
        result.current_floor = target
        result.message = message
        return result


def main(args=None):
    rclpy.init(args=args)
    node = MockEstimateFloorServer()
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
