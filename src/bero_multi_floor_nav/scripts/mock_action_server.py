#!/usr/bin/env python3
import threading

from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_msgs.msg import String


class MockActionServer(Node):
    """duration_sec 이후 및 수동 action 성공을 반환하는 mock action base 서버."""

    def __init__(
        self,
        node_name: str,
        action_type,
        action_name: str,
        manual_status_topic: str,
        target_extractor,
        expected_msg_generator,
        result_generator
    ):
        super().__init__(node_name)

        # ---------------- Declare Parameter ----------------
        self.declare_parameter('duration_sec', 3.0)

        # ---------------- Get Parameter ----------------
        self.duration_sec = self.get_parameter('duration_sec').value

        # ---------------- Runtime Variables ----------------
        self._active_lock = threading.Lock()
        self._is_active = False
        self._manual_triggered = False
        self._current_target = None

        # ---------------- Threading & Locks ----------------
        self.callback_group = ReentrantCallbackGroup()

        # ---------------- ROS Communication ----------------
        self._target_extractor = target_extractor
        self._expected_msg_generator = expected_msg_generator
        self._result_generator = result_generator

        self._action_server = ActionServer(
            self,
            action_type,
            action_name,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group
        )
        self.manual_status_sub = self.create_subscription(
            String,
            manual_status_topic,
            self.manual_status_cb,
            10,
            callback_group=self.callback_group
        )

        self.get_logger().info(f'Ready (duration_sec={self.duration_sec})')

    def manual_status_cb(self, msg: String):
        """외부에서 action 수동 종료"""
        with self._active_lock:
            if not self._is_active or self._current_target is None:
                return
            expected = self._expected_msg_generator(self._current_target)
            if msg.data == expected:
                self.get_logger().info(f"Manual trigger matched: {msg.data}")
                self._manual_triggered = True
            else:
                self.get_logger().warn(f"Unexpected message: '{msg.data}' (expected: '{expected}')")

    def goal_callback(self, goal_request):
        with self._active_lock:
            if self._is_active:
                self.get_logger().warn('Rejecting goal: action is already running')
                return GoalResponse.REJECT
            self._is_active = True
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Action canceled')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        try:
            target = self._target_extractor(goal_handle.request)
            self.get_logger().info(f'Received request with target: {target}')

            with self._active_lock:
                self._manual_triggered = False
                self._current_target = target

            check_rate = self.create_rate(10)
            start_t = self.get_clock().now().nanoseconds * 1e-9
            while (self.get_clock().now().nanoseconds * 1e-9 - start_t) < self.duration_sec:
                if goal_handle.is_cancel_requested:
                    return self._result_generator(target, success=False, message="Action canceled")
                with self._active_lock:
                    if self._manual_triggered:
                        self.get_logger().info('Completed early due to manual trigger')
                        break
                check_rate.sleep()

            self.get_logger().info('Action completed')
            goal_handle.succeed()
            return self._result_generator(target, success=True, message="Completed successfully")
        finally:
            with self._active_lock:
                self._is_active = False
                self._current_target = None
