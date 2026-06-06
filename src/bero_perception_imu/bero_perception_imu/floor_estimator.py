import threading
from collections import deque

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionServer, CancelResponse, GoalResponse

from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray
from bero_msgs.action import EstimateFloor

from bero_perception_imu.floor_estimation_pipeline import (
    FloorEstimationPipeline,
    PipelineConfig,
    PipelineUpdate,
)
from bero_perception_imu.floor_tracker import TrackDecision


class FloorEstimatorNode(Node):
    def __init__(self) -> None:
        super().__init__('floor_estimator')

        # ---------------- Declare Parameter ----------------
        self.declare_parameter('min_floor', 0)
        self.declare_parameter('max_floor', 6)
        self.declare_parameter('h_L1_m', 5.0)
        self.declare_parameter('h_12_m', 5.0)
        self.declare_parameter('h_norm_m', 3.0)
        self.declare_parameter('baseline_mps2', 9.84)
        self.declare_parameter('acc_clip_mps2_abs', 1.0)
        self.declare_parameter('raw_fs_hz', 143.0)
        self.declare_parameter('fs_hz', 40.0)
        self.declare_parameter('fc_hz', 2.0)
        self.declare_parameter('filt_order', 4)
        self.declare_parameter('filtfilt_order', 4)
        self.declare_parameter('min_seg_duration_sec', 2.0)
        self.declare_parameter('min_seg_height_m', 1.0)
        self.declare_parameter('pre_buf_lpf_delay_sec', 0.3)
        self.declare_parameter('pre_buf_ringing_trim_sec', 0.375)
        self.declare_parameter('acc_high_mps2_abs', 0.08)
        self.declare_parameter('acc_low_mps2_abs', 0.05)
        self.declare_parameter('std_low_mps2', 0.08)
        self.declare_parameter('stop_speed_mps_abs', 0.5)
        self.declare_parameter('stop_duration_sec', 0.5)
        self.declare_parameter('zupt_window_sec', 0.2)

        # 내부 파라미터
        self.declare_parameter('imu_queue_maxlen', 50)
        self.declare_parameter('action_server_timeout_sec', 300.0)
        self.declare_parameter('viz_pub_period', 0.1)

        # ---------------- Get Parameter ----------------
        self.min_floor = self.get_parameter('min_floor').value
        self.max_floor = self.get_parameter('max_floor').value
        self.h_L1 = self.get_parameter('h_L1_m').value
        self.h_12 = self.get_parameter('h_12_m').value
        self.h_norm = self.get_parameter('h_norm_m').value
        self.baseline = self.get_parameter('baseline_mps2').value
        self.acc_clip_abs = self.get_parameter('acc_clip_mps2_abs').value
        self.raw_fs = self.get_parameter('raw_fs_hz').value
        self.fs = self.get_parameter('fs_hz').value
        self.fc = self.get_parameter('fc_hz').value
        self.filt_order = self.get_parameter('filt_order').value
        self.filtfilt_order = self.get_parameter('filtfilt_order').value
        self.min_seg_dur = self.get_parameter('min_seg_duration_sec').value
        self.min_seg_h = self.get_parameter('min_seg_height_m').value
        self.pre_buf_lpf_delay_sec = self.get_parameter('pre_buf_lpf_delay_sec').value
        self.pre_buf_ringing_trim_sec = self.get_parameter('pre_buf_ringing_trim_sec').value
        self.pre_buf_sec = self.pre_buf_lpf_delay_sec + self.pre_buf_ringing_trim_sec
        self.pre_buf_trim_sec = self.pre_buf_ringing_trim_sec
        self.acc_high = self.get_parameter('acc_high_mps2_abs').value
        self.acc_low = self.get_parameter('acc_low_mps2_abs').value
        self.std_low = self.get_parameter('std_low_mps2').value
        self.stop_speed_threshold = self.get_parameter('stop_speed_mps_abs').value
        self.stop_dur = self.get_parameter('stop_duration_sec').value
        self.zupt_window = self.get_parameter('zupt_window_sec').value
        self.imu_queue_maxlen = self.get_parameter('imu_queue_maxlen').value
        self.server_timeout = self.get_parameter('action_server_timeout_sec').value
        self.viz_pub_period = self.get_parameter('viz_pub_period').value

        # ---------------- Configuration ----------------
        self.floors = list(range(self.min_floor, self.max_floor + 1))

        # ---------------- Runtime Variables ----------------
        self._is_goal_active = False
        self._is_estimating_active = False
        self._target_floor = None
        self._loop_rate = None
        self._imu_buf = deque(maxlen=self.imu_queue_maxlen)
        self._viz_buf: list[float] = []
        pipeline_config = PipelineConfig(
            floors=self.floors,
            h_L1=self.h_L1,
            h_12=self.h_12,
            h_norm=self.h_norm,
            acc_clip_abs=self.acc_clip_abs,
            raw_fs=self.raw_fs,
            fs=self.fs,
            fc=self.fc,
            filt_order=self.filt_order,
            filtfilt_order=self.filtfilt_order,
            min_seg_dur=self.min_seg_dur,
            min_seg_h=self.min_seg_h,
            pre_buf_sec=self.pre_buf_sec,
            pre_buf_trim_sec=self.pre_buf_trim_sec,
            acc_high=self.acc_high,
            acc_low=self.acc_low,
            std_low=self.std_low,
            stop_speed_threshold=self.stop_speed_threshold,
            stop_dur=self.stop_dur,
            zupt_window=self.zupt_window,
        )
        self._pipeline = FloorEstimationPipeline(pipeline_config)

        # ---------------- Threading & Locks ----------------
        self._estimate_floor_cb_group = ReentrantCallbackGroup()
        self._state_lock = threading.Lock()
        self._imu_buf_lock = threading.Lock()
        self._viz_lock = threading.Lock()

        # ---------------- ROS Communication ----------------
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_cb, qos_profile_sensor_data)  # noqa
        self.action_server = ActionServer(
            self,
            EstimateFloor,
            'estimate_floor',
            goal_callback=self.goal_cb,
            execute_callback=self.execute_cb,
            cancel_callback=self.cancel_cb,
            callback_group=self._estimate_floor_cb_group,
        )
        self.viz_timer = self.create_timer(self.viz_pub_period, self.viz_timer_cb)
        self.viz_pub = self.create_publisher(Float64MultiArray, '/floor_estimator/viz_data', 10)

        self.get_logger().info(
            f"Floor Estimator ready \n"
            f"(baseline={self.baseline:.4f}, fs={self.fs:.1f}Hz, raw_fs={self.raw_fs:.1f}Hz, fc={self.fc:.1f}Hz, "  # noqa
            f"acc_high={self.acc_high}, acc_low={self.acc_low}, std_low={self.std_low})"
        )

    # ---------------- Sub & Timer Callbacks ----------------

    def imu_cb(self, msg: Imu):
        if not self._is_estimating():
            return

        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        az = msg.linear_acceleration.z

        with self._imu_buf_lock:
            if not self._is_estimating():
                return
            self._imu_buf.append((stamp_sec, az))

    def viz_timer_cb(self):
        if not self._has_viz_subscribers():
            with self._viz_lock:
                if self._viz_buf:
                    self._viz_buf.clear()
            return

        with self._viz_lock:
            if not self._viz_buf:
                return
            viz_data = list(self._viz_buf)
            self._viz_buf.clear()

        msg = Float64MultiArray()
        msg.data = viz_data
        self.viz_pub.publish(msg)

    # ---------------- Action Server Callbacks ----------------

    def goal_cb(self, goal_request):
        start_floor = goal_request.start_floor
        target_floor = goal_request.target_floor

        with self._state_lock:
            if self._is_goal_active:
                self.get_logger().error("Rejecting goal: floor estimation is already running")
                return GoalResponse.REJECT

            if start_floor not in self.floors or target_floor not in self.floors:
                self.get_logger().error(f"Rejecting goal: floor must be within [{self.min_floor}, {self.max_floor}]")  # noqa
                return GoalResponse.REJECT

            self._is_goal_active = True

        self.get_logger().info(f'Action goal: {start_floor} -> {target_floor}')
        return GoalResponse.ACCEPT

    def execute_cb(self, goal_handle):
        try:
            self.get_logger().info("Received floor estimation request")
            self._init_goal_context(goal_handle)

            loop_result = self._estimation_loop(goal_handle, timeout=self.server_timeout)
            if loop_result == 'canceled':
                return self._handle_cancel(goal_handle)
            if loop_result == 'timeout':
                return self._handle_timeout(goal_handle)
            return self._handle_success(goal_handle)

        finally:
            self._set_estimating(False)
            self._destroy_loop_rate()
            with self._state_lock:
                self._is_goal_active = False

    def cancel_cb(self, goal_handle):
        self.get_logger().info("Cancel request accepted")
        return CancelResponse.ACCEPT

    # ---------------- Estimation Loop ----------------

    def _estimation_loop(self, goal_handle, timeout: float) -> str:
        start_t = self._get_now_sec()

        while True:
            if goal_handle.is_cancel_requested:
                return 'canceled'

            now = self._get_now_sec()
            if (now - start_t) >= timeout:
                return 'timeout'

            drained_samples = self._drain_imu_samples()
            for stamp_sec, az in drained_samples:
                self._pipeline.push_sample(stamp_sec, az)

            for update in self._pipeline.tick():
                reached_goal = self._handle_pipeline_update(update)
                if reached_goal:
                    self._append_viz_data()
                    self.get_logger().info("Floor estimation completed")
                    return 'goal_reached'

            self._append_viz_data()
            self._loop_rate.sleep()

    def _drain_imu_samples(self) -> list[tuple[float, float]]:
        with self._imu_buf_lock:
            if not self._imu_buf:
                return []
            drained_samples = list(self._imu_buf)
            self._imu_buf.clear()
            return drained_samples

    def _handle_pipeline_update(self, update: PipelineUpdate) -> bool:
        if update.track_result is None:
            self.get_logger().warn(
                f"Segment Ignored: dur={update.seg_result.dur:.2f}s "
                f"dh={update.seg_result.dh:.3f}m < min_height={self.min_seg_h}m"
            )
            return False

        track_result = update.track_result

        self.get_logger().info(
            f"Arrived at {track_result.current_floor}/{self._target_floor} | "
            f"Dur={update.seg_result.dur:.2f}s | Moved={update.seg_result.dh:+.3f}m | "
            f"Diff={track_result.h_diff:.3f}m | Decision={track_result.decision.value}"
        )

        return track_result.decision == TrackDecision.GETOFF

    # ---------------- execute_cb helpers ----------------

    def _init_goal_context(self, goal_handle) -> None:
        self._set_estimating(False)

        req = goal_handle.request
        start_floor = req.start_floor
        self._target_floor = req.target_floor

        # baseline_mps2 재설정
        self.baseline = self.get_parameter('baseline_mps2').value
        self.get_logger().info(f"Using baseline_mps2 = {self.baseline:.4f}")

        with self._imu_buf_lock:
            self._imu_buf.clear()
        with self._viz_lock:
            self._viz_buf.clear()

        self._pipeline.initialize_goal(
            baseline=self.baseline,
            start_floor=start_floor,
            target_floor=self._target_floor,
        )
        self._loop_rate = self.create_rate(self.fs, self._steady_clock)
        self._set_estimating(True)

    def _handle_cancel(self, goal_handle) -> EstimateFloor.Result:
        goal_handle.canceled()
        return self._make_result(False, 'Canceled')

    def _handle_timeout(self, goal_handle) -> EstimateFloor.Result:
        goal_handle.abort()
        return self._make_result(False, 'Timeout')

    def _handle_success(self, goal_handle) -> EstimateFloor.Result:
        goal_handle.succeed()
        return self._make_result(True, f"Arrived at floor {self._target_floor}")

    # ---------------- Helper methods ----------------

    def _get_now_sec(self) -> float:
        return self._steady_clock.now().nanoseconds * 1e-9

    def _is_estimating(self) -> bool:
        with self._state_lock:
            return self._is_estimating_active

    def _set_estimating(self, estimating: bool) -> None:
        with self._state_lock:
            self._is_estimating_active = estimating

    def _destroy_loop_rate(self) -> None:
        if self._loop_rate is None:
            return

        self.destroy_rate(self._loop_rate)
        self._loop_rate = None

    def _make_result(self, success: bool, message: str) -> EstimateFloor.Result:
        res = EstimateFloor.Result()
        res.success = success
        res.message = message
        res.current_floor = self._pipeline.get_current_floor()
        return res

    # ---------------- Visualization helpers ----------------

    def _has_viz_subscribers(self) -> bool:
        return self.viz_pub.get_subscription_count() > 0

    def _append_viz_data(self) -> None:
        if not self._has_viz_subscribers():
            return

        viz = self._pipeline.get_viz_data()
        if viz is None:
            return

        t, az_lpf, vel = viz
        current_floor = float(self._pipeline.get_current_floor())
        target_floor = float(self._target_floor)

        with self._viz_lock:
            self._viz_buf.extend([
                t,
                az_lpf,
                vel,
                current_floor,
                target_floor,
            ])


def main(args=None):
    rclpy.init(args=args)
    node = FloorEstimatorNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
