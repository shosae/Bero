import statistics
import time
import threading

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import Parameter, ParameterType
from rcl_interfaces.srv import SetParameters
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger


class AzOffsetCalibrator(Node):
    """/imu/data 기반 IMU baseline 보정 노드."""

    def __init__(self) -> None:
        super().__init__('az_offset_calibrator')

        # ---------------- Declare Parameter ----------------
        self.declare_parameter('target_node_name', '/floor_estimator')
        self.declare_parameter('param_server_timeout_sec', 2.0)
        self.declare_parameter('settling_time_sec', 0.5)
        self.declare_parameter('calibration_duration_sec', 0.3)
        self.declare_parameter('min_samples', 30)
        self.declare_parameter('safe_std_threshold_mps2', 0.04)
        self.declare_parameter('safe_range_min_mps2', 9.4)
        self.declare_parameter('safe_range_max_mps2', 10.0)

        # ---------------- Get Parameter ----------------
        self.target_node = self.get_parameter('target_node_name').value
        self.server_timeout = self.get_parameter('param_server_timeout_sec').value
        self.settling_time = self.get_parameter('settling_time_sec').value
        self.calib_dur = self.get_parameter('calibration_duration_sec').value
        self.min_samples = self.get_parameter('min_samples').value
        self.safe_std = self.get_parameter('safe_std_threshold_mps2').value
        self.safe_min = self.get_parameter('safe_range_min_mps2').value
        self.safe_max = self.get_parameter('safe_range_max_mps2').value

        # ---------------- Configuration ----------------
        self.set_param_service_name = f'{self.target_node}/set_parameters'

        # ---------------- Runtime Variables ----------------
        self._imu_buf = []
        self._is_calibrating = False

        # ---------------- Threading & Locks ----------------
        self._lock = threading.Lock()
        self._srv_cb_group = ReentrantCallbackGroup()

        # ---------------- ROS Communication ----------------
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_cb,
            qos_profile_sensor_data,
        )
        self.calib_srv = self.create_service(
            Trigger,
            'calibrate_az_offset',
            self.calibrate_cb,
            callback_group=self._srv_cb_group,
        )
        self.set_param_client = self.create_client(
            SetParameters,
            self.set_param_service_name,
            callback_group=self._srv_cb_group,
        )

        self.get_logger().info(f'IMU Offset Calibrator ready, target={self.target_node}')

    # ---------------- Callbacks ----------------
    def imu_cb(self, msg: Imu):
        if not self._is_calibrating:
            return
        with self._lock:
            if not self._is_calibrating:
                return
            self._imu_buf.append(msg.linear_acceleration.z)

    def calibrate_cb(self, _req, res):
        with self._lock:
            if self._is_calibrating:
                return self._make_log_and_res(res, 'warn', False, 'Calibration is already running')
            self._is_calibrating = True

        self.get_logger().info('Starting offset calibration')

        # 안정 대기
        time.sleep(self.settling_time)

        with self._lock:
            self._imu_buf.clear()

        # 측정
        time.sleep(self.calib_dur)

        with self._lock:
            self._is_calibrating = False
            data = self._imu_buf.copy()
            self._imu_buf.clear()

        # 개수 확인
        if len(data) < self.min_samples:
            msg = f'Not enough data: {len(data)} samples, skipping calibration'
            return self._make_log_and_res(res, 'warn', False, msg)

        # 표준편차 및 평균 계산
        std = statistics.pstdev(data)
        avg = statistics.fmean(data)
        self.get_logger().info(f'Measured avg={avg:.4f}')

        # 표준편차 확인
        if std > self.safe_std:
            msg = f'Unstable data std={std:.4f} > {self.safe_std:.4f}, skipping calibration'
            return self._make_log_and_res(res, 'warn', False, msg)

        # 평균 확인
        if not (self.safe_min < avg < self.safe_max):
            msg = f'Out of range avg={avg:.4f} not in [{self.safe_min:.4f}, {self.safe_max:.4f}], skipping calibration'  # noqa
            return self._make_log_and_res(res, 'warn', False, msg)

        # 파라미터 변경
        try:
            self._set_remote_parameter('baseline_mps2', avg)
        except Exception as e:
            msg = f'Failed to set parameter: {e}, skipping calibration'
            return self._make_log_and_res(res, 'error', False, msg)

        msg = f'Updated baseline_mps2 to {avg:.4f}'
        return self._make_log_and_res(res, 'info', True, msg)

    # ---------------- Helper methods ----------------

    def _set_remote_parameter(self, param_name: str, value: float) -> None:
        """Target 노드의 파라미터를 변경."""
        # 서버 확인
        if not self.set_param_client.wait_for_service(timeout_sec=self.server_timeout):
            raise RuntimeError(f'Service unavailable: {self.set_param_service_name}')

        # request 생성
        req = SetParameters.Request()
        param = Parameter()
        param.name = param_name
        param.value.type = ParameterType.PARAMETER_DOUBLE
        param.value.double_value = float(value)
        req.parameters = [param]

        future = self.set_param_client.call_async(req)
        start_t = time.monotonic()
        while rclpy.ok() and not future.done():
            if (time.monotonic() - start_t) >= self.server_timeout:
                raise RuntimeError('Timed out while waiting set_parameters response')
            time.sleep(0.01)

        result = future.result()
        if result is None or not result.results:
            raise RuntimeError('Empty response from set_parameters')
        if not result.results[0].successful:
            raise RuntimeError(f'Parameter update rejected: {result.results[0].reason}')

    def _make_log_and_res(self, res: Trigger.Response, log_level: str, success: bool, message: str) -> Trigger.Response:  # noqa
        if log_level == 'info':
            self.get_logger().info(message)
        elif log_level == 'warn':
            self.get_logger().warning(message)
        elif log_level == 'error':
            self.get_logger().error(message)
        res.success = success
        res.message = message
        return res


def main(args=None):
    rclpy.init(args=args)
    node = AzOffsetCalibrator()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
