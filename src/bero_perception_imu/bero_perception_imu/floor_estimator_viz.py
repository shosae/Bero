from collections import deque
import time

import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class FloorEstimatorVizNode(Node):
    """
    Floor Estimator 시각화 노드.

    - /floor_estimator/viz_data를 실시간으로 표시
    - current와 target이 불일치에서 일치로 변경되면 도착(getoff)으로 판정
    - 도착 판정 5초 뒤 그래프 초기화
    """

    def __init__(self) -> None:
        super().__init__('floor_estimator_viz')

        # ---------------- Declare Parameter ----------------
        self.declare_parameter('fs_hz', 40.0)
        self.declare_parameter('window_sec', 20.0)
        self.declare_parameter('plt_update_period', 0.2)
        self.declare_parameter('hold_sec', 5.0)
        self.declare_parameter('reset_gap_sec', 0.5)

        # ---------------- Get Parameter ----------------
        self.fs = self.get_parameter('fs_hz').value
        self.window = self.get_parameter('window_sec').value
        self.plt_update_period = self.get_parameter('plt_update_period').value
        self.hold_sec = self.get_parameter('hold_sec').value
        self.reset_gap_sec = self.get_parameter('reset_gap_sec').value

        # ---------------- Configuration ----------------
        buffer_len = int(self.fs * self.window * 1.5)

        # ---------------- Runtime Variables ----------------
        self._time_buf = deque(maxlen=buffer_len)
        self._accel_buf = deque(maxlen=buffer_len)
        self._vel_buf = deque(maxlen=buffer_len)

        self._current_floor = 0
        self._target_floor = 0
        self._last_getoff_time = None
        self._is_moving = False

        # ---------------- ROS Communication ----------------
        self.viz_sub = self.create_subscription(
            Float64MultiArray,
            '/floor_estimator/viz_data',
            self.viz_cb,
            10
        )
        self.timer = self.create_timer(self.plt_update_period, self.update_plt_cb)

        # ---------------- Device Initialization ----------------
        self._fig, (self._ax1, self._ax2) = plt.subplots(2, 1, figsize=(5, 3), sharex=True)
        self._fig.set_dpi(100)

        # 라인 객체 미리 생성
        self._line_accel, = self._ax1.plot([], [], linewidth=1.0)
        self._line_vel, = self._ax2.plot([], [], linewidth=1.0)

        # 텍스트 객체 미리 생성
        self._info_text = self._ax1.text(
            0.01, 0.9,
            "",
            transform=self._ax1.transAxes,
            fontsize=8,
            va='top',
            bbox=dict(facecolor='white', alpha=0.6, edgecolor='none')
        )

        self._init_axes()
        plt.ion()
        plt.tight_layout()
        plt.show(block=False)

    # ---------------- Callbacks ----------------
    def viz_cb(self, msg: Float64MultiArray):
        # [time, accel_z_lpf, velocity, current_floor, target_floor]
        data_len = len(msg.data)
        if data_len < 5:
            return

        stride = 5
        count = data_len // stride

        for i in range(count):
            idx = i * stride
            t = msg.data[idx]
            accel = msg.data[idx+1]
            vel = msg.data[idx+2]
            current_floor = int(msg.data[idx+3])
            target_floor = int(msg.data[idx+4])

            if self._time_buf and t < self._time_buf[-1] - self.reset_gap_sec:
                self._clear_buffers()

            self._time_buf.append(t)
            self._accel_buf.append(accel)
            self._vel_buf.append(vel)

            self._current_floor = current_floor
            self._target_floor = target_floor

            # 층이 다르면 이동 중, 이동 중이다가 같아지면 도착으로 처리
            if current_floor != target_floor:
                self._is_moving = True
            elif current_floor == target_floor and self._is_moving:
                self._last_getoff_time = time.time()
                self._is_moving = False

    def _init_axes(self) -> None:
        self._fig.suptitle("Floor Estimator", fontsize=10)
        self._ax1.set_ylabel("accel_z(m/s²)")
        self._ax2.set_ylabel("vel(m/s)")
        self._ax2.set_xlabel("time(s)")
        self._ax1.grid(True, alpha=0.3)
        self._ax2.grid(True, alpha=0.3)

    def _clear_buffers(self) -> None:
        self._time_buf.clear()
        self._accel_buf.clear()
        self._vel_buf.clear()
        self._current_floor = 0
        self._target_floor = 0
        self._is_moving = False
        self._last_getoff_time = None
        self._line_accel.set_data([], [])
        self._line_vel.set_data([], [])
        self._info_text.set_text("")
        self._fig.canvas.draw_idle()

    # ---------------- Plotting ----------------
    def update_plt_cb(self):
        if self._last_getoff_time is not None:
            elapsed = time.time() - self._last_getoff_time
            if elapsed > self.hold_sec:
                self._clear_buffers()
                self._last_getoff_time = None
                return

        if not self._time_buf:
            self._fig.canvas.flush_events()
            return

        t = list(self._time_buf)
        az = list(self._accel_buf)
        v = list(self._vel_buf)

        # 화면에 보이는 범위만 잘라서 그리기
        if t:
            tmax = t[-1]
            tmin = tmax - self.window

            start_idx = 0
            for i, val in enumerate(t):
                if val >= tmin:
                    start_idx = i
                    break

            t_slice = t[start_idx:]
            az_slice = az[start_idx:]
            v_slice = v[start_idx:]
        else:
            t_slice, az_slice, v_slice = [], [], []

        # 데이터 업데이트
        self._line_accel.set_data(t_slice, az_slice)
        self._line_vel.set_data(t_slice, v_slice)

        # 텍스트 업데이트
        self._info_text.set_text(
            f"current floor: {self._current_floor}\n"
            f"target floor: {self._target_floor}"
        )

        # 축 범위 조정
        if t:
            tmax = t[-1]
            tmin = max(0.0, tmax - self.window)
            self._ax1.set_xlim(tmin, tmax + 0.2)
            self._ax1.relim()
            self._ax1.autoscale_view(scalex=False, scaley=True)
            self._ax2.relim()
            self._ax2.autoscale_view(scalex=False, scaley=True)

        self._fig.canvas.draw_idle()
        self._fig.canvas.flush_events()


def main(args=None):
    rclpy.init(args=args)
    node = FloorEstimatorVizNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        plt.close('all')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
