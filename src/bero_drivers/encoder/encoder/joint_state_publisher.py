import math
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from sensor_msgs.msg import JointState
from pop.Encoder import Encoder

INT16_MAX = 32767
INT16_MIN = -32768
INT16_RANGE = 65536


def wrap_around(curr: int, prev: int) -> int:
    """
    Wrap around 보정 함수.

    int16 범위를 순환하는 Encoder counter에서 wrap-around가 발생할 때,
    그로 인해 계산되는 잘못된 변화량(Δtick)을 보정
    """
    delta = curr - prev
    if delta > INT16_MAX:
        delta -= INT16_RANGE
    elif delta < INT16_MIN:
        delta += INT16_RANGE
    return delta


class JointStatePublisherNode(Node):
    """
    Joint state Publisher.

    10Hz로 발행되는 Encoder 센서에서
    omni wheel 3개의 각도(position)와 각속도(velocity)를
    /joint_states 토픽으로 발행
    """

    def __init__(self):
        super().__init__("joint_state_publisher")

        # ---------------- Configuration ----------------
        self.cpr = 1950.0  # CPR(바퀴축 기준, 모든 바퀴 동일하게 1950.0)
        self.joint_names = [
            "wheel_joint_left",
            "wheel_joint_right",
            "wheel_joint_back",
        ]  # Joint 이름

        # ---------------- Runtime Variables ----------------
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)  # Steady time으로 정확하게
        self.last_callback_time = None  # dt 계산용
        self.last_ticks = (0, 0, 0)  # (L, R, B)
        self.wheel_positions = [0.0, 0.0, 0.0]  # 누적 각도 (rad)

        # ---------------- ROS Communication ----------------
        self.joint_state_pub = self.create_publisher(JointState, "/joint_states", 10)

        # ---------------- Device Initialization ----------------
        self.encoder = Encoder("can0", 500000)
        self.encoder.callback(self.encoder_cb, repeat=1)

        self.get_logger().info("Joint state publisher started")

    # encoder callback
    def encoder_cb(self, data: tuple[int, int, int, int]) -> None:
        """엔코더 센서로부터 메시지를 수신할 때마다 /joint_states를 계산하고 발행."""
        _, e1, e2, e3 = data
        now_steady = self.steady_clock.now()
        time_stamp = self.get_clock().now().to_msg()

        # 첫 콜백 초기화
        if self.last_callback_time is None:
            self.last_ticks = (e1, e2, e3)
            self.last_callback_time = now_steady
            return

        # 이전 콜백과의 시간 간격(s) 계산
        dt_duration = now_steady - self.last_callback_time
        dt_sec = dt_duration.nanoseconds / 1e9

        # 비정상적으로 짧은 주기 방지
        if dt_sec < 1e-6:
            return

        # Δtick (wrap around 보정)
        ticks = (e1, e2, e3)
        delta_ticks = [
            wrap_around(ticks[i], self.last_ticks[i]) for i in range(len(ticks))
        ]

        # 현재 ticks, 시각 저장 (다음 loop에서 참조)
        self.last_ticks = ticks
        self.last_callback_time = now_steady

        # 각도 및 각속도 계산
        # delta_angle (rad) = (delta_tick / CPR) * 2 * PI
        delta_angles = [(d_tick / self.cpr) * 2.0 * math.pi for d_tick in delta_ticks]

        # 각속도 (rad/s)
        velocities = [da / dt_sec for da in delta_angles]

        # 누적 각도
        for i in range(3):
            self.wheel_positions[i] += delta_angles[i]

        # # 디버그 로그 출력
        # self.get_logger().debug(f"velocities: {velocities}")
        # self.get_logger().debug(f"wheel_positions: {self.wheel_positions}")

        # 계산된 joint 정보 발행
        self.publish_joint(velocities, time_stamp)

    def publish_joint(self, velocities, time_stamp):
        msg = JointState()
        msg.header.stamp = time_stamp
        msg.name = self.joint_names
        msg.position = self.wheel_positions
        msg.velocity = velocities
        self.joint_state_pub.publish(msg)

    # Encoder thread 확실하게 종료
    def destroy_node(self):
        try:
            self.get_logger().info("Stopping Encoder Publisher")
            self.encoder.stop()
        except Exception as e:
            self.get_logger().error(f"Failed to stop the encoder during shutdown: {e}")
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
