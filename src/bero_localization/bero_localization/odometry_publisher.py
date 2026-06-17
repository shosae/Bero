import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster


def yaw_to_quat(yaw: float) -> Quaternion:
    """yaw로부터 Quaternion을 계산."""
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw * 0.5), w=math.cos(yaw * 0.5))


class OdometryPublisherNode(Node):
    """
    Wheel Odometry Publisher.

    /joint_states 토픽을 구독하여
    omni wheel 3개의 joint position과 velocity로부터 odometry를 계산하고
    publish_tf 파라미터가 True일 경우 odom->base_link TF 발행
    """

    def __init__(self):
        super().__init__("odometry_publisher")

        # ---------------- Declare Parameter ----------------
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("odom_topic", "/wheel/odom")

        # EKF 등 TF를 외부에서 발행할 경우 False로 설정
        self.declare_parameter("publish_tf", False)

        # 로봇 기구학 파라미터
        self.declare_parameter("wheel_radius_m", 0.041)
        self.declare_parameter("wheel_to_center_m", 0.135)
        self.declare_parameter("wheel_angles_deg", [60.0, 300.0, 180.0])
        self.declare_parameter("joint_names", ["wheel_joint_left", "wheel_joint_right", "wheel_joint_back"])  # noqa

        # pose, twist Covariance
        self.declare_parameter("pose_cov_diag", [1e-1, 1e-1, 1e6, 1e6, 1e6, 2e-3])
        self.declare_parameter("twist_cov_diag", [1e-2, 1e-2, 1e6, 1e6, 1e6, 1e-3])

        # ---------------- Get Parameter ----------------
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.publish_tf = self.get_parameter("publish_tf").value

        self.r = self.get_parameter("wheel_radius_m").value
        self.R = self.get_parameter("wheel_to_center_m").value
        self.joint_names = self.get_parameter("joint_names").value
        self.pose_cov_diag = self.get_parameter("pose_cov_diag").value
        self.twist_cov_diag = self.get_parameter("twist_cov_diag").value

        angles_rad = [math.radians(a) for a in self.get_parameter("wheel_angles_deg").value]

        # ---------------- Runtime Variables ----------------
        self.sin = [math.sin(a) for a in angles_rad]
        self.cos = [math.cos(a) for a in angles_rad]

        # 로봇의 현재 상태 (x, y, yaw)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_joint_positions = None  # 이전 joint position 저장용

        # ---------------- Sub/Pub Initialization ----------------
        self.joint_sub = self.create_subscription(JointState, "/joint_states", self.joint_state_cb, 10)  # noqa
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)

        # TF Broadcaster는 publish_tf가 True일 때만 초기화
        self.tf_broadcaster = None
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)
            self.get_logger().warning("odom-base_link TF publishing is enabled.")

        self.get_logger().info("Wheel odometry publisher started")

    def joint_state_cb(self, msg: JointState):
        """/joint_states 메시지를 수신하여 오도메트리를 계산하고 발행."""
        # 각 wheel joint의 인덱스 저장
        indices = [msg.name.index(name) for name in self.joint_names]

        # 각 wheel의 선형 속도 계산 (v = ω * r)
        v1, v2, v3 = [msg.velocity[idx] * self.r for idx in indices]

        # 정기구학 기반 base_link 기준 속도 계산
        vx = (-self.sin[0] * v1 - self.sin[1] * v2 - self.sin[2] * v3) / 1.5
        vy = (self.cos[0] * v1 + self.cos[1] * v2 + self.cos[2] * v3) / 1.5
        wz = (v1 + v2 + v3) / (3.0 * self.R)

        # 현재 wheel joint 누적 각도 가져오기 (rad)
        positions = [msg.position[idx] for idx in indices]

        # 첫 콜백 초기화
        if self.last_joint_positions is None:
            self.last_joint_positions = positions
            return

        # 각 wheel의 이동 거리 계산 (d = Δθ * r)
        d1, d2, d3 = [
            (curr - prev) * self.r
            for curr, prev in zip(positions, self.last_joint_positions)
        ]

        # 정기구학 기반 base_link 기준 이동 거리 및 회전량 계산
        dx_b = (-self.sin[0] * d1 - self.sin[1] * d2 - self.sin[2] * d3) / 1.5
        dy_b = (self.cos[0] * d1 + self.cos[1] * d2 + self.cos[2] * d3) / 1.5
        dyaw = (d1 + d2 + d3) / (3.0 * self.R)

        time_stamp = msg.header.stamp

        # 다음 계산을 위해 현재 wheel joint position 저장
        self.last_joint_positions = positions

        # odom 좌표계 기준으로 변환 후 누적
        cos_y = math.cos(self.yaw)
        sin_y = math.sin(self.yaw)
        self.x += cos_y * dx_b - sin_y * dy_b
        self.y += sin_y * dx_b + cos_y * dy_b
        self.yaw += dyaw
        self.yaw = (self.yaw + math.pi) % (2 * math.pi) - math.pi  # normalization

        # 디버그 로그 출력
        self.get_logger().debug(
            "\n"
            f"vx: {vx:.4f}, vy: {vy:.4f}, wz: {wz:.4f}\n"
            f"dx_b: {dx_b:.4f}, dy_b: {dy_b:.4f}, dyaw: {dyaw:.4f}\n"
            f"x: {self.x:.4f}, y: {self.y:.4f}, yaw: {self.yaw:.4f}"
        )

        # 계산된 odometry 정보 발행
        self.publish_odometry(vx, vy, wz, time_stamp)

    def publish_odometry(self, vx: float, vy: float, wz: float, time_stamp) -> None:
        """계산된 odometry를 /wheel/odom 토픽과 TF로 발행."""
        # Odometry 메시지 생성
        odom = Odometry()
        odom.header.stamp = time_stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        # ---------------- Pose ----------------
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = yaw_to_quat(self.yaw)

        # Pose covariance (diagonal only)
        odom.pose.covariance = [0.0] * 36
        for i, v in enumerate(self.pose_cov_diag):
            odom.pose.covariance[i*7] = v

        # ---------------- Twist ----------------
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz

        # Twist covariance (diagonal only)
        odom.twist.covariance = [0.0] * 36
        for i, v in enumerate(self.twist_cov_diag):
            odom.twist.covariance[i*7] = v

        # Wheel_odom 메시지 발행
        self.odom_pub.publish(odom)

        # publish_tf 파라미터가 True일 때만 TF 발행
        if self.tf_broadcaster:
            tf = TransformStamped()
            tf.header.stamp = time_stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.translation.z = 0.0
            tf.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryPublisherNode()
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
