import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pop.driving_base import DrivingBase


class OmniDriveController(Node):
    """
    Omni Drive Controller.

    /cmd_vel 토픽을 구독하여 로봇 속도 -> 각 바퀴의 각속도로 변환하고 제어
    DrivingBase 라이브러리를 사용하며, 각 바퀴의 속도 명령은 기준값 WHEEL_CENTER(100)을 중심으로
    100보다 작으면 역방향, 100보다 크면 정방향으로 회전

    모터 속도가 증가함에 따라 속도 명령 대비 실제 속도 증가폭이 감소하는 비선형적인 동작을 보여,
    실제 측정한 명령-속도 매핑 테이블을 사용하여 비율을 보정
    """

    def __init__(self):
        super().__init__("omni_drive_controller")

        # ---------------- Configuration ----------------
        self.r = 0.041  # 바퀴 반지름 (m)
        self.R = 0.1465  # 로봇 중심에서 바퀴까지의 거리 (m)
        self.angles_deg = [60.0, 300.0, 180.0]  # 각 바퀴의 각도 (deg)
        self.vel_mapping()  # 명령-속도 매핑 테이블

        # 각도(rad) 및 sin/cos 값 미리 계산
        angles_rad = [math.radians(deg) for deg in self.angles_deg]
        self.sin = [math.sin(rad) for rad in angles_rad]
        self.cos = [math.cos(rad) for rad in angles_rad]

        # ---------------- DrivingBase & Subscriber Initialization ----------------
        self.driver = DrivingBase("can0", 500000)
        self.subscription = self.create_subscription(
            Twist, "/cmd_vel", self.cmd_vel_callback, 10
        )

    def vel_mapping(self):
        """실제 측정값 기반 명령-속도 매핑 테이블."""
        self.cmd_values = [i for i in range(0, 81)]
        self.actual_velocities = [
            0, 2.00832, 2.11070, 2.21423, 2.31454, 2.41930, 2.52994, 2.63416, 2.73842,
            2.84276, 2.94427, 3.04667, 3.14937, 3.24979, 3.34983, 3.45665, 3.55730,
            3.65918, 3.76559, 3.86499, 3.96927, 4.07428, 4.17111, 4.27926, 4.38213,
            4.48546, 4.59145, 4.69252, 4.79835, 4.90096, 5.00738, 5.11186, 5.21490,
            5.32083, 5.42681, 5.53944, 5.63574, 5.74270, 5.84779, 5.95286, 6.05930,
            6.16499, 6.26801, 6.37679, 6.47986, 6.58630, 6.69247, 6.79808, 6.90474,
            7.00993, 7.11286, 7.21733, 7.32155, 7.42807, 7.53386, 7.63807, 7.74297,
            7.81623, 7.92091, 8.02478, 8.12866, 8.25195, 8.32442, 8.41934, 8.57043,
            8.67347, 8.77555, 8.91054, 9.01008, 9.11378, 9.25121, 9.35359, 9.45801,
            9.55409, 9.65882, 9.76252, 9.85996, 9.96249, 10.06160, 10.16744, 10.27037
        ]

        self.max_vel = max(self.actual_velocities)  # 최대 바퀴 각속도
        self.get_logger().info(f"Maximum achievable velocity: {self.max_vel:.2f} rad/s")

    def vel_to_cmd(self, target_velocity):
        """목표 각속도(rad/s)를 실제 명령값으로 변환."""
        abs_velocity = abs(target_velocity)
        sign = 1 if target_velocity >= 0 else -1  # 회전 방향 저장

        # 속도가 최대값을 초과하는 경우
        if abs_velocity > self.max_vel:
            return sign * 80  # 속도 한계치가 80

        # 선형 보간을 사용하여 매핑
        cmd = np.interp(abs_velocity, self.actual_velocities, self.cmd_values)

        return sign * int(round(cmd))

    def cmd_vel_callback(self, msg):
        # 수신한 Twist 메시지 (목표 로봇 속도)
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        # 역기구학 (Inverse Kinematics) 계산
        v1 = -self.sin[0] * vx + self.cos[0] * vy + self.R * wz
        v2 = -self.sin[1] * vx + self.cos[1] * vy + self.R * wz
        v3 = -self.sin[2] * vx + self.cos[2] * vy + self.R * wz

        # 각 바퀴의 선속도 -> 각 바퀴의 각속도(rad/s)
        w1 = v1 / self.r
        w2 = v2 / self.r
        w3 = v3 / self.r

        # 바퀴 각속도 한계 초과 시 가장 빠른 바퀴 기준 전체 scaling
        max_wheel = max(abs(w1), abs(w2), abs(w3))
        if max_wheel > self.max_vel:
            scale = self.max_vel / max_wheel
            w1 *= scale
            w2 *= scale
            w3 *= scale
            # self.get_logger().info(
            #     f"Wheel angular velocities scaled by {scale:.2f} to fit hardware limits."
            # )

        # 각 바퀴의 목표 속도 logging
        # self.get_logger().info(
        #     f"Wheel Angular Velocities: w1={w1:.2f}, w2={w2:.2f}, w3={w3:.2f}"
        # )

        # 목표 속도 - 하드웨어 명령 매핑
        cmd1 = self.vel_to_cmd(w1)
        cmd2 = self.vel_to_cmd(w2)
        cmd3 = self.vel_to_cmd(w3)

        self.driver.wheel_vec[0] = DrivingBase.WHEEL_CENTER + cmd1
        self.driver.wheel_vec[1] = DrivingBase.WHEEL_CENTER + cmd2
        self.driver.wheel_vec[2] = DrivingBase.WHEEL_CENTER + cmd3

        # cmd_vel -> HW 바퀴 명령값 logging
        # self.get_logger().info(
        #     f"CmdVel: [vx:{vx:.2f}, vy:{vy:.2f}, wz:{wz:.2f}] -> Wheels: {self.driver.wheel_vec}"
        # )

        # 하드웨어에 명령 전송
        self.driver.transfer()

    # 예상치 못한 종료에 로봇을 멈추도록
    def destroy_node(self):
        self.get_logger().info("Stopping the robot...")
        self.driver.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OmniDriveController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():  # ctrl+c shutdown 충돌 방지
            rclpy.shutdown()


if __name__ == "__main__":
    main()
