import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from pop.Imu import Imu as popImu


class ImuPublisherNode(Node):
    """
    Imu publisher.

    Imu 센서에서 accel, gyro, quat 데이터를 읽어 /imu/data 토픽으로 발행
    """

    def __init__(self):
        super().__init__("imu_publisher")

        # ---------------- Declare Parameter ----------------
        # EKF에서 사용되지 않는 항목이 무시되도록 1e3으로 크게 설정
        self.declare_parameter("rpy_cov_diag", [1e3, 1e3, 1e3])
        self.declare_parameter("gyro_cov_diag", [1e3, 1e3, 1e-4])
        self.declare_parameter("accel_cov_diag", [1e3, 1e3, 1e3])

        # ---------------- Get Parameter ----------------
        self.orientation_cov_diag = self.get_parameter("rpy_cov_diag").value
        self.ang_vel_cov_diag = self.get_parameter("gyro_cov_diag").value
        self.lin_acc_cov_diag = self.get_parameter("accel_cov_diag").value

        # ---------------- Configuration ----------------
        self.frame_id = "imu_link"

        # ---------------- ROS Communication ----------------
        self.pub = self.create_publisher(Imu, "/imu/data", qos_profile_sensor_data)

        # ---------------- Device Initialization ----------------
        self.imu = popImu("can0", 500000)
        self.imu.callback(self.imu_cb, repeat=1)

        self.get_logger().info("IMU publisher started")

    # imu callback
    def imu_cb(self, data: tuple[list[float], list[float], list[float], list[float], list[float]]) -> None:  # noqa
        accel, mag, gyro, euler, quat = data  # mag, euler는 sensor_msgs.msg/Imu에 포함되지 않음

        # # [DEBUG] Imu Data 확인
        # self.get_logger().debug(
        #     f"\n"
        #     f"[IMU Data]\n"
        #     f"  Accel: {accel}\n"
        #     f"  Mag:   {mag}\n"
        #     f"  Gyro:  {gyro}\n"
        #     f"  Euler: {euler}\n"
        #     f"  Quat:  {quat}\n"
        # )

        # Imu 메시지 생성
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # ---------------- Orientation ----------------
        # quat data: (w,x,y,z)
        msg.orientation.x = float(quat[1])
        msg.orientation.y = float(quat[2])
        msg.orientation.z = float(quat[3])
        msg.orientation.w = float(quat[0])

        # Orientation covariance
        orientation_cov = [0.0] * 9
        orientation_cov[0] = float(self.orientation_cov_diag[0])  # Roll
        orientation_cov[4] = float(self.orientation_cov_diag[1])  # Pitch
        orientation_cov[8] = float(self.orientation_cov_diag[2])  # Yaw
        msg.orientation_covariance = orientation_cov

        # ---------------- Angular velocity ----------------
        msg.angular_velocity.x = float(gyro[0])
        msg.angular_velocity.y = float(gyro[1])
        msg.angular_velocity.z = float(gyro[2])

        # Angular velocity covariance
        ang_vel_cov = [0.0] * 9
        ang_vel_cov[0] = float(self.ang_vel_cov_diag[0])  # angular_x
        ang_vel_cov[4] = float(self.ang_vel_cov_diag[1])  # angular_y
        ang_vel_cov[8] = float(self.ang_vel_cov_diag[2])  # angular_z
        msg.angular_velocity_covariance = ang_vel_cov

        # ---------------- Linear acceleration ----------------
        msg.linear_acceleration.x = float(accel[0])
        msg.linear_acceleration.y = float(accel[1])
        msg.linear_acceleration.z = float(accel[2])

        # Linear acceleration covariance
        lin_acc_cov = [0.0] * 9
        lin_acc_cov[0] = float(self.lin_acc_cov_diag[0])  # linear_x
        lin_acc_cov[4] = float(self.lin_acc_cov_diag[1])  # linear_y
        lin_acc_cov[8] = float(self.lin_acc_cov_diag[2])  # linear_z
        msg.linear_acceleration_covariance = lin_acc_cov

        # Imu 메시지 발행
        self.pub.publish(msg)

    # Imu thread 확실하게 종료
    def destroy_node(self):
        try:
            self.get_logger().info("Stopping Imu Publisher")
            self.imu.stop()
        except Exception as e:
            self.get_logger().error(f"Failed to stop the imu during shutdown: {e}")
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImuPublisherNode()
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
