import cv2
from cv_bridge import CvBridge
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image


class CameraNode(Node):
    """
    Camera Node.

    IMX219 카메라에서 gstreamer 파이프라인을 통해 이미지를 얻어서
    /camera/image_raw 토픽으로 발행
    """

    def __init__(self):
        super().__init__('camera_node')

        # ---------------- Declare Parameter ----------------
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('fps', 20)
        self.declare_parameter('rotate', True)
        self.declare_parameter('frame_id', 'camera')

        # ---------------- Get Parameter ----------------
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value
        self.frame_id = self.get_parameter('frame_id').value
        self.rotate = self.get_parameter('rotate').value

        self.get_logger().info(f"Resolution: {self.width}x{self.height} @ {self.fps}fps, Rotate: {self.rotate}")  # noqa

        # ---------------- ROS Communication ----------------
        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', qos_profile_sensor_data)

        # ---------------- Device Initialization ----------------

        # gstreamer pipeline
        flip_method = 2 if self.rotate else 0
        gst_str = (
            f"nvarguscamerasrc ! "
            f"video/x-raw(memory:NVMM), width={self.width}, height={self.height}, format=NV12, framerate={self.fps}/1 ! "  # noqa
            f"nvvidconv flip-method={flip_method} ! video/x-raw, width=960, height=544, format=BGRx ! "  # noqa
            f"videoconvert ! video/x-raw, format=BGR ! "
            f"videorate ! video/x-raw, framerate={self.fps}/1 ! "
            f"appsink max-buffers=1 drop=true sync=false"
        )

        self.cap = cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)

        if not self.cap.isOpened():
            self.get_logger().fatal("Camera not opened")
            raise RuntimeError("Camera not opened")

        # ---------------- Threading ----------------
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()

    def _capture_loop(self):
        """프레임을 읽고 발행."""
        while rclpy.ok() and not self._stop_event.is_set():
            if not self.cap.isOpened():
                break

            ret, frame = self.cap.read()

            if not ret:
                self.get_logger().fatal("Camera not returning frames. Shutting down node")
                self.cap.release()
                if rclpy.ok():
                    rclpy.shutdown()
                break

            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id

            if rclpy.ok():
                self.image_pub.publish(msg)

    def destroy_node(self):
        """thread/카메라 정리."""
        try:
            self.get_logger().info("Stopping Camera Node")
            self._stop_event.set()

            if hasattr(self, '_thread') and self._thread.is_alive():
                self._thread.join(timeout=1.0)

            if hasattr(self, 'cap') and self.cap.isOpened():
                try:
                    self.cap.release()
                except Exception as e:
                    self.get_logger().error(f"cap.release() failed: {e}")

        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
