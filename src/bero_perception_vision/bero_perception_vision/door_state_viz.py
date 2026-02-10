import cv2

from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from bero_msgs.msg import DoorStateViz


class DoorStateVisualizer(Node):
    """
    DoorStateVisualizer.

    Flow:
    1. DoorStateViz 메시지를 구독하여 이미지와 상태 정보 수신
    2. 수신한 데이터를 기반으로 openCV window에 상태 표시
    """

    def __init__(self):
        super().__init__('door_state_visualizer')

        # ---------------- Configuration ----------------
        self.window_name = 'Door State Monitor'

        # ---------------- QoS ----------------
        viz_qos = QoSProfile(depth=1)
        viz_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        viz_qos.durability = DurabilityPolicy.VOLATILE

        # ---------------- ROS Communication ----------------
        self._bridge = CvBridge()

        self.viz_sub = self.create_subscription(
            DoorStateViz,
            '/elevator/door_status_viz',
            self.viz_cb,
            viz_qos
        )

        # ---------------- GUI Initialization ----------------
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 960, 540)

        self.get_logger().info("DoorStateVisualizer started")

    # ---------------- Callbacks ----------------

    def viz_cb(self, msg: DoorStateViz):
        """시각화 콜백."""
        try:
            # 이미지 변환
            frame = self._bridge.imgmsg_to_cv2(msg.image, desired_encoding='bgr8')

            # 박스 그리기
            if msg.bbox_class:
                x1 = msg.bbox_x1
                y1 = msg.bbox_y1
                x2 = msg.bbox_x2
                y2 = msg.bbox_y2

                if "opened" in msg.bbox_class:
                    color = (0, 255, 0)
                elif "closed" in msg.bbox_class:
                    color = (0, 0, 255)
                else:
                    color = (255, 165, 0)
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                cv2.putText(frame, f"{msg.bbox_class} {msg.bbox_conf:.2f}", (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            # 상태 정보 그리기
            status_text = f"Status: {msg.status}"
            count_text = f"O:{msg.count_opened} C:{msg.count_closed} M:{msg.count_moving}"

            cv2.putText(frame, status_text, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.putText(frame, count_text, (10, 65),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

            cv2.imshow(self.window_name, frame)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Failed to process visualization: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = DoorStateVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
