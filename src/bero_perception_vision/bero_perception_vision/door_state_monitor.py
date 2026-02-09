from threading import Lock

import numpy as np
import torch
from ultralytics import YOLO

from cv_bridge import CvBridge
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

from bero_msgs.msg import DoorStateViz


class DoorStateMonitor(Node):
    """
    Door State Monitoring Node.

    Flow:
    1. 카메라 이미지 구독
    2. YOLO 모델을 이용한 문 상태(state) 추론
    3. 추론 결과를 바탕으로 문의 개폐 상태 판단
    4. 판단된 문 상태(status)와 시각화 데이터를 토픽으로 발행
    """

    def __init__(self):
        super().__init__('door_state_monitoring_node')

        # ---------------- Declare Parameter ----------------
        self.declare_parameter('model_file', '')
        self.declare_parameter('imgsz', [544, 960])
        self.declare_parameter('conf', 0.8)
        self.declare_parameter('iou', 0.4)
        self.declare_parameter('count_threshold', 8)
        self.declare_parameter('infer_period', 0.1)  # 10Hz

        self.declare_parameter('label_opened', 'door_opened')
        self.declare_parameter('label_closed', 'door_closed')
        self.declare_parameter('label_moving', 'door_moving')

        # ---------------- Get Parameter ----------------
        model_file = self.get_parameter('model_file').value
        self.imgsz = self.get_parameter('imgsz').value
        self.conf = self.get_parameter('conf').value
        self.iou = self.get_parameter('iou').value
        infer_period = self.get_parameter('infer_period').value
        self.count_threshold = self.get_parameter('count_threshold').value

        label_opened = self.get_parameter('label_opened').value
        label_closed = self.get_parameter('label_closed').value
        label_moving = self.get_parameter('label_moving').value

        if not model_file:
            self.get_logger().fatal("Parameter 'model_file' is empty")
            raise RuntimeError("model_file is empty")

        # ---------------- Configuration ----------------
        self.label_to_state = {
            label_opened: "opened",
            label_closed: "closed",
            label_moving: "moving"
        }

        # ---------------- Runtime Variables ----------------
        self._latest_frame = None
        self._last_img_stamp = None
        self._enabled = False
        self._last_stable_status = "unknown"
        self._last_published_status = "unknown"
        self._counts = {"opened": 0, "closed": 0, "moving": 0}

        # ---------------- Threading & Locks ----------------
        self._infer_cb_group = MutuallyExclusiveCallbackGroup()
        self._sub_cb_group = MutuallyExclusiveCallbackGroup()
        self._lock = Lock()

        # ---------------- QoS ----------------
        viz_qos = QoSProfile(depth=1)
        viz_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        viz_qos.durability = DurabilityPolicy.VOLATILE

        # ---------------- ROS Communication ----------------
        self._bridge = CvBridge()

        self.img_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_cb,
            qos_profile_sensor_data,
            callback_group=self._sub_cb_group
        )
        self.infer_enable_sub = self.create_subscription(
            Bool,
            '/elevator/door_monitoring_enable',
            self.infer_enable_cb,
            10,
            callback_group=self._sub_cb_group
        )

        self.infer_timer = self.create_timer(
            infer_period,
            self.infer_timer_cb,
            callback_group=self._infer_cb_group
        )

        self.viz_pub = self.create_publisher(DoorStateViz, '/elevator/door_status_viz', viz_qos)
        self.status_pub = self.create_publisher(String, '/elevator/door_status', 10)

        # ---------------- Model settings ----------------
        self._model = YOLO(model_file, task='detect')
        self.get_logger().info(f"Loaded model: {model_file}, imgsz: {self.imgsz}")

        if torch.cuda.is_available():
            self._device = 'cuda:0'
            self.get_logger().info(f"YOLO model loaded on GPU: {torch.cuda.get_device_name(0)}")
        else:
            self._device = 'cpu'
            self.get_logger().warn("CUDA not available, using CPU")

        # warmup
        self.get_logger().info("Model warmup start")
        dummy = np.zeros((self.imgsz[0], self.imgsz[1], 3), dtype=np.uint8)
        self._model.predict(
            source=dummy,
            imgsz=self.imgsz,
            device=self._device,
            verbose=False,
        )
        self.get_logger().info("Model warmup finished")

    # ---------------- Callbacks ----------------

    def image_cb(self, msg: Image):
        """카메라 이미지 구독 콜백."""
        with self._lock:
            self._latest_frame = msg

    def infer_enable_cb(self, msg: Bool):
        """추론 활성화/비활성화 구독 콜백."""
        with self._lock:
            self._enabled = msg.data
            if not self._enabled:
                self._last_stable_status = "unknown"
                self._last_published_status = "unknown"
                self._counts = {"opened": 0, "closed": 0, "moving": 0}
        self.get_logger().info(f"Door monitor enable signal: {msg.data}")

    def infer_timer_cb(self):
        """추론 타이머 콜백."""
        # 시각화용 변수
        last_pub_status_viz = "unknown"
        counts_viz = None
        has_subscribers = self.viz_pub.get_subscription_count() > 0

        with self._lock:
            if not self._enabled and not has_subscribers:
                return
            frame_msg = self._latest_frame
            enabled = self._enabled
            counts_viz = self._counts.copy()

        # 이미지가 없을 경우
        if frame_msg is None:
            return

        # Frame drop 중복 처리
        if self._last_img_stamp == frame_msg.header.stamp:
            return
        self._last_img_stamp = frame_msg.header.stamp

        # 이미지 변환
        try:
            frame_bgr = self._bridge.imgmsg_to_cv2(frame_msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().debug(f"Error while converting Image: {e}")
            return

        # 헤더 복사
        msg_header_viz = frame_msg.header

        # 추론 비활성화 시 시각화
        if not enabled:
            if has_subscribers:
                self._publish_viz_data(
                    msg_header_viz, frame_bgr, "Inference disabled", counts_viz, []
                )
            return

        # YOLO 추론
        try:
            results = self._model.predict(
                source=frame_bgr,
                imgsz=self.imgsz,
                rect=True,
                conf=self.conf,
                iou=self.iou,
                max_det=1,
                device=self._device,
                verbose=False,
            )
        except Exception as e:
            self.get_logger().debug(f"YOLO inference failed: {e}")
            return

        # 추론 결과 가공
        detections = []
        detected_state = "unknown"

        if results[0].boxes:
            box = results[0].boxes[0]
            cls_name = results[0].names[int(box.cls[0].item())]
            conf = float(box.conf.item())

            if has_subscribers:
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                detections.append({
                    "bbox": [x1, y1, x2, y2],
                    "label": cls_name,
                    "conf": conf
                })

            detected_state = self.label_to_state.get(cls_name, "unknown")

        # 추론 결과 처리
        status = None
        with self._lock:
            if self._enabled:
                self._update_counts(detected_state)
                current_status = None

                # 상태 결정 로직
                if self._counts["moving"] >= self.count_threshold:
                    if self._last_stable_status == "closed":
                        current_status = "opening"
                    elif self._last_stable_status == "opened":
                        current_status = "closing"
                    else:
                        current_status = "moving"

                elif self._counts["opened"] >= self.count_threshold:
                    current_status = "opened"
                    self._last_stable_status = "opened"

                elif self._counts["closed"] >= self.count_threshold:
                    current_status = "closed"
                    self._last_stable_status = "closed"

                # 마지막 상태와 다를 경우 publish
                if current_status and current_status != self._last_published_status:
                    self._last_published_status = current_status
                    status = current_status

                # 시각화 데이터 준비
                if has_subscribers:
                    last_pub_status_viz = self._last_published_status
                    counts_viz = self._counts.copy()

        # 상태 publish
        if status:
            self.status_pub.publish(String(data=status))
            self.get_logger().info(f"Door Status: {status}")

        # 시각화 데이터 publish
        if has_subscribers:
            self._publish_viz_data(
                msg_header_viz, frame_bgr, last_pub_status_viz, counts_viz, detections
            )

    # ---------------- Helper Functions ----------------

    def _update_counts(self, detected_state: str):
        """
        Door state count를 업데이트.

        순간 노이즈를 방지하기 위해
        detected_state와 동일한 state의 count를 1 증가시키고,
        나머지 state의 count를 1 감소시킨다.
        """
        for state in self._counts:
            if state == detected_state:
                self._counts[state] = min(self.count_threshold, self._counts[state] + 1)
            else:
                self._counts[state] = max(0, self._counts[state] - 1)

    def _publish_viz_data(self, header, frame_bgr: np.ndarray, status: str, counts: dict, detections: list):  # noqa
        """
        /elevator/door_status_viz 토픽으로 시각화 데이터 전송.

        데이터 순서: [Header, Image, Status, Counts, BBox]
        """
        msg = DoorStateViz()

        # Header
        msg.header = header

        # Image
        msg.image.header = header
        msg.image = self._bridge.cv2_to_imgmsg(frame_bgr, encoding="bgr8")

        # Status
        msg.status = status

        # Counts
        if counts:
            msg.count_opened = counts['opened']
            msg.count_closed = counts['closed']
            msg.count_moving = counts['moving']
        else:
            msg.count_opened = 0
            msg.count_closed = 0
            msg.count_moving = 0

        # BBox
        if detections:
            det = detections[0]
            bbox = det['bbox']
            msg.bbox_x1 = bbox[0]
            msg.bbox_y1 = bbox[1]
            msg.bbox_x2 = bbox[2]
            msg.bbox_y2 = bbox[3]
            msg.bbox_class = det['label']
            msg.bbox_conf = float(det['conf'])
        else:
            msg.bbox_x1 = 0
            msg.bbox_y1 = 0
            msg.bbox_x2 = 0
            msg.bbox_y2 = 0
            msg.bbox_class = ""
            msg.bbox_conf = 0.0
        self.viz_pub.publish(msg)


def main():
    rclpy.init()
    node = DoorStateMonitor()
    executor = MultiThreadedExecutor(num_threads=2)
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
