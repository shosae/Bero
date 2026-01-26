import os
import sys
import threading
import queue
from typing import Optional
from ament_index_python.packages import get_package_share_directory

from PyQt5.QtCore import Qt, QThread, QTimer, pyqtSignal, pyqtSlot
from PyQt5.QtGui import QCloseEvent, QFont, QFontDatabase
from PyQt5.QtWidgets import (
    QApplication,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QPushButton,
    QLabel,
    QSizePolicy,
    QStackedWidget,
)

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy

from unique_identifier_msgs.msg import UUID
from bero_ui_nav.action import DeliverToRoom


class DeliverToRoomActionClient(Node):
    """
    RosThread에서 동작하는 deliver_to_room Action Client Node.

    Qt UI로부터 action goal 혹은 pickup_confirm 요청을 받아 서버로 전송하고,
    진행 상황(Feedback/Result)을 Qt Signal로 UI로 전달한다.
    """

    def __init__(self, status_signal: pyqtSignal):
        super().__init__("deliver_to_room_action_client")
        self._status_signal = status_signal
        self._delivery_client = ActionClient(self, DeliverToRoom, "deliver_to_room")
        self._current_goal_handle = None

        # Subscriber가 늦게 들어와도 마지막 메시지를 받을 수 있도록 TRANSIENT_LOCAL 설정
        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._pickup_confirm_pub = self.create_publisher(UUID, "/pickup/confirm", qos)

    def _emit_status(self, phase: str, status: str):
        self._status_signal.emit(phase, status)

    def send_goal(self, room_number: str):
        """deliver_to_room action의 goal을 비동기로 요청."""
        if not self._delivery_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("deliver_to_room action server not available")
            self._emit_status("error", "배달 서버가 준비되지 않았습니다.")
            return

        goal_msg = DeliverToRoom.Goal()
        goal_msg.room_number = room_number
        self._emit_status("waiting", "배달 요청 중...")

        future = self._delivery_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback,
        )
        future.add_done_callback(self._goal_response_callback)

    def _feedback_callback(self, feedback_msg):
        """Goal feedback을 UI로 전달."""
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Mission feedback: {feedback.phase} - {feedback.status}")
        self._emit_status(feedback.phase, feedback.status)

    def _goal_response_callback(self, future):
        """Goal 수락 여부 확인."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Mission goal rejected")
            self._emit_status("error", "배달 요청이 거절되었습니다.\n(이미 실행 중 or 잘못된 호실 번호)")
            return

        self._current_goal_handle = goal_handle
        self.get_logger().info("Mission goal accepted")
        self._emit_status("started", "배달 시작!")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future):
        """Goal result를 UI로 전달."""
        result = future.result().result
        if result.success:
            self.get_logger().info(f"Mission goal completed: {result.message}")
            self._emit_status("completed", result.message)
        else:
            self.get_logger().info(f"Mission goal failed: {result.message}")
            self._emit_status("error", result.message)
        self._current_goal_handle = None

    def publish_pickup_confirm(self):
        """수령 확인을 위해 현재 goal의 UUID를 publish."""
        if self._current_goal_handle is None:
            self._emit_status("error", "현재 진행중인 배달이 없습니다.")
            return

        goal_id = self._current_goal_handle.goal_id
        self._pickup_confirm_pub.publish(goal_id)
        self.get_logger().info("/pickup/confirm: UUID published.")


class RosThread(QThread):
    """
    ROS2 노드를 실행하는 Worker thread.

    UI -> ROS 명령은 Queue에 추가하는 방식으로 전달받고,
    ROS -> UI 상태 전환은 Signal과 Slot을 통해 전달한다.
    """

    mission_status = pyqtSignal(str, str)  # phase, status

    def __init__(self, args=None, parent=None):
        super().__init__(parent)
        self._args = args
        self._cmd_queue = queue.Queue()
        self._running = threading.Event()
        self._node: Optional[DeliverToRoomActionClient] = None

    def run(self):
        self._running.set()
        rclpy.init(args=self._args)
        self._node = DeliverToRoomActionClient(self.mission_status)

        # UI -> ROS Queue에 작업이 있으면 먼저 처리
        try:
            while self._running.is_set() and rclpy.ok():
                self._drain_commands()
                if not self._running.is_set():
                    break
                rclpy.spin_once(self._node, timeout_sec=0.1)
        finally:
            self._node.destroy_node()
            rclpy.shutdown()

    def _drain_commands(self):
        while True:
            try:
                cmd, payload = self._cmd_queue.get_nowait()
            except queue.Empty:
                return

            if cmd == "send_goal" and self._node:
                self._node.send_goal(payload or "")
            elif cmd == "publish_pickup_confirm" and self._node:
                self._node.publish_pickup_confirm()
            elif cmd == "stop":
                self._running.clear()
                return

    def send_delivery_request(self, room_number: str):
        self._cmd_queue.put(("send_goal", room_number))

    def publish_pickup_confirm(self):
        self._cmd_queue.put(("publish_pickup_confirm", None))

    def stop(self):
        self._cmd_queue.put(("stop", None))
        self.wait(3000)


class RoomInputWindow(QWidget):
    """
    Main thread에서 실행되는 Qt UI.

    사용자에게 방 번호를 입력받고 진행 상황을 표시한다.
    버튼 이벤트를 RosThread의 Queue에 추가하는 방식으로 전달하며,
    Signal을 통해 전달된 상황에 따라 QStackedWidget으로 화면을 전환한다.
    """

    def __init__(self, ros_thread: RosThread):
        super().__init__()
        self._ros_thread = ros_thread
        self.setWindowTitle("BERO")

        self._room_number = ""
        self._mission_active = False

        self._build_ui()
        self._show_input()

        self._ros_thread.mission_status.connect(self._update_mission_status)

    # ---------------- UI BUILD ----------------

    def _center_label(self, text: str, style: Optional[str] = None) -> QLabel:
        lbl = QLabel(text)
        lbl.setAlignment(Qt.AlignCenter)
        if style:
            lbl.setStyleSheet(style)
        return lbl

    def _build_ui(self):
        self._stacked = QStackedWidget(self)

        # Room number Input page
        self._input_widget = QWidget()
        input_layout = QVBoxLayout()

        self._room_label = self._center_label("호실 번호를 입력하세요")
        input_layout.addWidget(self._room_label)

        btn_layout = QHBoxLayout()

        self._mission_goal_publish_btn = QPushButton("확인")
        self._mission_goal_publish_btn.clicked.connect(self._handle_start_delivery_mission)
        btn_layout.addWidget(self._mission_goal_publish_btn)

        self._backspace_btn = QPushButton("Backspace")
        self._backspace_btn.clicked.connect(self._handle_backspace)
        btn_layout.addWidget(self._backspace_btn)

        input_layout.addLayout(btn_layout)

        num_layout = QVBoxLayout()
        numbers = [str(i) for i in range(10)]
        for row in range(2):
            row_layout = QHBoxLayout()
            for col in range(5):
                idx = row * 5 + col
                if idx < len(numbers):
                    n = numbers[idx]
                    btn = QPushButton(n)
                    btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
                    btn.clicked.connect(lambda _, digit=n: self._handle_add_digit(digit))
                    row_layout.addWidget(btn)
            num_layout.addLayout(row_layout)

        input_layout.addLayout(num_layout)
        self._input_widget.setLayout(input_layout)

        # Delivery progress page
        self._mission_status_widget = QWidget()
        mission_layout = QVBoxLayout()
        self._mission_status_label = self._center_label("배달 중 상태 라벨")
        mission_layout.addWidget(self._mission_status_label)
        self._mission_status_widget.setLayout(mission_layout)

        # Pickup confirm page
        self._pickup_confirm_widget = QWidget()
        pickup_confirm_layout = QVBoxLayout()
        self._confirm_label = self._center_label("수령 확인 라벨")
        pickup_confirm_layout.addWidget(self._confirm_label)

        self._pickup_confirm_btn = QPushButton("확인")
        self._pickup_confirm_btn.clicked.connect(self._handle_pickup_confirm)
        pickup_confirm_layout.addWidget(self._pickup_confirm_btn)
        self._pickup_confirm_widget.setLayout(pickup_confirm_layout)

        # Error page
        self._error_widget = QWidget()
        error_layout = QVBoxLayout()
        self._error_label = self._center_label("오류 라벨", style="color: red; font-size: 18px;")
        error_layout.addWidget(self._error_label)

        self._error_confirm_btn = QPushButton("확인")
        self._error_confirm_btn.clicked.connect(self._show_input)
        error_layout.addWidget(self._error_confirm_btn)
        self._error_widget.setLayout(error_layout)

        # Stack
        self._stacked.addWidget(self._input_widget)
        self._stacked.addWidget(self._mission_status_widget)
        self._stacked.addWidget(self._pickup_confirm_widget)
        self._stacked.addWidget(self._error_widget)

        root = QVBoxLayout()
        root.addWidget(self._stacked)
        self.setLayout(root)

    # ---------------- INPUT HANDLERS ----------------

    def _handle_add_digit(self, digit: str):
        if self._mission_active:
            return
        if len(self._room_number) < 4:
            self._room_number += digit
            self._room_label.setStyleSheet("")
            self._room_label.setText(self._room_number)

    def _handle_backspace(self):
        if self._mission_active:
            return
        self._room_number = self._room_number[:-1]
        if self._room_number:
            self._room_label.setStyleSheet("")
            self._room_label.setText(self._room_number)
        else:
            self._room_label.setStyleSheet("color: #888888; font-size: 12pt;")
            self._room_label.setText("호실 번호를 입력하세요")

    def _handle_start_delivery_mission(self):
        if not self._room_number or self._mission_active:
            return
        self._mission_active = True
        self._show_mission_status("Action server 연결 중...")
        self._ros_thread.send_delivery_request(self._room_number)

    def _handle_pickup_confirm(self):
        self._ros_thread.publish_pickup_confirm()

    # ---------------- SCREEN HELPERS ----------------

    def _show_input(self):
        self._stacked.setCurrentWidget(self._input_widget)
        if self._room_number:
            self._room_label.setStyleSheet("")
            self._room_label.setText(self._room_number)
        else:
            self._room_label.setStyleSheet("color: #888888; font-size: 12pt;")
            self._room_label.setText("호실 번호를 입력하세요")

    def _show_mission_status(self, text: str):
        self._stacked.setCurrentWidget(self._mission_status_widget)
        self._mission_status_label.setText(text)

    def _show_pickup_confirm(self, text: str):
        self._stacked.setCurrentWidget(self._pickup_confirm_widget)
        self._confirm_label.setText(text)
        self._pickup_confirm_btn.setEnabled(True)

    def _show_error(self, text: str):
        self._stacked.setCurrentWidget(self._error_widget)
        self._error_label.setText(f"오류 발생!\n{text}")

    # ---------------- STATUS UPDATE ----------------

    @pyqtSlot(str, str)
    def _update_mission_status(self, phase: str, status: str):
        if phase == "waiting":
            self._mission_active = True
            self._show_mission_status("Action server 연결 중...")

        elif phase in ("started", "moving_to_room", "return"):
            self._mission_active = True
            self._show_mission_status(status)

        elif phase == "confirm":
            self._mission_active = True
            self._show_pickup_confirm(status)

        elif phase == "completed":
            self._mission_active = False
            self._room_number = ""
            self._show_mission_status(status)
            QTimer.singleShot(2000, self._show_input)

        elif phase == "error":
            self._mission_active = False
            self._room_number = ""
            self._show_error(status)

    def closeEvent(self, event: QCloseEvent):
        self._ros_thread.stop()
        super().closeEvent(event)


def main(args=None):
    app = QApplication(sys.argv)

    # ---------------- LOAD FONT ----------------

    package_name = 'bero_ui'
    share_dir = get_package_share_directory(package_name)
    font_path = os.path.join(share_dir, 'fonts', 'NanumGothic.ttf')

    font_id = QFontDatabase.addApplicationFont(font_path)
    if font_id != -1:
        font_family = QFontDatabase.applicationFontFamilies(font_id)[0]
        app.setFont(QFont(font_family, 12))
        print(f"[INFO] Loaded font: {font_family} from {font_path}")
    else:
        print(f"[ERROR] Failed to load font at: {font_path}")

    ros_thread = RosThread(args=args)
    window = RoomInputWindow(ros_thread)
    ros_thread.start()

    app.aboutToQuit.connect(ros_thread.stop)
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
