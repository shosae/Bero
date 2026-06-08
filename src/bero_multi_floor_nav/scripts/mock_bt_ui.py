#!/usr/bin/env python3
import sys
import os
import threading
import queue
from typing import Optional, Dict, Any
from dataclasses import dataclass, field
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QGroupBox
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QFont

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from bero_msgs.srv import GetMissionData

import yaml
from ament_index_python.packages import get_package_share_directory


# ========== BT Stage Definitions ==========

BT_STAGES = [
    {"stage": 0, "name": "호실 데이터 받기", "action": "auto", "waypoint": None},
    {"stage": 1, "name": "엘리베이터 버튼 앞", "action": "nav", "waypoint": "elevator_updown_button"},
    {"stage": 2, "name": "상승 버튼 누르기", "action": "arm", "button": "up"},
    {"stage": 3, "name": "엘리베이터 앞 이동", "action": "nav", "waypoint": "elevator_entrance"},
    {"stage": 4, "name": "문 열림 대기", "action": "door_monitor"},
    {"stage": 5, "name": "엘리베이터 진입", "action": "nav", "waypoint": "elevator_inside"},
    {"stage": 6, "name": "엘리베이터 이동 중", "action": "floor_arrival"},
    {"stage": 7, "name": "문 열림 대기", "action": "door_monitor"},
    {"stage": 8, "name": "엘리베이터 하차", "action": "nav", "waypoint": "elevator_exit"},
    {"stage": 9, "name": "호실로 이동", "action": "nav", "waypoint": "room"},
    {"stage": 10, "name": "배달 확인 대기", "action": "confirm"},
    {"stage": 11, "name": "엘리베이터 버튼 앞", "action": "nav", "waypoint": "elevator_updown_button"},
    {"stage": 12, "name": "하강 버튼 누르기", "action": "arm", "button": "down"},
    {"stage": 13, "name": "엘리베이터 앞 이동", "action": "nav", "waypoint": "elevator_entrance"},
    {"stage": 14, "name": "문 열림 대기", "action": "door_monitor"},
    {"stage": 15, "name": "엘리베이터 진입", "action": "nav", "waypoint": "elevator_inside"},
    {"stage": 16, "name": "엘리베이터 이동 중", "action": "floor_arrival", "floor": "0"},
    {"stage": 17, "name": "문 열림 대기", "action": "door_monitor"},
    {"stage": 18, "name": "엘리베이터 하차", "action": "nav", "waypoint": "elevator_exit"},
    {"stage": 19, "name": "베이스캠프 복귀", "action": "nav", "waypoint": "basecamp"},
]


# ========== Data Classes ==========

@dataclass
class MissionState:
    """Current mission state for UI display."""

    current_stage: int = 0
    target_room: str = "10"
    target_floor: str = "4"
    mission_uuid: list = field(default_factory=list)


# ========== ROS2 Worker Node ==========

class MockUINode(Node):
    """Multi floor nav 전용 mock UI."""

    def __init__(self, ui_queue: queue.Queue):
        super().__init__('mock_bt_ui')
        self.ui_queue = ui_queue
        self.callback_group = ReentrantCallbackGroup()
        self.waypoints: Dict[str, Any] = {}

        self.load_waypoints()

        # ========== Publishers ==========
        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self.elevator_status_pub = self.create_publisher(
            String, '/elevator/door_status', 1)
        self.floor_status_pub = self.create_publisher(
            String, '/floor_estimator/mock_status', 10)
        self.press_button_status_pub = self.create_publisher(
            String, '/press_button/mock_status', 10)

        # ========== Service Clients ==========
        self.get_mission_data_client = self.create_client(
            GetMissionData, 'get_mission_data')

        # ========== Subscribers ==========
        self.bt_phase_sub = self.create_subscription(
            String, '/bt_phase', self.on_bt_phase, 10,
            callback_group=self.callback_group)

        self.get_logger().info('[MockUINode] Initialized with waypoints loaded')

    def load_waypoints(self):
        """Waypoints 불러오기."""
        try:
            pkg_dir = get_package_share_directory('bero_multi_floor_nav')
            yaml_path = os.path.join(pkg_dir, 'config', 'waypoints.yaml')

            with open(yaml_path, 'r') as f:
                data = yaml.safe_load(f)

            self.waypoints = data.get('waypoints', {})
            self.get_logger().info(f'[MockUINode] Loaded {len(self.waypoints)} waypoints')
        except Exception as e:
            self.get_logger().warn(f'[MockUINode] Failed to load waypoints: {e}')
            self.waypoints = {}

    def get_waypoint(self, name: str) -> Optional[Dict]:
        return self.waypoints.get(name)

    def get_all_waypoint_names(self) -> list:
        return list(self.waypoints.keys())

    # ========== Subscription Callbacks ==========

    def on_bt_phase(self, msg: String):
        """BT phase 업데이트."""
        self.ui_queue.put(('phase', msg.data))
        self.query_mission_data()

    def query_mission_data(self):
        """Query action information."""
        if not self.get_mission_data_client.service_is_ready():
            return

        req = GetMissionData.Request()
        future = self.get_mission_data_client.call_async(req)

        def cb(fut):
            try:
                res = fut.result()
                if res and res.success:
                    self.ui_queue.put(('mission_data', (res.target_room_number, res.mission_uuid.uuid)))
            except Exception as e:
                self.get_logger().error(f"Failed to get mission data: {e}")

        future.add_done_callback(cb)

    # ========== Publisher Methods ==========

    def publish_floor_arrival(self, floor: str):
        """엘레베이터 도착 신호 publish."""
        msg = String()
        msg.data = f'getoff {floor}'
        self.floor_status_pub.publish(msg)
        self.get_logger().info(f'[MockUINode] Published floor arrival: getoff {floor}')

    def publish_button_pressed(self, button: str):
        """버튼 신호 publish."""
        msg = String()
        msg.data = f'pressed {button}'
        self.press_button_status_pub.publish(msg)
        self.get_logger().info(f'[MockUINode] Published button pressed: pressed {button}')

    def publish_elevator_door_opened(self):
        """엘레베이터 문 열림 신호 publish."""
        msg = String()
        msg.data = 'opened'
        self.elevator_status_pub.publish(msg)
        self.get_logger().info('[MockUINode] Published opened')

    def teleport_to_waypoint(self, waypoint_name: str):
        """지정된 waypoint로 로봇 이동."""
        import math
        wp = self.get_waypoint(waypoint_name)
        if not wp:
            self.get_logger().error(f'[MockUINode] Waypoint not found: {waypoint_name}')
            return False

        qx = float(wp['orientation'].get('x', 0.0))
        qy = float(wp['orientation'].get('y', 0.0))
        qz = float(wp['orientation']['z'])
        qw = float(wp['orientation']['w'])

        mag = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
        if mag > 0:
            qx, qy, qz, qw = qx/mag, qy/mag, qz/mag, qw/mag

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = wp.get('frame_id', 'map')
        msg.pose.pose.position.x = float(wp['position']['x'])
        msg.pose.pose.position.y = float(wp['position']['y'])
        msg.pose.pose.position.z = float(wp['position'].get('z', 0.0))
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        self.initialpose_pub.publish(msg)
        self.get_logger().info(f'[MockUINode] Teleport to {waypoint_name}')
        return True


# ========== PyQt5 Main Window ==========

class MockUIWindow(QMainWindow):
    """BT Mock UI의 Main Window."""

    phase_changed = pyqtSignal(str)

    def __init__(self, node: MockUINode, ui_queue: queue.Queue):
        super().__init__()
        self.node = node
        self.ui_queue = ui_queue
        self.mission_state = MissionState()

        self.init_ui()
        self.connect_signals()
        self.start_queue_timer()
        self.update_stage_display()

    def init_ui(self):
        """UI 초기화."""
        self.setWindowTitle('BT Flow Test UI')
        self.setMinimumSize(550, 400)
        self.resize(550, 400)

        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setContentsMargins(12, 12, 12, 12)
        main_layout.setSpacing(10)

        # ========== Mission Info ==========
        mission_group = QGroupBox('현재 Action')
        mission_layout = QHBoxLayout(mission_group)
        self.mission_label = QLabel('Action 대기 중...')
        self.mission_label.setFont(QFont('Arial', 14, QFont.Bold))
        self.mission_label.setAlignment(Qt.AlignCenter)
        self.mission_label.setStyleSheet('color: #4CAF50;')
        mission_layout.addWidget(self.mission_label)
        main_layout.addWidget(mission_group)

        # ========== Current Stage ==========
        stage_group = QGroupBox('현재 스테이지')
        stage_layout = QVBoxLayout(stage_group)

        # Stage 정보
        self.stage_label = QLabel('Stage 0: 호실 데이터 받기')
        self.stage_label.setFont(QFont('Arial', 14, QFont.Bold))
        self.stage_label.setAlignment(Qt.AlignCenter)
        stage_layout.addWidget(self.stage_label)

        # Stage 버튼
        stage_nav = QHBoxLayout()
        self.prev_btn = QPushButton('< 이전')
        self.prev_btn.clicked.connect(self.prev_stage)
        stage_nav.addWidget(self.prev_btn)

        self.next_btn = QPushButton('다음 >')
        self.next_btn.clicked.connect(self.next_stage)
        stage_nav.addWidget(self.next_btn)
        stage_layout.addLayout(stage_nav)

        main_layout.addWidget(stage_group)

        # ========== Action Button ==========
        self.action_btn = QPushButton('완료')
        self.action_btn.setMinimumHeight(80)
        self.action_btn.setStyleSheet('''
            QPushButton {
                background-color: #4CAF50; color: white;
                font-size: 20px; font-weight: bold;
                border-radius: 10px;
            }
            QPushButton:hover { background-color: #45a049; }
        ''')
        self.action_btn.clicked.connect(self.execute_current_stage)
        main_layout.addWidget(self.action_btn)

        main_layout.addStretch()

        # Dark theme
        self.setStyleSheet('''
            QMainWindow { background-color: #1e1e1e; }
            QGroupBox {
                font-weight: bold; color: #fff;
                border: 1px solid #555; border-radius: 8px;
                margin-top: 12px; padding-top: 10px;
            }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; }
            QLabel { color: #ccc; }
            QComboBox {
                background-color: #333; color: #fff;
                border: 1px solid #555; border-radius: 4px; padding: 5px;
            }
            QPushButton {
                background-color: #444; color: #fff;
                border: 1px solid #666; border-radius: 5px; padding: 8px;
            }
            QPushButton:hover { background-color: #555; }
        ''')

    def connect_signals(self):
        self.phase_changed.connect(self.on_phase_update)

    def start_queue_timer(self):
        self.queue_timer = QTimer()
        self.queue_timer.timeout.connect(self.process_queue)
        self.queue_timer.start(50)

    def process_queue(self):
        try:
            while True:
                msg_type, data = self.ui_queue.get_nowait()
                if msg_type == 'phase':
                    self.phase_changed.emit(data)
                elif msg_type == 'mission_data':
                    self.update_mission_data(data)
        except queue.Empty:
            pass

    # ========== Stage Navigation ==========

    def prev_stage(self):
        if self.mission_state.current_stage > 0:
            self.mission_state.current_stage -= 1
            self.update_stage_display()

    def next_stage(self):
        if self.mission_state.current_stage < len(BT_STAGES) - 1:
            self.mission_state.current_stage += 1
            self.update_stage_display()
        else:
            self.reset_to_waiting()

    def reset_to_waiting(self):
        """다시 대기 중으로 초기화."""
        self.mission_state.current_stage = 0
        self.mission_state.target_room = "10"
        self.mission_state.target_floor = "4"
        self.mission_state.mission_uuid = []

        self.mission_label.setText('Action 대기 중...')
        self.update_stage_display()

    def update_stage_display(self):
        stage = BT_STAGES[self.mission_state.current_stage]
        self.stage_label.setText(f"Stage {stage['stage']}: {stage['name']}")

        action = stage['action']
        if action == 'nav':
            wp = stage.get('waypoint', '')
            if wp == 'room':
                wp = self.mission_state.target_room
            self.action_btn.setText(f"{wp}로 이동 완료")
        elif action == 'arm':
            btn = stage.get('button', '')
            if btn == 'floor':
                btn = self.mission_state.target_floor
            self.action_btn.setText(f"버튼 {btn} 완료")
        elif action == 'door_monitor':
            if stage['stage'] in [7, 17]:
                self.action_btn.setText("문 열림 완료 (맵 변경 후 클릭)")
            else:
                self.action_btn.setText("문 열림 완료")
        elif action == 'floor_arrival':
            self.action_btn.setText("목표 층 도착")
        elif action == 'confirm':
            self.action_btn.setText("픽업 확인 완료")
        else:  # auto
            self.action_btn.setText("다음 스테이지")

    def execute_current_stage(self):
        stage = BT_STAGES[self.mission_state.current_stage]
        action = stage['action']

        if action == 'nav':
            wp = stage.get('waypoint', '')
            if wp == 'room':
                wp = self.mission_state.target_room
            self.node.teleport_to_waypoint(wp)
        elif action == 'arm':
            btn = stage.get('button', '')
            if btn == 'floor':
                btn = self.mission_state.target_floor
            self.node.publish_button_pressed(btn)
        elif action == 'door_monitor':
            self.node.publish_elevator_door_opened()
        elif action == 'floor_arrival':
            target_floor = stage.get('floor', self.mission_state.target_floor)
            self.node.publish_floor_arrival(target_floor)
        elif action == 'confirm':
            pass  # 실제 UI에서 수행

        self.next_stage()

    def on_phase_update(self, phase: str):
        """호실 정보 추출."""
        if phase:
            self.mission_label.setText(f"{phase}")

        import re
        match = re.search(r'(\d{3,4})호실', phase)
        if match:
            room = match.group(1)
            self.mission_state.target_room = room[-2:]
            if len(room) == 4:
                self.mission_state.target_floor = room[1]
                if room[1] == '2':
                    self.mission_state.target_room = room[1:]
                else:
                    self.mission_state.target_room = room[2:]
            elif len(room) == 3:
                self.mission_state.target_floor = room[0]
                if room[0] == '2':
                    self.mission_state.target_room = room
                else:
                    self.mission_state.target_room = room[1:]
            self.mission_label.setText(f"{room}호실 배달 중")
            self.update_stage_display()

    def update_mission_data(self, data):
        """받은 Action 데이터로 상태 업데이트."""
        room_number, uuid_bytes = data
        if not room_number:
            return

        self.mission_state.target_room = room_number[-2:]
        if len(room_number) == 4:
            self.mission_state.target_floor = room_number[1]
            if room_number[1] == '2':
                self.mission_state.target_room = room_number[1:]
            else:
                self.mission_state.target_room = room_number[2:]
        elif len(room_number) == 3:
            self.mission_state.target_floor = room_number[0]
            if room_number[0] == '2':
                self.mission_state.target_room = room_number
            else:
                self.mission_state.target_room = room_number[1:]

        self.mission_state.mission_uuid = list(uuid_bytes) if uuid_bytes is not None else []
        self.mission_label.setText(f"{room_number}호실 배달 중")
        self.update_stage_display()

    def closeEvent(self, event):
        self.queue_timer.stop()
        event.accept()


# ========== Main Entry Point ==========

def main(args=None):
    rclpy.init(args=args)

    ui_queue = queue.Queue()
    node = MockUINode(ui_queue)

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    app = QApplication(sys.argv)
    window = MockUIWindow(node, ui_queue)
    window.show()

    try:
        exit_code = app.exec_()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == '__main__':
    main()
