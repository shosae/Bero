import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_path = get_package_share_directory('bero_perception_vision')
    engine_path = os.path.join(pkg_path, 'models', 'yolo26s_544_960_half_e2e_simplify.engine')
    pt_path = os.path.join(pkg_path, 'models', 'yolo26s_960_rect.pt')

    door_monitor_engine_node = Node(
        package='bero_perception_vision',
        executable='door_state_monitor',
        name='door_monitor_engine',
        output='screen',
        parameters=[{
            'model_file': engine_path,
            'imgsz': [544, 960],
            'show_window': True
        }],
        remappings=[
            ('/elevator/door_status', '/elevator/door_status_engine')
        ]
    )

    door_monitor_pt_node = Node(
        package='bero_perception_vision',
        executable='door_state_monitor',
        name='door_monitor_pt',
        output='screen',
        parameters=[{
            'model_file': pt_path,
            'imgsz': [960, 960],
            'show_window': True
        }],
        remappings=[
            ('/elevator/door_status', '/elevator/door_status_pt')
        ]
    )

    return LaunchDescription([
        door_monitor_engine_node,
        door_monitor_pt_node,
    ])
