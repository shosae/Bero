import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    pkg_path = get_package_share_directory('bero_perception_vision')
    engine_path = os.path.join(pkg_path, 'models', 'yolo26s_544_960_half_e2e_simplify.engine')
    pt_path = os.path.join(pkg_path, 'models', 'yolo26s_960_rect.pt')

    show_viz_arg = DeclareLaunchArgument(
        'show_viz',
        default_value='False',
        description='window display condition'
    )

    door_monitor_engine_node = Node(
        package='bero_perception_vision',
        executable='door_state_monitor',
        name='door_monitor_engine',
        output='screen',
        parameters=[{
            'model_file': engine_path,
            'imgsz': [544, 960],
        }],
        remappings=[
            ('/elevator/door_status', '/elevator/door_status_engine'),
            ('/elevator/door_status_viz', '/elevator/door_status_viz_engine')
        ]
    )

    door_monitor_pt_node = Node(
        package='bero_perception_vision',
        executable='door_state_monitor',
        name='door_monitor_pt',
        output='screen',
        parameters=[{
            'model_file': pt_path,
            'imgsz': [540, 960],
        }],
        remappings=[
            ('/elevator/door_status', '/elevator/door_status_pt'),
            ('/elevator/door_status_viz', '/elevator/door_status_viz_pt')
        ]
    )

    viz_engine_node = Node(
        package='bero_perception_vision',
        executable='door_state_viz',
        name='door_state_viz_engine',
        output='screen',
        condition=IfCondition(LaunchConfiguration('show_viz')),
        remappings=[
            ('/elevator/door_status_viz', '/elevator/door_status_viz_engine')
        ]
    )

    viz_pt_node = Node(
        package='bero_perception_vision',
        executable='door_state_viz',
        name='door_state_viz_pt',
        output='screen',
        condition=IfCondition(LaunchConfiguration('show_viz')),
        remappings=[
            ('/elevator/door_status_viz', '/elevator/door_status_viz_pt')
        ]
    )

    return LaunchDescription([
        show_viz_arg,
        door_monitor_engine_node,
        door_monitor_pt_node,
        viz_engine_node,
        viz_pt_node,
    ])
