import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    pkg_path = get_package_share_directory('bero_perception_vision')
    model_path = os.path.join(pkg_path, 'models', 'yolo26s_544_960_half_e2e_simplify.engine')

    show_viz_arg = DeclareLaunchArgument(
        'show_viz',
        default_value='False',
        description='window display condition'
    )

    door_monitor_node = Node(
        package='bero_perception_vision',
        executable='door_state_monitor',
        name='door_monitor',
        output='screen',
        parameters=[{
            'model_file': model_path,
            'imgsz': [544, 960]
        }]
    )

    viz_node = Node(
        package='bero_perception_vision',
        executable='door_state_viz',
        name='door_state_viz',
        output='screen',
        condition=IfCondition(LaunchConfiguration('show_viz'))
    )

    return LaunchDescription([
        show_viz_arg,
        door_monitor_node,
        viz_node
    ])
