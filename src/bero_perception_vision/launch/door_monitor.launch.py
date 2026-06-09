import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    # Package directories
    pkg_bero_perception_vision = get_package_share_directory('bero_perception_vision')

    # Path to files
    model_path = os.path.join(pkg_bero_perception_vision, 'models', 'yolo26s_544_960_half_e2e_simplify.engine')  # noqa: E501

    # Arguments
    show_viz = LaunchConfiguration('show_viz')

    show_viz_arg = DeclareLaunchArgument(
        'show_viz',
        default_value='False',
        description='Whether to start the door visualization node',
    )

    # Nodes
    door_monitor_node = Node(
        package='bero_perception_vision',
        executable='door_state_monitor',
        name='door_monitor',
        output='screen',
        parameters=[{
            'model_file': model_path,
            'imgsz': [544, 960],
        }],
    )

    viz_node = Node(
        package='bero_perception_vision',
        executable='door_state_viz',
        name='door_state_viz',
        output='screen',
        condition=IfCondition(show_viz),
    )

    return LaunchDescription([
        show_viz_arg,
        door_monitor_node,
        viz_node,
    ])
