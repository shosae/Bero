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
    engine_model_path = os.path.join(pkg_bero_perception_vision, 'models', 'yolo26s_544_960_half_e2e_simplify.engine')  # noqa: E501
    pt_model_path = os.path.join(pkg_bero_perception_vision, 'models', 'yolo26s_960_rect.pt')

    # Arguments
    show_viz = LaunchConfiguration('show_viz')

    show_viz_arg = DeclareLaunchArgument(
        'show_viz',
        default_value='False',
        description='Whether to start the comparison visualization nodes',
    )

    # Nodes
    door_monitor_engine_node = Node(
        package='bero_perception_vision',
        executable='door_state_monitor',
        name='door_monitor_engine',
        output='screen',
        parameters=[{
            'model_file': engine_model_path,
            'imgsz': [544, 960],
        }],
        remappings=[
            ('/elevator/door_status', '/elevator/door_status_engine'),
            ('/elevator/door_status_viz', '/elevator/door_status_viz_engine'),
        ],
    )

    door_monitor_pt_node = Node(
        package='bero_perception_vision',
        executable='door_state_monitor',
        name='door_monitor_pt',
        output='screen',
        parameters=[{
            'model_file': pt_model_path,
            'imgsz': [544, 960],
        }],
        remappings=[
            ('/elevator/door_status', '/elevator/door_status_pt'),
            ('/elevator/door_status_viz', '/elevator/door_status_viz_pt'),
        ],
    )

    viz_engine_node = Node(
        package='bero_perception_vision',
        executable='door_state_viz',
        name='door_state_viz_engine',
        output='screen',
        condition=IfCondition(show_viz),
        remappings=[
            ('/elevator/door_status_viz', '/elevator/door_status_viz_engine'),
        ],
    )

    viz_pt_node = Node(
        package='bero_perception_vision',
        executable='door_state_viz',
        name='door_state_viz_pt',
        output='screen',
        condition=IfCondition(show_viz),
        remappings=[
            ('/elevator/door_status_viz', '/elevator/door_status_viz_pt'),
        ],
    )

    return LaunchDescription([
        show_viz_arg,
        door_monitor_engine_node,
        door_monitor_pt_node,
        viz_engine_node,
        viz_pt_node,
    ])
