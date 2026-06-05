import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    # Package directories
    pkg_bero_perception_imu = get_package_share_directory('bero_perception_imu')

    # Path to files
    estimator_config_path = os.path.join(
        pkg_bero_perception_imu,
        'config',
        'floor_estimator_params.yaml',
    )

    # Arguments
    config_file = LaunchConfiguration('config_file')
    use_viz = LaunchConfiguration('use_viz')

    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=estimator_config_path,
        description='Path to floor estimator parameter file',
    )

    use_viz_arg = DeclareLaunchArgument(
        'use_viz',
        default_value='true',
        description='Whether to start the floor estimate visualization node',
    )

    # Nodes
    floor_estimator_node = Node(
        package='bero_perception_imu',
        executable='floor_estimator',
        name='floor_estimator',
        output='screen',
        parameters=[config_file],
    )

    floor_estimator_viz_node = Node(
        package='bero_perception_imu',
        executable='floor_estimator_viz',
        name='floor_estimator_viz',
        output='screen',
        condition=IfCondition(use_viz),
    )

    return LaunchDescription([
        config_arg,
        use_viz_arg,
        floor_estimator_node,
        floor_estimator_viz_node,
    ])
