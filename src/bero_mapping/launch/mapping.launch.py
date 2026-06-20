import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    # Package directories
    pkg_bero_mapping = get_package_share_directory('bero_mapping')
    pkg_bero_bringup = get_package_share_directory('bero_bringup')

    # Path to files
    mapping_config_path = os.path.join(pkg_bero_mapping, 'config', 'mapper_params_online_async.yaml')  # noqa: E501
    rviz_config_path = os.path.join(pkg_bero_mapping, 'config', 'mapping.rviz')
    bero_bringup_launch_path = os.path.join(pkg_bero_bringup, 'launch', 'bero_bringup.launch.py')

    # Arguments
    use_rviz = LaunchConfiguration('use_rviz')
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz',
    )

    # Included launches
    bero_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bero_bringup_launch_path)
    )

    # Nodes
    async_slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[mapping_config_path],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        use_rviz_arg,
        bero_bringup_launch,
        async_slam_toolbox_node,
        rviz_node,
    ])
