import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    # Package directories
    pkg_bero_navigation = get_package_share_directory('bero_navigation')
    pkg_nav2 = get_package_share_directory('nav2_bringup')
    pkg_bero_bringup = get_package_share_directory('bero_bringup')

    # Path to files
    map_path = os.path.join(pkg_bero_navigation, 'maps', 'dorm/floor_L.yaml')
    nav2_config_path = os.path.join(pkg_bero_navigation, 'config', 'nav2_params.yaml')
    rviz_config_path = os.path.join(pkg_bero_navigation, 'config', 'bero.rviz')
    nav2_bringup_launch_path = os.path.join(pkg_nav2, 'launch', 'bringup_launch.py')
    bero_bringup_launch_path = os.path.join(pkg_bero_bringup, 'launch', 'bero_bringup.launch.py')

    # Arguments
    map_config = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=map_path,
        description='Full path to map yaml file to load',
    )
    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=nav2_config_path,
        description='Full path to param file to load',
    )

    # Included launches
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_bringup_launch_path),
        launch_arguments={
            'autostart': 'true',  # activate all nav2 nodes, 없으면 하나하나 activate 필요
            'map': map_config,
            'params_file': params_file,
        }.items(),
    )

    bero_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bero_bringup_launch_path)
    )

    # Nodes
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
    )

    return LaunchDescription([
        map_arg,
        params_arg,
        bero_bringup_launch,
        nav2_bringup_launch,
        rviz_node,
    ])
