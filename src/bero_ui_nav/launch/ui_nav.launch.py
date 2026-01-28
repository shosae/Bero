import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
)

from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile


def generate_launch_description():

    # Package directories
    pkg_bero_ui_nav = get_package_share_directory('bero_ui_nav')
    pkg_bero_navigation = get_package_share_directory('bero_navigation')

    # Path to files
    waypoints_path = os.path.join(pkg_bero_ui_nav, 'config', 'waypoints.yaml')
    pkg_bero_navigation_path = os.path.join(pkg_bero_navigation, 'launch', 'navigation.launch.py')

    # Arguments
    waypoints_arg = DeclareLaunchArgument(
        'waypoints',
        default_value=waypoints_path
    )

    # Bero navigation launch
    bero_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(pkg_bero_navigation_path)
    )

    # Mission Manager node
    mission_manager_node = Node(
        package='bero_ui_nav',
        executable='mission_manager',
        name='mission_manager',
        output='screen',
        parameters=[ParameterFile(LaunchConfiguration('waypoints'))],
    )

    return LaunchDescription([
        waypoints_arg,
        bero_navigation_launch,
        mission_manager_node,
    ])
