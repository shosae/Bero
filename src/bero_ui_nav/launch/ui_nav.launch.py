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
from launch_ros.descriptions import ParameterFile


def generate_launch_description():

    # Package directories
    pkg_bero_ui_nav = get_package_share_directory('bero_ui_nav')
    pkg_bero_navigation = get_package_share_directory('bero_navigation')

    # Path to files
    waypoints_path = os.path.join(pkg_bero_ui_nav, 'config', 'waypoints.yaml')
    navigation_launch_path = os.path.join(pkg_bero_navigation, 'launch', 'navigation.launch.py')

    # Arguments
    waypoints = LaunchConfiguration('waypoints')
    launch_ui = LaunchConfiguration('launch_ui')

    waypoints_arg = DeclareLaunchArgument(
        'waypoints',
        default_value=waypoints_path,
        description='Path to waypoint parameter file',
    )

    launch_ui_arg = DeclareLaunchArgument(
        'launch_ui',
        default_value='true',
        description='Whether to launch the PyQt UI',
    )

    # Included launches
    bero_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch_path)
    )

    # Nodes
    mission_manager_node = Node(
        package='bero_ui_nav',
        executable='mission_manager',
        name='mission_manager',
        output='screen',
        parameters=[ParameterFile(waypoints)],
    )

    ui_node = Node(
        package='bero_ui',
        executable='qt_ui_node',
        name='qt_ui_node',
        output='screen',
        condition=IfCondition(launch_ui),
    )

    return LaunchDescription([
        waypoints_arg,
        launch_ui_arg,
        bero_navigation_launch,
        mission_manager_node,
        ui_node,
    ])
