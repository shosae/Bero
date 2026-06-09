import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():

    # Package directories
    pkg_bero_localization = get_package_share_directory('bero_localization')

    # Path to files
    ekf_config_path = os.path.join(pkg_bero_localization, 'config', 'ekf_odom.yaml')

    # Nodes
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_odom',
        output='screen',
        parameters=[ekf_config_path],
    )

    return LaunchDescription([
        ekf_node,
    ])
