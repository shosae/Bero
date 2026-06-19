import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():

    # Package directories
    pkg_imu = get_package_share_directory('imu')
    pkg_bero_localization = get_package_share_directory('bero_localization')
    pkg_bero_description = get_package_share_directory('bero_description')
    pkg_rplidar_ros = get_package_share_directory('rplidar_ros')

    # Path to files
    imu_config_path = os.path.join(pkg_imu, 'config', 'imu_cov.yaml')
    wheel_odom_config_path = os.path.join(pkg_bero_localization, 'config', 'wheel_odom_cov.yaml')
    ekf_launch_path = os.path.join(pkg_bero_localization, 'launch', 'ekf_odom.launch.py')
    bero_description_launch_path = os.path.join(pkg_bero_description, 'launch', 'bero_description.launch.py')  # noqa: E501
    rplidar_launch_path = os.path.join(pkg_rplidar_ros, 'launch', 'rplidar_s3_launch.py')

    # Included launches
    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ekf_launch_path)
    )

    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bero_description_launch_path)
    )

    rplidar_s3_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rplidar_launch_path),
        launch_arguments={
            'scan_mode': 'DenseBoost',
            'scan_frequency': '20.0',
        }.items(),
    )

    # Nodes
    imu_publisher_node = Node(
        package='imu',
        executable='imu_publisher',
        name='imu_publisher',
        output='screen',
        parameters=[imu_config_path],
    )

    joint_state_publisher_node = Node(
        package='encoder',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
    )

    odometry_publisher_node = Node(
        package='bero_localization',
        executable='odometry_publisher',
        name='odometry_publisher',
        output='screen',
        parameters=[wheel_odom_config_path],
    )

    camera_node = Node(
        package='camera',
        executable='camera_node',
        name='camera_node',
        output='screen',
    )

    return LaunchDescription([
        ekf_launch,
        robot_state_publisher_launch,
        rplidar_s3_launch,
        imu_publisher_node,
        joint_state_publisher_node,
        odometry_publisher_node,
        camera_node,
    ])
