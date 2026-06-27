import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    # Package directories
    pkg_bero_multi_floor_nav = get_package_share_directory('bero_multi_floor_nav')
    pkg_bero_navigation = get_package_share_directory('bero_navigation')
    pkg_bero_perception_imu = get_package_share_directory('bero_perception_imu')
    pkg_bero_perception_vision = get_package_share_directory('bero_perception_vision')
    pkg_bero_bringup = get_package_share_directory('bero_bringup')
    pkg_nav2 = get_package_share_directory('nav2_bringup')

    # Path to files
    waypoints_path = os.path.join(pkg_bero_multi_floor_nav, 'config', 'waypoints.yaml')
    bt_xml_path = os.path.join(pkg_bero_multi_floor_nav, 'behavior_trees', 'multi_floor_nav.xml')
    initial_map_path = os.path.join(pkg_bero_navigation, 'maps/dorm', 'floor_L.yaml')
    map_base_path = os.path.join(pkg_bero_navigation, 'maps/dorm', 'floor_')
    nav2_config_path = os.path.join(pkg_bero_navigation, 'config', 'nav2_params.yaml')
    rviz_config_path = os.path.join(pkg_bero_navigation, 'config', 'navigation.rviz')
    floor_estimator_config = os.path.join(pkg_bero_perception_imu, 'config', 'floor_estimator_params.yaml')
    bero_bringup_launch_path = os.path.join(pkg_bero_bringup, 'launch', 'bero_bringup.launch.py')
    nav2_bringup_launch_path = os.path.join(pkg_nav2, 'launch', 'bringup_launch.py')
    door_monitor_launch_path = os.path.join(pkg_bero_perception_vision, 'launch', 'door_monitor.launch.py')

    # Arguments
    initial_map_file = LaunchConfiguration('initial_map')
    map_base = LaunchConfiguration('map_base_path')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    waypoints_file = LaunchConfiguration('waypoints_path')
    launch_ui = LaunchConfiguration('launch_ui')
    launch_rviz = LaunchConfiguration('launch_rviz')
    launch_door_viz = LaunchConfiguration('launch_door_viz')
    use_mock = LaunchConfiguration('use_mock')

    initial_map_file_arg = DeclareLaunchArgument(
        'initial_map',
        default_value=initial_map_path,
        description='Full path to initial map yaml file',
    )

    nav2_params_arg = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=nav2_config_path,
        description='Full path to param file',
    )

    map_base_arg = DeclareLaunchArgument(
        'map_base_path',
        default_value=map_base_path,
        description='Full path prefix to multi-floor map yaml files',
    )

    waypoints_file_arg = DeclareLaunchArgument(
        'waypoints_path',
        default_value=waypoints_path,
        description='Full path to waypoint yaml file',
    )

    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Whether to launch RViz',
    )

    launch_ui_arg = DeclareLaunchArgument(
        'launch_ui',
        default_value='true',
        description='Whether to launch the PyQt UI',
    )

    launch_door_viz_arg = DeclareLaunchArgument(
        'launch_door_viz',
        default_value='true',
        description='Whether to show door monitor visualization',
    )

    use_mock_arg = DeclareLaunchArgument(
        'use_mock',
        default_value='false',
        description='Whether to use mock servers',
    )

    # Included launches
    nav2_params_with_overrides = RewrittenYaml(
        source_file=nav2_params_file,
        param_rewrites={
            'bt_navigator_navigate_to_pose_rclcpp_node.ros__parameters.multi_floor_waypoints_file_path': waypoints_file,  # noqa: E501
            'bt_navigator_navigate_to_pose_rclcpp_node.ros__parameters.multi_floor_map_base_path': map_base,
        },
    )

    bero_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bero_bringup_launch_path)
    )

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_bringup_launch_path),
        launch_arguments={
            'autostart': 'true',
            'map': initial_map_file,
            'params_file': nav2_params_with_overrides,
            'use_sim_time': 'false',
        }.items(),
    )

    door_monitor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(door_monitor_launch_path),
        launch_arguments={
            'show_viz': launch_door_viz,
        }.items(),
    )

    # Nodes
    mission_manager_node = Node(
        package='bero_multi_floor_nav',
        executable='mission_manager',
        name='mission_manager',
        output='screen',
        parameters=[{
            'bt_xml_path': bt_xml_path,
            'waypoints_path': waypoints_file,
        }],
    )

    floor_estimator_node = Node(
        package='bero_perception_imu',
        executable='floor_estimator',
        name='floor_estimator',
        output='screen',
        parameters=[
            floor_estimator_config,
        ],
    )

    az_offset_calibrator_node = Node(
        package='bero_perception_imu',
        executable='az_offset_calibrator',
        name='az_offset_calibrator',
        output='screen',
        parameters=[{
            'target_node_name': '/floor_estimator',
            'calibration_duration_sec': 1.0,
            'min_samples': 50,
        }],
    )

    ui_node = Node(
        package='bero_ui',
        executable='qt_ui_node',
        name='qt_ui_node',
        output='screen',
        condition=IfCondition(launch_ui),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(launch_rviz),
    )

    mock_press_button_server = Node(
        package='bero_multi_floor_nav',
        executable='mock_press_button_server.py',
        name='mock_press_button_arm',
        output='screen',
    )

    mock_estimate_floor_server = Node(
        package='bero_multi_floor_nav',
        executable='mock_estimate_floor_server.py',
        name='mock_floor_estimator',
        output='screen',
    )

    mock_az_offset_server = Node(
        package='bero_multi_floor_nav',
        executable='mock_az_offset_server.py',
        name='mock_az_offset_calibrator',
        output='screen',
    )

    mock_door_status_server = Node(
        package='bero_multi_floor_nav',
        executable='mock_door_status_server.py',
        name='mock_door_status_server',
        output='screen',
    )

    mock_ui_node = Node(
        package='bero_multi_floor_nav',
        executable='mock_bt_ui.py',
        name='mock_bt_ui',
        output='screen',
    )

    # Groups
    elevator_nav_group = GroupAction(
        actions=[
            floor_estimator_node,
            az_offset_calibrator_node,
            door_monitor_launch,
        ],
        condition=UnlessCondition(use_mock),
    )

    mock_server_group = GroupAction(
        actions=[
            mock_estimate_floor_server,
            mock_az_offset_server,
            mock_door_status_server,
            mock_press_button_server,
            mock_ui_node,
        ],
        condition=IfCondition(use_mock),
    )

    return LaunchDescription([
        initial_map_file_arg,
        map_base_arg,
        nav2_params_arg,
        waypoints_file_arg,
        launch_ui_arg,
        launch_rviz_arg,
        launch_door_viz_arg,
        use_mock_arg,
        bero_bringup_launch,
        nav2_bringup_launch,
        mission_manager_node,
        ui_node,
        rviz_node,
        elevator_nav_group,
        mock_server_group,
    ])
