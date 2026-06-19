from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    # Arguments
    channel_type = LaunchConfiguration('channel_type')
    serial_port = LaunchConfiguration('serial_port')
    serial_baudrate = LaunchConfiguration('serial_baudrate')
    frame_id = LaunchConfiguration('frame_id')
    inverted = LaunchConfiguration('inverted')
    angle_compensate = LaunchConfiguration('angle_compensate')
    scan_mode = LaunchConfiguration('scan_mode')
    scan_frequency = LaunchConfiguration('scan_frequency')

    channel_type_arg = DeclareLaunchArgument(
        'channel_type',
        default_value='serial',
        description='Specifying channel type of lidar',
    )
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Specifying usb port to connected lidar',
    )
    serial_baudrate_arg = DeclareLaunchArgument(
        'serial_baudrate',
        default_value='1000000',
        description='Specifying usb port baudrate to connected lidar',
    )
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='laser',
        description='Specifying frame_id of lidar',
    )
    inverted_arg = DeclareLaunchArgument(
        'inverted',
        default_value='false',
        description='Specifying whether or not to invert scan data',
    )
    angle_compensate_arg = DeclareLaunchArgument(
        'angle_compensate',
        default_value='true',
        description='Specifying whether or not to enable angle_compensate of scan data',
    )
    scan_mode_arg = DeclareLaunchArgument(
        'scan_mode',
        default_value='DenseBoost',
        description='Specifying scan mode of lidar',
    )
    scan_frequency_arg = DeclareLaunchArgument(
        'scan_frequency',
        default_value='15.0',
        description='Specifying scan frequency of lidar(Hz). S3 supports 10-20Hz',
    )

    # Nodes
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        output='screen',
        parameters=[
            {
                'channel_type': channel_type,
                'serial_port': serial_port,
                'serial_baudrate': serial_baudrate,
                'frame_id': frame_id,
                'inverted': inverted,
                'angle_compensate': angle_compensate,
                'scan_mode': scan_mode,
                'scan_frequency': scan_frequency,
            }
        ],
    )

    return LaunchDescription([
        channel_type_arg,
        serial_port_arg,
        serial_baudrate_arg,
        frame_id_arg,
        inverted_arg,
        angle_compensate_arg,
        scan_mode_arg,
        scan_frequency_arg,
        rplidar_node,
    ])
