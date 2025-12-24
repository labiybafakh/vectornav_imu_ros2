from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyUSB0',
        description='Serial port for VectorNav sensor'
    )

    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='Baud rate for VectorNav sensor'
    )

    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='imu_link',
        description='Frame ID for IMU messages'
    )

    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='100.0',
        description='Publishing rate in Hz'
    )

    # VectorNav IMU node
    vectornav_imu_node = Node(
        package='vectornav_imu_ros2',
        executable='vectornav_imu_node',
        name='vectornav_imu_publisher',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baud_rate': LaunchConfiguration('baud_rate'),
            'frame_id': LaunchConfiguration('frame_id'),
            'publish_rate': LaunchConfiguration('publish_rate'),
        }],
        remappings=[
            ('imu/data', 'imu/data')
        ]
    )

    return LaunchDescription([
        port_arg,
        baud_rate_arg,
        frame_id_arg,
        publish_rate_arg,
        vectornav_imu_node
    ])