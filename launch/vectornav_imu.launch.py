from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package directory
    package_dir = get_package_share_directory('vectornav_imu_ros2')

    # Path to the config file
    config_file = os.path.join(package_dir, 'config', 'vectornav_imu_params.yaml')

    # Declare launch argument for config file path (optional override)
    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=config_file,
        description='Path to the configuration file'
    )

    # VectorNav IMU node
    vectornav_imu_node = Node(
        package='vectornav_imu_ros2',
        executable='vectornav_imu_node',
        name='vectornav_imu_publisher',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        remappings=[
            ('imu/data', 'imu/data')
        ]
    )

    return LaunchDescription([
        config_arg,
        vectornav_imu_node
    ])