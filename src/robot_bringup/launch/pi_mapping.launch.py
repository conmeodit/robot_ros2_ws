import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    vacuum_share = get_package_share_directory('vacuum_driver')
    mapping_launch = os.path.join(vacuum_share, 'launch', 'mapping.launch.py')
    default_slam_params = os.path.join(vacuum_share, 'config', 'slam.yaml')
    default_hardware_params = os.path.join(vacuum_share, 'config', 'real_hardware.yaml')
    default_rviz_config = os.path.join(vacuum_share, 'rviz', 'mapping.rviz')

    return LaunchDescription(
        [
            DeclareLaunchArgument('use_sim_time', default_value='false'),
            DeclareLaunchArgument('use_rviz', default_value='false'),
            DeclareLaunchArgument('use_lidar', default_value='true'),
            DeclareLaunchArgument('use_scan_filter', default_value='true'),
            DeclareLaunchArgument('reset_slam', default_value='true'),
            DeclareLaunchArgument('arduino_port', default_value='auto'),
            DeclareLaunchArgument('arduino_baudrate', default_value='115200'),
            DeclareLaunchArgument('lidar_port', default_value='auto'),
            DeclareLaunchArgument('lidar_baudrate', default_value='auto'),
            DeclareLaunchArgument('raw_scan_topic', default_value='/scan/raw'),
            DeclareLaunchArgument('scan_topic', default_value='/scan'),
            DeclareLaunchArgument('slam_params', default_value=default_slam_params),
            DeclareLaunchArgument('hardware_params', default_value=default_hardware_params),
            DeclareLaunchArgument('rviz_config', default_value=default_rviz_config),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(mapping_launch),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'use_rviz': LaunchConfiguration('use_rviz'),
                    'use_lidar': LaunchConfiguration('use_lidar'),
                    'use_scan_filter': LaunchConfiguration('use_scan_filter'),
                    'reset_slam': LaunchConfiguration('reset_slam'),
                    'arduino_port': LaunchConfiguration('arduino_port'),
                    'arduino_baudrate': LaunchConfiguration('arduino_baudrate'),
                    'lidar_port': LaunchConfiguration('lidar_port'),
                    'lidar_baudrate': LaunchConfiguration('lidar_baudrate'),
                    'raw_scan_topic': LaunchConfiguration('raw_scan_topic'),
                    'scan_topic': LaunchConfiguration('scan_topic'),
                    'slam_params': LaunchConfiguration('slam_params'),
                    'hardware_params': LaunchConfiguration('hardware_params'),
                    'rviz_config': LaunchConfiguration('rviz_config'),
                }.items(),
            ),
        ]
    )
