import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('vacuum_driver')
    default_slam_params = os.path.join(pkg_share, 'config', 'slam.yaml')
    default_hardware_params = os.path.join(pkg_share, 'config', 'real_hardware.yaml')
    default_rviz_config = os.path.join(pkg_share, 'rviz', 'mapping.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    use_lidar = LaunchConfiguration('use_lidar')
    use_scan_filter = LaunchConfiguration('use_scan_filter')
    reset_slam = LaunchConfiguration('reset_slam')
    arduino_port = LaunchConfiguration('arduino_port')
    arduino_baudrate = LaunchConfiguration('arduino_baudrate')
    lidar_port = LaunchConfiguration('lidar_port')
    lidar_baudrate = LaunchConfiguration('lidar_baudrate')
    raw_scan_topic = LaunchConfiguration('raw_scan_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    slam_params = LaunchConfiguration('slam_params')
    hardware_params = LaunchConfiguration('hardware_params')
    rviz_config = LaunchConfiguration('rviz_config')

    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'mapping.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_rviz': use_rviz,
            'use_lidar': use_lidar,
            'use_scan_filter': use_scan_filter,
            'reset_slam': reset_slam,
            'arduino_port': arduino_port,
            'arduino_baudrate': arduino_baudrate,
            'lidar_port': lidar_port,
            'lidar_baudrate': lidar_baudrate,
            'raw_scan_topic': raw_scan_topic,
            'scan_topic': scan_topic,
            'slam_params': slam_params,
            'hardware_params': hardware_params,
            'rviz_config': rviz_config,
        }.items(),
    )

    autonomy_node = Node(
        package='vacuum_driver',
        executable='autonomous_cleaning_node',
        name='autonomous_cleaning_node',
        output='screen',
        parameters=[
            {
                'map_topic': '/map',
                'scan_topic': scan_topic,
                'cmd_vel_topic': '/cmd_vel',
                'map_frame': 'map',
                'base_frame': 'base_link',
                'robot_radius_m': 0.26,
                'front_stop_distance_m': 0.34,
                'emergency_stop_distance_m': 0.22,
                'side_clearance_m': 0.23,
                'max_linear_speed': 0.12,
                'min_linear_speed': 0.035,
                'max_angular_speed': 0.80,
                'frontier_min_cluster_size': 6,
                'frontier_relaxed_min_cluster_size': 3,
                'frontier_min_distance_m': 0.30,
                'map_stable_duration_sec': 10.0,
                'exploration_settle_sec': 7.0,
                'map_stable_origin_delta_m': 0.05,
                'coverage_spacing_m': 0.24,
                'coverage_visited_radius_m': 0.24,
                'coverage_required_ratio': 0.985,
                'coverage_switch_distance_weight': 0.12,
                'stuck_timeout_sec': 5.0,
                'stuck_min_progress_m': 0.08,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('use_sim_time', default_value='false'),
            DeclareLaunchArgument('use_rviz', default_value='true'),
            DeclareLaunchArgument('use_lidar', default_value='true'),
            DeclareLaunchArgument('use_scan_filter', default_value='true'),
            DeclareLaunchArgument('reset_slam', default_value='true'),
            DeclareLaunchArgument('arduino_port', default_value='/dev/rfcomm0'),
            DeclareLaunchArgument('arduino_baudrate', default_value='9600'),
            DeclareLaunchArgument('lidar_port', default_value='auto'),
            DeclareLaunchArgument('lidar_baudrate', default_value='115200'),
            DeclareLaunchArgument('raw_scan_topic', default_value='/scan/raw'),
            DeclareLaunchArgument('scan_topic', default_value='/scan'),
            DeclareLaunchArgument('slam_params', default_value=default_slam_params),
            DeclareLaunchArgument('hardware_params', default_value=default_hardware_params),
            DeclareLaunchArgument('rviz_config', default_value=default_rviz_config),
            mapping_launch,
            autonomy_node,
        ]
    )
