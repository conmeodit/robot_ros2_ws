import glob
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _truthy(value):
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def _port_score(port_path):
    path = port_path.lower()
    score = 0
    if '/dev/serial/by-id/' in path:
        score += 100
    if '/dev/serial/by-path/' in path:
        score += 80
    if '/dev/ttyusb' in path:
        score += 20
    if '/dev/ttyacm' in path:
        score += 10
    for hint in ('lidar', 'rplidar', 'sllidar', 'slamtec', 'cp210', 'silicon_labs'):
        if hint in path:
            score += 30
    for hint in ('arduino', 'mega', 'wch', 'ch340', 'usb-serial', 'rfcomm'):
        if hint in path:
            score -= 60
    return score


def _detect_lidar_port():
    candidates = []
    candidates.extend(sorted(glob.glob('/dev/serial/by-id/*')))
    candidates.extend(sorted(glob.glob('/dev/serial/by-path/*')))
    candidates.extend(sorted(glob.glob('/dev/ttyUSB*')))
    candidates.extend(sorted(glob.glob('/dev/ttyACM*')))
    ordered = []
    seen = set()
    for candidate in candidates:
        if candidate not in seen:
            ordered.append(candidate)
            seen.add(candidate)
    if not ordered:
        return '/dev/ttyUSB0', []
    ranked = sorted(ordered, key=lambda port: (_port_score(port), port), reverse=True)
    return ranked[0], ordered


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('vacuum_driver')
    slam_share = get_package_share_directory('slam_toolbox')
    default_urdf = os.path.join(pkg_share, 'urdf', 'vacuum_robot.urdf')

    use_sim_time_arg = LaunchConfiguration('use_sim_time').perform(context)
    use_sim_time = _truthy(use_sim_time_arg)
    use_rviz = _truthy(LaunchConfiguration('use_rviz').perform(context))
    use_lidar = _truthy(LaunchConfiguration('use_lidar').perform(context))
    use_scan_filter = _truthy(LaunchConfiguration('use_scan_filter').perform(context))
    reset_slam = _truthy(LaunchConfiguration('reset_slam').perform(context))
    slam_params = LaunchConfiguration('slam_params').perform(context)
    hardware_params = LaunchConfiguration('hardware_params').perform(context)
    rviz_config = LaunchConfiguration('rviz_config').perform(context)
    arduino_port = LaunchConfiguration('arduino_port').perform(context)
    arduino_baudrate = int(LaunchConfiguration('arduino_baudrate').perform(context))
    lidar_port_arg = LaunchConfiguration('lidar_port').perform(context)
    lidar_baudrate = int(LaunchConfiguration('lidar_baudrate').perform(context))
    raw_scan_topic = LaunchConfiguration('raw_scan_topic').perform(context)
    scan_topic = LaunchConfiguration('scan_topic').perform(context)

    actions = []
    if lidar_port_arg == 'auto':
        lidar_port, candidates = _detect_lidar_port()
        if candidates:
            actions.append(
                LogInfo(
                    msg='[vacuum_driver] lidar_port=auto selected: {} (candidates: {})'.format(
                        lidar_port,
                        ', '.join(candidates),
                    )
                )
            )
        else:
            actions.append(
                LogInfo(
                    msg='[vacuum_driver] lidar_port=auto found no devices, fallback to /dev/ttyUSB0'
                )
            )
    else:
        lidar_port = lidar_port_arg

    driver_node = Node(
        package='vacuum_driver',
        executable='real_driver',
        name='real_hardware_driver',
        output='screen',
        parameters=[
            hardware_params,
            {
                'port': arduino_port,
                'baudrate': arduino_baudrate,
                'cmd_vel_topic': '/cmd_vel',
                'odom_topic': '/odom',
                'odom_raw_topic': '/odom/raw',
                'imu_topic': '/imu/data',
                'imu_raw_topic': '/imu/data_raw',
                'publish_odom_tf': True,
                'publish_lidar_tf': False,
                'base_frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'imu_frame_id': 'base_link',
                'lidar_frame_id': 'laser',
                'wheel_radius': 0.0425,
                'wheel_separation': 0.42,
                'encoder_ticks_per_rev': 1320.0,
                'auto_detect_encoder_direction': True,
                'encoder_direction_min_delta_ticks': 3,
                'max_linear_speed_mps': 0.16,
                'max_angular_speed_radps': 0.85,
            },
        ],
    )

    with open(default_urdf, 'r', encoding='utf-8') as urdf_file:
        robot_description = urdf_file.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }
        ],
    )
    actions.extend([driver_node, robot_state_publisher_node])

    if use_lidar:
        lidar_output_topic = raw_scan_topic if use_scan_filter else scan_topic
        actions.append(
            Node(
                package='sllidar_ros2',
                executable='sllidar_node',
                name='sllidar_node',
                output='screen',
                parameters=[
                    {
                        'serial_port': lidar_port,
                        'serial_baudrate': lidar_baudrate,
                        'frame_id': 'laser',
                        'angle_compensate': True,
                    }
                ],
                remappings=[('/scan', lidar_output_topic), ('scan', lidar_output_topic)],
            )
        )

    if use_scan_filter:
        actions.append(
            Node(
                package='vacuum_driver',
                executable='scan_filter_node',
                name='scan_filter_node',
                output='screen',
                parameters=[
                    hardware_params,
                    {
                        'input_topic': raw_scan_topic,
                        'output_topic': scan_topic,
                        'reverse_scan': False,
                        'scan_filter_enabled': True,
                        'scan_filter_window': 5,
                        'scan_filter_min_valid_neighbors': 3,
                        'scan_filter_outlier_threshold_m': 0.12,
                        'scan_filter_temporal_alpha': 0.60,
                        'scan_filter_temporal_jump_threshold_m': 0.20,
                    },
                ],
            )
        )

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_share, 'launch', 'online_async_launch.py')
            ),
            launch_arguments={
                'slam_params_file': slam_params,
                'use_sim_time': use_sim_time_arg,
                'autostart': 'true',
                'use_lifecycle_manager': 'false',
            }.items(),
        )
    )

    if reset_slam:
        actions.append(
            Node(
                package='vacuum_driver',
                executable='slam_session_manager_node',
                name='slam_session_manager_node',
                output='screen',
                parameters=[
                    {
                        'startup_delay_sec': 2.0,
                        'service_wait_timeout_sec': 20.0,
                        'shutdown_after_reset': True,
                    }
                ],
            )
        )

    if use_rviz:
        actions.append(
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config],
                output='screen',
            )
        )
    return actions


def generate_launch_description():
    pkg_share = get_package_share_directory('vacuum_driver')
    default_slam_params = os.path.join(pkg_share, 'config', 'slam.yaml')
    default_hardware_params = os.path.join(pkg_share, 'config', 'real_hardware.yaml')
    default_rviz_config = os.path.join(pkg_share, 'rviz', 'mapping.rviz')

    return LaunchDescription(
        [
            DeclareLaunchArgument('use_sim_time', default_value='false'),
            DeclareLaunchArgument('use_rviz', default_value='false'),
            DeclareLaunchArgument('use_lidar', default_value='true'),
            DeclareLaunchArgument('use_scan_filter', default_value='true'),
            DeclareLaunchArgument('reset_slam', default_value='true'),
            DeclareLaunchArgument('arduino_port', default_value='/dev/ttyUSB0'),
            DeclareLaunchArgument('arduino_baudrate', default_value='115200'),
            DeclareLaunchArgument('lidar_port', default_value='auto'),
            DeclareLaunchArgument('lidar_baudrate', default_value='115200'),
            DeclareLaunchArgument('raw_scan_topic', default_value='/scan/raw'),
            DeclareLaunchArgument('scan_topic', default_value='/scan'),
            DeclareLaunchArgument('slam_params', default_value=default_slam_params),
            DeclareLaunchArgument('hardware_params', default_value=default_hardware_params),
            DeclareLaunchArgument('rviz_config', default_value=default_rviz_config),
            OpaqueFunction(function=launch_setup),
        ]
    )
