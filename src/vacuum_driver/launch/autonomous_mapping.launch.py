import glob
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _truthy(value):
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def _auto_arg(value):
    return str(value).strip().lower() in ('auto', 'probe')


def _canonical_device(path):
    try:
        return os.path.realpath(path)
    except Exception:
        return path


def _video_number(path):
    name = os.path.basename(_canonical_device(path))
    if name.startswith('video'):
        try:
            return int(name[5:])
        except ValueError:
            return 999
    return 999


def _camera_score(path):
    lowered = path.lower()
    score = 0
    if '/dev/v4l/by-id/' in lowered:
        score += 100
    if '/dev/v4l/by-path/' in lowered:
        score += 80
    if '/dev/video' in lowered:
        score += 20
    if 'video-index0' in lowered or 'index0' in lowered:
        score += 60
    if 'video-index1' in lowered or 'index1' in lowered:
        score -= 40
    if 'metadata' in lowered:
        score -= 80
    for hint in ('camera', 'webcam', 'uvc', 'usb'):
        if hint in lowered:
            score += 10
    return score


def _detect_camera_device(camera_device_arg):
    if camera_device_arg and not _auto_arg(camera_device_arg):
        return camera_device_arg, []

    candidates = []
    for pattern in ('/dev/v4l/by-id/*', '/dev/v4l/by-path/*', '/dev/video*'):
        candidates.extend(sorted(glob.glob(pattern)))

    unique = []
    seen_devices = set()
    for candidate in candidates:
        device = _canonical_device(candidate)
        if not os.path.exists(device):
            continue
        if device in seen_devices:
            continue
        unique.append(candidate)
        seen_devices.add(device)

    if not unique:
        return '/dev/video0', [
            '[vacuum_driver] camera_device=auto found no video devices; fallback to /dev/video0'
        ]

    selected = max(
        unique,
        key=lambda path: (_camera_score(path), -_video_number(path), path),
    )
    return selected, [
        '[vacuum_driver] camera_device=auto selected: {} (candidates: {})'.format(
            selected,
            ', '.join(unique),
        )
    ]


def camera_launch_setup(context, *args, **kwargs):
    use_camera = LaunchConfiguration('use_camera').perform(context)
    use_vision = LaunchConfiguration('use_vision').perform(context)
    camera_enabled = _truthy(use_camera) or (_auto_arg(use_camera) and _truthy(use_vision))
    if not camera_enabled:
        return []

    camera_device_arg = LaunchConfiguration('camera_device').perform(context)
    camera_device, logs = _detect_camera_device(camera_device_arg)
    camera_frame_id = LaunchConfiguration('camera_frame_id').perform(context)
    camera_image_topic = LaunchConfiguration('camera_image_topic').perform(context)
    camera_info_topic = LaunchConfiguration('camera_info_topic').perform(context)

    actions = [LogInfo(msg=log) for log in logs]
    actions.append(
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='downward_camera',
            output='screen',
            parameters=[
                {
                    'video_device': camera_device,
                    'camera_frame_id': camera_frame_id,
                    'image_size': [640, 480],
                    'time_per_frame': [1, 20],
                }
            ],
            remappings=[
                ('image_raw', camera_image_topic),
                ('camera_info', camera_info_topic),
            ],
        )
    )
    return actions


def generate_launch_description():
    pkg_share = get_package_share_directory('vacuum_driver')
    vision_share = get_package_share_directory('robot_vision')
    default_slam_params = os.path.join(pkg_share, 'config', 'slam.yaml')
    default_hardware_params = os.path.join(pkg_share, 'config', 'real_hardware.yaml')
    default_rviz_config = os.path.join(pkg_share, 'rviz', 'mapping.rviz')
    default_vision_params = os.path.join(vision_share, 'config', 'vision.yaml')
    default_vision_model = os.path.join(vision_share, 'models', 'best.pt')

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
    use_vision = LaunchConfiguration('use_vision')
    vision_params = LaunchConfiguration('vision_params')
    vision_model_path = LaunchConfiguration('vision_model_path')
    camera_image_topic = LaunchConfiguration('camera_image_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')

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

    vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(vision_share, 'launch', 'detection.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'vision_params': vision_params,
            'model_path': vision_model_path,
            'image_topic': camera_image_topic,
            'camera_info_topic': camera_info_topic,
        }.items(),
        condition=IfCondition(use_vision),
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
                'use_vision_obstacles': ParameterValue(use_vision, value_type=bool),
                'vision_obstacles_topic': '/vision/trash_obstacles',
                'vision_obstacle_ttl_sec': 8.0,
                'vision_obstacle_radius_m': 0.12,
                'vision_obstacle_min_confidence': 0.0,
                'map_frame': 'map',
                'base_frame': 'base_link',
                'robot_radius_m': 0.0,
                'robot_length_m': 0.35,
                'robot_width_m': 0.42,
                'camera_front_overhang_m': 0.08,
                'footprint_padding_m': 0.06,
                'lidar_offset_x_m': -0.10,
                'lidar_offset_y_m': 0.0,
                'obstacle_min_cluster_size': 4,
                'front_stop_distance_m': 0.18,
                'emergency_stop_distance_m': 0.10,
                'side_clearance_m': 0.10,
                'side_clearance_gain': 0.65,
                'side_clearance_deadband_m': 0.03,
                'front_block_heading_threshold_rad': 0.55,
                'max_linear_speed': 0.06,
                'min_linear_speed': 0.0175,
                'max_angular_speed': 0.40,
                'frontier_min_cluster_size': 6,
                'frontier_relaxed_min_cluster_size': 3,
                'frontier_min_distance_m': 0.30,
                'map_stable_duration_sec': 10.0,
                'exploration_settle_sec': 7.0,
                'frontier_force_coverage_sec': 5.0,
                'map_stable_origin_delta_m': 0.05,
                'coverage_spacing_m': 0.24,
                'coverage_visited_radius_m': 0.24,
                'coverage_required_ratio': 0.985,
                'coverage_switch_distance_weight': 0.12,
                'stuck_timeout_sec': 5.0,
                'stuck_min_progress_m': 0.08,
                'no_target_search_timeout_sec': 4.0,
                'escape_drive_duration_sec': 0.65,
                'backup_speed': 0.0275,
                'escape_drive_speed': 0.025,
                'search_turn_speed': 0.14,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('use_sim_time', default_value='false'),
            DeclareLaunchArgument('use_rviz', default_value='false'),
            DeclareLaunchArgument('use_lidar', default_value='true'),
            DeclareLaunchArgument('use_scan_filter', default_value='true'),
            DeclareLaunchArgument('reset_slam', default_value='true'),
            DeclareLaunchArgument('arduino_port', default_value='auto'),
            DeclareLaunchArgument('arduino_baudrate', default_value='auto'),
            DeclareLaunchArgument('lidar_port', default_value='auto'),
            DeclareLaunchArgument('lidar_baudrate', default_value='auto'),
            DeclareLaunchArgument('raw_scan_topic', default_value='/scan/raw'),
            DeclareLaunchArgument('scan_topic', default_value='/scan'),
            DeclareLaunchArgument('slam_params', default_value=default_slam_params),
            DeclareLaunchArgument('hardware_params', default_value=default_hardware_params),
            DeclareLaunchArgument('rviz_config', default_value=default_rviz_config),
            DeclareLaunchArgument('use_vision', default_value='false'),
            DeclareLaunchArgument('use_camera', default_value='auto'),
            DeclareLaunchArgument('camera_device', default_value='auto'),
            DeclareLaunchArgument('camera_frame_id', default_value='camera_link'),
            DeclareLaunchArgument('vision_params', default_value=default_vision_params),
            DeclareLaunchArgument('vision_model_path', default_value=default_vision_model),
            DeclareLaunchArgument('camera_image_topic', default_value='/camera/image_raw'),
            DeclareLaunchArgument('camera_info_topic', default_value='/camera/camera_info'),
            mapping_launch,
            OpaqueFunction(function=camera_launch_setup),
            vision_launch,
            autonomy_node,
        ]
    )
