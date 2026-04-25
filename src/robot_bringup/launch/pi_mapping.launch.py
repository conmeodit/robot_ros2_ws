import glob
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


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

    lidar_hints = (
        'lidar',
        'rplidar',
        'sllidar',
        'slamtec',
        'cp210',
        'silicon_labs',
    )
    for hint in lidar_hints:
        if hint in path:
            score += 30

    non_lidar_hints = (
        'arduino',
        'mega',
        'wch',
        'ch340',
        'usb-serial',
    )
    for hint in non_lidar_hints:
        if hint in path:
            score -= 60

    return score


def _detect_lidar_port():
    candidates = []
    candidates.extend(sorted(glob.glob('/dev/serial/by-id/*')))
    candidates.extend(sorted(glob.glob('/dev/serial/by-path/*')))
    candidates.extend(sorted(glob.glob('/dev/ttyUSB*')))
    candidates.extend(sorted(glob.glob('/dev/ttyACM*')))

    ordered_candidates = []
    seen = set()
    for candidate in candidates:
        if candidate not in seen:
            ordered_candidates.append(candidate)
            seen.add(candidate)

    if not ordered_candidates:
        return '/dev/ttyUSB0', []

    ranked_candidates = sorted(
        ordered_candidates,
        key=lambda port: (_port_score(port), port),
        reverse=True,
    )
    return ranked_candidates[0], ordered_candidates


def launch_setup(context, *args, **kwargs):
    requested_port = LaunchConfiguration('serial_port').perform(context)
    startup_logs = []
    if requested_port == 'auto':
        serial_port, candidates = _detect_lidar_port()
        if candidates:
            startup_logs.append(
                LogInfo(msg='[pi_mapping] serial_port=auto selected: {} (candidates: {})'.format(
                    serial_port,
                    ', '.join(candidates)
                ))
            )
        else:
            startup_logs.append(
                LogInfo(msg='[pi_mapping] serial_port=auto found no devices, fallback to /dev/ttyUSB0')
            )
    else:
        serial_port = requested_port
    serial_baudrate = int(LaunchConfiguration('serial_baudrate').perform(context))

    bringup_dir = get_package_share_directory('robot_bringup')
    desc_dir = get_package_share_directory('robot_description')

    xacro_file = os.path.join(desc_dir, 'urdf', 'robot.urdf.xacro')
    doc = xacro.process_file(xacro_file)
    robot_description = {'robot_description': doc.toxml()}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    sllidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[{
            'serial_port': serial_port,
            'serial_baudrate': serial_baudrate,
            'frame_id': 'laser',
            'angle_compensate': True
        }],
        output='screen'
    )

    static_tf_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    )

    slam_params_file = os.path.join(bringup_dir, 'config', 'slam.yaml')
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py'
        )]),
        launch_arguments={'slam_params_file': slam_params_file}.items()
    )

    return [
        *startup_logs,
        robot_state_publisher,
        static_tf_odom,
        sllidar_node,
        slam_toolbox,
    ]

def generate_launch_description():
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='auto',
        description='Serial port for the lidar device, or auto to detect it'
    )
    serial_baudrate_arg = DeclareLaunchArgument(
        'serial_baudrate',
        default_value='115200',
        description='Serial baudrate for the lidar device'
    )

    return LaunchDescription([
        serial_port_arg,
        serial_baudrate_arg,
        OpaqueFunction(function=launch_setup),
    ])
