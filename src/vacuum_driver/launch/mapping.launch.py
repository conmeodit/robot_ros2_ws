import glob
import os
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

try:
    import serial
except ImportError:
    serial = None


DEFAULT_LIDAR_BAUDRATES = (115200, 256000, 460800, 1000000)
DEFAULT_ARDUINO_BAUDRATES = (115200, 57600, 9600)
ARDUINO_HINTS = ('arduino', 'mega', 'ch340', 'wch', '1a86', 'usb-serial', 'usb_serial')
LIDAR_HINTS = ('lidar', 'rplidar', 'sllidar', 'slamtec', 'cp210', 'silicon_labs')


def _truthy(value):
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def _auto_arg(value):
    return str(value).strip().lower() in ('auto', 'probe')


def _canonical_device(path):
    try:
        return os.path.realpath(path)
    except Exception:
        return path


def _serial_candidates(include_rfcomm=False):
    patterns = [
        '/dev/serial/by-id/*',
        '/dev/serial/by-path/*',
        '/dev/ttyUSB*',
        '/dev/ttyACM*',
    ]
    if include_rfcomm:
        patterns.insert(0, '/dev/rfcomm*')

    candidates = []
    for pattern in patterns:
        candidates.extend(sorted(glob.glob(pattern)))

    ordered = []
    seen_devices = set()
    for candidate in candidates:
        device = _canonical_device(candidate)
        if device not in seen_devices:
            ordered.append(candidate)
            seen_devices.add(device)
    return ordered


def _same_serial_device(a, b):
    return _canonical_device(a) == _canonical_device(b)


def _lidar_port_score(port_path):
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
    for hint in LIDAR_HINTS:
        if hint in path:
            score += 30
    for hint in ARDUINO_HINTS + ('rfcomm',):
        if hint in path:
            score -= 60
    return score


def _arduino_port_score(port_path):
    path = port_path.lower()
    score = 0
    if '/dev/rfcomm' in path:
        score += 120
    if '/dev/serial/by-id/' in path:
        score += 100
    if '/dev/serial/by-path/' in path:
        score += 80
    if '/dev/ttyacm' in path:
        score += 35
    if '/dev/ttyusb' in path:
        score += 20
    for hint in ARDUINO_HINTS:
        if hint in path:
            score += 50
    for hint in LIDAR_HINTS:
        if hint in path:
            score -= 70
    return score


def _detect_lidar_candidates(exclude_ports=None):
    exclude_ports = exclude_ports or []
    candidates = _serial_candidates(include_rfcomm=False)
    filtered = []
    for candidate in candidates:
        if any(_same_serial_device(candidate, excluded) for excluded in exclude_ports if excluded):
            continue
        filtered.append(candidate)
    ranked = sorted(filtered, key=lambda port: (_lidar_port_score(port), port), reverse=True)
    return ranked, candidates


def _read_probe_response(ser, timeout_sec=0.65):
    deadline = time.monotonic() + timeout_sec
    response = bytearray()
    while time.monotonic() < deadline:
        chunk = ser.read(64)
        if chunk:
            response.extend(chunk)
            if _has_sllidar_descriptor(response):
                break
        else:
            time.sleep(0.02)
    return bytes(response)


def _has_sllidar_descriptor(response):
    if not response:
        return False
    start = response.find(b'\xa5\x5a')
    if start < 0 or len(response) < start + 7:
        return False
    descriptor = response[start:start + 7]
    response_len = (
        descriptor[2]
        | (descriptor[3] << 8)
        | (descriptor[4] << 16)
        | ((descriptor[5] & 0x3F) << 24)
    )
    response_type = descriptor[6]
    return (response_len, response_type) in ((20, 0x04), (3, 0x06))


def _probe_sllidar_baudrate(port, baudrate):
    if serial is None:
        return False, 'python3-serial is not available in the launch environment'

    try:
        with serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=0.05,
            write_timeout=0.2,
        ) as ser:
            time.sleep(0.08)
            try:
                ser.reset_input_buffer()
            except Exception:
                pass

            # Stop any previous scan, then request device info and health.
            ser.write(b'\xa5\x25')
            ser.flush()
            time.sleep(0.05)

            for command in (b'\xa5\x50', b'\xa5\x52'):
                try:
                    ser.reset_input_buffer()
                except Exception:
                    pass
                ser.write(command)
                ser.flush()
                response = _read_probe_response(ser)
                if _has_sllidar_descriptor(response):
                    return True, response[:12].hex(' ')
        return False, 'no SLLIDAR descriptor returned'
    except Exception as exc:
        return False, str(exc)


def _probe_arduino_baudrate(port, baudrate):
    if serial is None:
        return False, 'python3-serial is not available in the launch environment'

    try:
        with serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=0.05,
            write_timeout=0.2,
        ) as ser:
            time.sleep(0.25)
            try:
                ser.reset_input_buffer()
            except Exception:
                pass

            ser.write(b'PING\n')
            ser.flush()
            deadline = time.monotonic() + 1.0
            seen = []
            while time.monotonic() < deadline:
                line = ser.readline()
                if not line:
                    time.sleep(0.02)
                    continue
                text = line.decode(errors='ignore').strip()
                if not text:
                    continue
                seen.append(text)
                if text.startswith(('PONG', 'BOOT', 'READY', 'STAT')):
                    return True, text
        return False, 'no Arduino protocol response' + (': ' + '; '.join(seen[:3]) if seen else '')
    except Exception as exc:
        return False, str(exc)


def _resolve_arduino_baudrate(port, baudrate_arg):
    if not _auto_arg(baudrate_arg):
        return int(baudrate_arg), []

    failures = []
    for baudrate in DEFAULT_ARDUINO_BAUDRATES:
        ok, detail = _probe_arduino_baudrate(port, baudrate)
        if ok:
            return baudrate, [
                '[vacuum_driver] arduino_baudrate=auto selected: {} on {} '
                '(probe response: {})'.format(baudrate, port, detail)
            ]
        failures.append('{}: {}'.format(baudrate, detail))

    fallback = DEFAULT_ARDUINO_BAUDRATES[0]
    return fallback, [
        '[vacuum_driver] arduino_baudrate=auto could not verify Arduino on {}. '
        'Fallback to {}. Probe failures: {}'.format(port, fallback, '; '.join(failures))
    ]


def _resolve_arduino_port_and_baudrate(port_arg, baudrate_arg):
    if port_arg and not _auto_arg(port_arg):
        baudrate, logs = _resolve_arduino_baudrate(port_arg, baudrate_arg)
        return port_arg, baudrate, logs

    candidates = _serial_candidates(include_rfcomm=True)
    if not candidates:
        fallback_port = '/dev/rfcomm0'
        fallback_baudrate = DEFAULT_ARDUINO_BAUDRATES[0]
        return fallback_port, fallback_baudrate, [
            '[vacuum_driver] arduino auto-detect found no serial devices. '
            'Fallback to {} at {} baud.'.format(fallback_port, fallback_baudrate)
        ]

    ranked = sorted(candidates, key=lambda port: (_arduino_port_score(port), port), reverse=True)
    baudrates = (
        DEFAULT_ARDUINO_BAUDRATES
        if _auto_arg(baudrate_arg)
        else (int(baudrate_arg),)
    )

    failures = []
    for port in ranked:
        for baudrate in baudrates:
            ok, detail = _probe_arduino_baudrate(port, baudrate)
            if ok:
                return port, baudrate, [
                    '[vacuum_driver] Arduino auto-detect selected: {} at {} baud '
                    '(probe response: {}; candidates: {})'.format(
                        port,
                        baudrate,
                        detail,
                        ', '.join(ranked),
                    )
                ]
            failures.append('{}@{}: {}'.format(port, baudrate, detail))

    fallback_port = ranked[0]
    fallback_baudrate = DEFAULT_ARDUINO_BAUDRATES[0]
    return fallback_port, fallback_baudrate, [
        '[vacuum_driver] Arduino auto-detect could not verify the protocol on any serial '
        'device. Fallback to {} at {} baud. Probe failures: {}'.format(
            fallback_port,
            fallback_baudrate,
            '; '.join(failures),
        )
    ]


def _resolve_lidar_baudrate(port, baudrate_arg):
    value = str(baudrate_arg).strip().lower()
    if not _auto_arg(value):
        return int(baudrate_arg), None

    failures = []
    for baudrate in DEFAULT_LIDAR_BAUDRATES:
        ok, detail = _probe_sllidar_baudrate(port, baudrate)
        if ok:
            return (
                baudrate,
                '[vacuum_driver] lidar_baudrate=auto selected: {} on {} '
                '(probe response: {})'.format(baudrate, port, detail),
            )
        failures.append('{}: {}'.format(baudrate, detail))

    fallback = DEFAULT_LIDAR_BAUDRATES[0]
    return (
        fallback,
        '[vacuum_driver] lidar_baudrate=auto could not verify a SLLIDAR on {}. '
        'Fallback to {}. Probe failures: {}'.format(port, fallback, '; '.join(failures)),
    )


def _resolve_lidar_port_and_baudrate(port_arg, baudrate_arg, exclude_ports=None):
    if port_arg and not _auto_arg(port_arg):
        if not _auto_arg(baudrate_arg):
            return port_arg, int(baudrate_arg), []
        baudrate, log = _resolve_lidar_baudrate(port_arg, baudrate_arg)
        return port_arg, baudrate, [log] if log else []

    ranked, candidates = _detect_lidar_candidates(exclude_ports=exclude_ports)
    if not ranked:
        fallback_port = '/dev/ttyUSB0'
        fallback_baudrate = DEFAULT_LIDAR_BAUDRATES[0]
        return fallback_port, fallback_baudrate, [
            '[vacuum_driver] lidar_port=auto found no non-Arduino serial devices. '
            'Fallback to {} at {} baud. All candidates: {}'.format(
                fallback_port,
                fallback_baudrate,
                ', '.join(candidates) if candidates else '<none>',
            )
        ]

    if not _auto_arg(baudrate_arg):
        selected = ranked[0]
        baudrate = int(baudrate_arg)
        return selected, baudrate, [
            '[vacuum_driver] lidar_port=auto selected by port hints: {} at {} baud '
            '(non-Arduino candidates: {})'.format(
                selected,
                baudrate,
                ', '.join(ranked),
            )
        ]

    failures = []
    for port in ranked:
        for baudrate in DEFAULT_LIDAR_BAUDRATES:
            ok, detail = _probe_sllidar_baudrate(port, baudrate)
            if ok:
                return port, baudrate, [
                    '[vacuum_driver] lidar auto-detect selected: {} at {} baud '
                    '(probe response: {}; non-Arduino candidates: {})'.format(
                        port,
                        baudrate,
                        detail,
                        ', '.join(ranked),
                    )
                ]
            failures.append('{}@{}: {}'.format(port, baudrate, detail))

    fallback_port = ranked[0]
    fallback_baudrate = DEFAULT_LIDAR_BAUDRATES[0]
    return fallback_port, fallback_baudrate, [
        '[vacuum_driver] lidar auto-detect could not verify a SLLIDAR on any non-Arduino '
        'serial device. Fallback to {} at {} baud. Probe failures: {}'.format(
            fallback_port,
            fallback_baudrate,
            '; '.join(failures),
        )
    ]


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
    arduino_port_arg = LaunchConfiguration('arduino_port').perform(context)
    arduino_baudrate_arg = LaunchConfiguration('arduino_baudrate').perform(context)
    lidar_port_arg = LaunchConfiguration('lidar_port').perform(context)
    lidar_baudrate_arg = LaunchConfiguration('lidar_baudrate').perform(context)
    raw_scan_topic = LaunchConfiguration('raw_scan_topic').perform(context)
    scan_topic = LaunchConfiguration('scan_topic').perform(context)

    actions = []
    arduino_port, arduino_baudrate, arduino_logs = _resolve_arduino_port_and_baudrate(
        arduino_port_arg,
        arduino_baudrate_arg,
    )
    for log in arduino_logs:
        actions.append(LogInfo(msg=log))

    lidar_port = None
    lidar_baudrate = None
    if use_lidar:
        lidar_port, lidar_baudrate, lidar_logs = _resolve_lidar_port_and_baudrate(
            lidar_port_arg,
            lidar_baudrate_arg,
            exclude_ports=[arduino_port],
        )
        for log in lidar_logs:
            actions.append(LogInfo(msg=log))

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
                'max_linear_speed_mps': 0.08,
                'max_angular_speed_radps': 0.425,
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
            DeclareLaunchArgument('arduino_port', default_value='auto'),
            DeclareLaunchArgument('arduino_baudrate', default_value='auto'),
            DeclareLaunchArgument('lidar_port', default_value='auto'),
            DeclareLaunchArgument('lidar_baudrate', default_value='auto'),
            DeclareLaunchArgument('raw_scan_topic', default_value='/scan/raw'),
            DeclareLaunchArgument('scan_topic', default_value='/scan'),
            DeclareLaunchArgument('slam_params', default_value=default_slam_params),
            DeclareLaunchArgument('hardware_params', default_value=default_hardware_params),
            DeclareLaunchArgument('rviz_config', default_value=default_rviz_config),
            OpaqueFunction(function=launch_setup),
        ]
    )
