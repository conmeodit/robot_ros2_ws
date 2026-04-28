import glob
import os
import re

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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

    lidar_hints = ('lidar', 'rplidar', 'slamtec', 'ch340', '1a86')
    for hint in lidar_hints:
        if hint in path:
            score += 30

    non_lidar_hints = ('arduino', 'mega', 'cp210', 'silicon_labs')
    for hint in non_lidar_hints:
        if hint in path:
            score -= 60

    return score


def _normalize_port_path(port_path):
    if not port_path:
        return None
    return os.path.realpath(port_path)


def _read_configured_port(config_path):
    pattern = re.compile(r'^\s*port:\s*(\S+)\s*$')
    try:
        with open(config_path, 'r', encoding='utf-8') as config_file:
            for line in config_file:
                match = pattern.match(line)
                if match:
                    return match.group(1)
    except OSError:
        return None
    return None


def _detect_lidar_port(excluded_ports=None):
    normalized_excluded = {
        _normalize_port_path(port)
        for port in (excluded_ports or [])
        if _normalize_port_path(port)
    }

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

    filtered_candidates = [
        candidate
        for candidate in ordered_candidates
        if _normalize_port_path(candidate) not in normalized_excluded
    ]

    if not filtered_candidates:
        return None, ordered_candidates, []

    ranked = sorted(filtered_candidates, key=lambda port: (_port_score(port), port), reverse=True)
    return ranked[0], ordered_candidates, filtered_candidates


def launch_setup(context, *args, **kwargs):
    requested_port = LaunchConfiguration('serial_port').perform(context)
    baudrate = int(LaunchConfiguration('serial_baudrate').perform(context))
    scan_mode = LaunchConfiguration('scan_mode').perform(context)
    frame_id = LaunchConfiguration('frame_id').perform(context)

    bringup_dir = os.path.dirname(os.path.dirname(__file__))
    serial_bridge_config = os.path.join(bringup_dir, 'config', 'serial_bridge.yaml')
    mega_port = _read_configured_port(serial_bridge_config)

    logs = []
    if requested_port == 'auto':
        serial_port, candidates, usable_candidates = _detect_lidar_port(excluded_ports=[mega_port])
        if serial_port:
            logs.append(
                LogInfo(
                    msg='[rplidar] serial_port=auto selected: {} (candidates: {})'.format(
                        serial_port,
                        ', '.join(usable_candidates),
                    )
                )
            )
        else:
            logs.append(
                LogInfo(
                    msg='[rplidar] serial_port=auto found no distinct lidar device. '
                    'excluded Mega port: {}. raw candidates: {}'.format(
                        mega_port or '(none)',
                        ', '.join(candidates) if candidates else '(none)',
                    )
                )
            )
    else:
        serial_port = requested_port

    actions = list(logs)
    if serial_port:
        actions.append(
            Node(
                package='rplidar_ros',
                executable='rplidar_node',
                name='rplidar_node',
                parameters=[{
                    'channel_type': 'serial',
                    'serial_port': serial_port,
                    'serial_baudrate': baudrate,
                    'frame_id': frame_id,
                    'inverted': False,
                    'angle_compensate': True,
                    'scan_mode': scan_mode,
                }],
                output='screen',
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value='auto',
            description='Serial port for RPLIDAR device, or auto to detect',
        ),
        DeclareLaunchArgument(
            'serial_baudrate',
            default_value='115200',
            description='RPLIDAR serial baudrate',
        ),
        DeclareLaunchArgument(
            'scan_mode',
            default_value='Standard',
            description='RPLIDAR scan mode',
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='laser',
            description='Laser frame id',
        ),
        OpaqueFunction(function=launch_setup),
    ])
