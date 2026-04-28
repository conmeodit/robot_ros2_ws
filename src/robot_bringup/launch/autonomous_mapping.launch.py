"""Autonomous mapping launch file for REAL HARDWARE.

Equivalent to ``ros2 launch vacuum_driver autonomous_mapping.launch.py``
but uses real hardware (Arduino Mega + RPLidar/SLLidar) instead of Webots.

Nodes launched:
  1. robot_state_publisher  — URDF → TF tree
  2. sllidar_node           — LiDAR → /scan
  3. mega_bridge_node       — Serial USB ↔ Arduino (CMD_VEL + encoder + IMU)
  4. wheel_odom_node        — encoder ticks → /odom + TF odom→base_link
  5. slam_toolbox           — SLAM: /scan + /odom → /map + TF map→odom
  6. slam_session_manager   — reset SLAM session at startup
  7. autonomous_cleaning    — frontier explore + coverage → /cmd_vel
  8. rviz2                  — visualization (optional)
"""

import glob
import os
import re

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# ---------------------------------------------------------------------------
# LiDAR auto-detection (reused from rplidar.launch.py)
# ---------------------------------------------------------------------------

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
    lidar_hints = ('lidar', 'rplidar', 'sllidar', 'slamtec', 'cp210', 'silicon_labs')
    for hint in lidar_hints:
        if hint in path:
            score += 30
    non_lidar_hints = ('arduino', 'mega', 'wch', 'ch340', 'usb-serial')
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
    seen = set()
    unique = []
    for c in candidates:
        if c not in seen:
            seen.add(c)
            unique.append(c)
    if not unique:
        return '/dev/ttyUSB0', []
    ranked = sorted(unique, key=lambda p: (_port_score(p), p), reverse=True)
    return ranked[0], unique


# ---------------------------------------------------------------------------
# Launch setup
# ---------------------------------------------------------------------------

def launch_setup(context, *args, **kwargs):
    bringup_dir = get_package_share_directory('robot_bringup')
    desc_dir = get_package_share_directory('robot_description')
    slam_share = get_package_share_directory('slam_toolbox')

    # Resolve parameters
    use_rviz = LaunchConfiguration('use_rviz')
    use_autonomy = LaunchConfiguration('use_autonomy')
    lidar_port_param = LaunchConfiguration('lidar_port').perform(context)
    lidar_baudrate = int(LaunchConfiguration('lidar_baudrate').perform(context))

    # --- URDF ---
    xacro_file = os.path.join(desc_dir, 'urdf', 'robot.urdf.xacro')
    robot_description_xml = xacro.process_file(xacro_file).toxml()

    # --- LiDAR port ---
    logs = []
    if lidar_port_param == 'auto':
        lidar_port, candidates = _detect_lidar_port()
        logs.append(LogInfo(
            msg=f'[autonomous_mapping] LiDAR port auto-detected: {lidar_port} '
                f'(candidates: {", ".join(candidates)})'
        ))
    else:
        lidar_port = lidar_port_param

    # --- Config paths ---
    slam_params_file = os.path.join(bringup_dir, 'config', 'slam.yaml')
    serial_bridge_config = os.path.join(bringup_dir, 'config', 'serial_bridge.yaml')
    localization_config = os.path.join(bringup_dir, 'config', 'localization.yaml')
    rviz_config = os.path.join(bringup_dir, 'rviz', 'robot.rviz')

    # ── 1. Robot State Publisher ──────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_xml}],
    )

    # ── 2. LiDAR Node (sllidar_ros2) ─────────────────────────────────────
    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[{
            'serial_port': lidar_port,
            'serial_baudrate': lidar_baudrate,
            'frame_id': 'laser',
            'angle_compensate': True,
        }],
        output='screen',
    )

    # ── 3. Mega Bridge Node (Serial ↔ Arduino) ───────────────────────────
    mega_bridge_node = Node(
        package='robot_serial_bridge',
        executable='mega_bridge_node',
        name='mega_bridge_node',
        output='screen',
        parameters=[serial_bridge_config],
    )

    # ── 4. Wheel Odometry Node ────────────────────────────────────────────
    wheel_odom_node = Node(
        package='robot_localization_pkg',
        executable='wheel_odom_node',
        name='wheel_odom_node',
        output='screen',
        parameters=[localization_config],
    )

    # ── 5. SLAM Toolbox ──────────────────────────────────────────────────
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_share, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': slam_params_file,
            'use_sim_time': 'false',
            'autostart': 'true',
            'use_lifecycle_manager': 'false',
        }.items(),
    )

    # ── 6. SLAM Session Manager (from vacuum_driver) ─────────────────────
    slam_session_manager = Node(
        package='vacuum_driver',
        executable='slam_session_manager_node',
        name='slam_session_manager_node',
        output='screen',
        parameters=[{
            'startup_delay_sec': 2.0,
            'service_wait_timeout_sec': 20.0,
            'shutdown_after_reset': True,
        }],
    )

    # ── 7. Autonomous Cleaning Node (from vacuum_driver) ──────────────────
    autonomy_node = Node(
        package='vacuum_driver',
        executable='autonomous_cleaning_node',
        name='autonomous_cleaning_node',
        output='screen',
        condition=IfCondition(use_autonomy),
        parameters=[{
            'map_topic': '/map',
            'scan_topic': '/scan',
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
        }],
    )

    # ── 8. RViz2 ─────────────────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(use_rviz),
    )

    return [
        *logs,
        robot_state_publisher,
        lidar_node,
        mega_bridge_node,
        wheel_odom_node,
        slam_launch,
        slam_session_manager,
        autonomy_node,
        rviz_node,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_autonomy', default_value='true'),
        DeclareLaunchArgument('lidar_port', default_value='auto',
                              description='Serial port for LiDAR, or auto'),
        DeclareLaunchArgument('lidar_baudrate', default_value='115200'),
        OpaqueFunction(function=launch_setup),
    ])
