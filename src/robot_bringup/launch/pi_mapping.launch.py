import glob
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def _detect_lidar_port():
    preferred_paths = sorted(glob.glob('/dev/serial/by-id/*'))
    if preferred_paths:
        return preferred_paths[0]

    tty_paths = sorted(glob.glob('/dev/ttyUSB*')) + sorted(glob.glob('/dev/ttyACM*'))
    if tty_paths:
        return tty_paths[0]

    return '/dev/ttyUSB0'


def launch_setup(context, *args, **kwargs):
    requested_port = LaunchConfiguration('serial_port').perform(context)
    serial_port = _detect_lidar_port() if requested_port == 'auto' else requested_port
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
