import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    bringup_dir = get_package_share_directory('robot_bringup')
    desc_dir = get_package_share_directory('robot_description')

    # 1. Xử lý file URDF
    xacro_file = os.path.join(desc_dir, 'urdf', 'robot.urdf.xacro')
    doc = xacro.process_file(xacro_file)
    robot_description = {'robot_description': doc.toxml()}

    # Node State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # 2. Khởi chạy Lidar A1M8 bằng sllidar_ros2
    sllidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 115200,
            'frame_id': 'laser',
            'angle_compensate': True
        }],
        output='screen'
    )

    # 3. Tạo khung liên kết odom -> base_link
    static_tf_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    )

    # 4. SLAM Toolbox Node
    slam_params_file = os.path.join(bringup_dir, 'config', 'slam.yaml')
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py'
        )]),
        launch_arguments={'slam_params_file': slam_params_file}.items()
    )

    return LaunchDescription([
        robot_state_publisher,
        static_tf_odom,
        sllidar_node,
        slam_toolbox
    ])
