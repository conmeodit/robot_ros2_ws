from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('robot_bringup')
    params_file = os.path.join(pkg_share, 'config', 'rplidar.yaml')

    rplidar = Node(
        package='rplidar_ros',
        executable='rplidar_node',   # nếu executable khác, ta sẽ đổi sau
        name='rplidar_node',
        output='screen',
        parameters=[params_file],
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_tf_pub',
        output='screen',
        arguments=[
            '--frame-id', 'base_link',
            '--child-frame-id', 'laser',
            '--x', '0.0', '--y', '0.0', '--z', '0.15',
            '--roll', '0', '--pitch', '0', '--yaw', '0'
        ]
    )

    return LaunchDescription([rplidar, static_tf])
