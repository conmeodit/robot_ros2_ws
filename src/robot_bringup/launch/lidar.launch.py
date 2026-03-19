from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_bringup')
    params_file = os.path.join(pkg_share, 'config', 'rplidar.yaml')

    laser_frame_arg = DeclareLaunchArgument('laser_frame', default_value='laser')
    laser_x_arg = DeclareLaunchArgument('laser_x', default_value='0.0')
    laser_y_arg = DeclareLaunchArgument('laser_y', default_value='0.0')
    laser_z_arg = DeclareLaunchArgument('laser_z', default_value='0.15')
    laser_roll_arg = DeclareLaunchArgument('laser_roll', default_value='0.0')
    laser_pitch_arg = DeclareLaunchArgument('laser_pitch', default_value='0.0')
    laser_yaw_arg = DeclareLaunchArgument('laser_yaw', default_value='0.0')

    rplidar = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        output='screen',
        parameters=[params_file, {'frame_id': LaunchConfiguration('laser_frame')}],
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_tf_pub',
        output='screen',
        arguments=[
            '--frame-id', 'base_link',
            '--child-frame-id', LaunchConfiguration('laser_frame'),
            '--x', LaunchConfiguration('laser_x'), '--y', LaunchConfiguration('laser_y'), '--z', LaunchConfiguration('laser_z'),
            '--roll', LaunchConfiguration('laser_roll'), '--pitch', LaunchConfiguration('laser_pitch'), '--yaw', LaunchConfiguration('laser_yaw')
        ]
    )

    return LaunchDescription([
        laser_frame_arg,
        laser_x_arg,
        laser_y_arg,
        laser_z_arg,
        laser_roll_arg,
        laser_pitch_arg,
        laser_yaw_arg,
        rplidar,
        static_tf,
    ])
