import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_vision')
    default_params = os.path.join(pkg_share, 'config', 'vision.yaml')
    default_model = os.path.join(pkg_share, 'models', 'best.pt')

    use_sim_time = LaunchConfiguration('use_sim_time')
    vision_params = LaunchConfiguration('vision_params')
    model_path = LaunchConfiguration('model_path')
    image_topic = LaunchConfiguration('image_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')

    return LaunchDescription(
        [
            DeclareLaunchArgument('use_sim_time', default_value='false'),
            DeclareLaunchArgument('vision_params', default_value=default_params),
            DeclareLaunchArgument('model_path', default_value=default_model),
            DeclareLaunchArgument('image_topic', default_value='/camera/image_raw'),
            DeclareLaunchArgument('camera_info_topic', default_value='/camera/camera_info'),
            Node(
                package='robot_vision',
                executable='robot_vision_node',
                name='robot_vision_node',
                output='screen',
                parameters=[
                    vision_params,
                    {
                        'use_sim_time': use_sim_time,
                        'model_path': model_path,
                        'image_topic': image_topic,
                        'camera_info_topic': camera_info_topic,
                    },
                ],
            ),
        ]
    )
