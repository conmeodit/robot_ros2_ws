from urllib.parse import unquote, urlparse

from launch import LaunchDescription
from launch.actions import LogInfo, UnsetEnvironmentVariable
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def _cyclonedds_uri_to_path(uri: str) -> str | None:
    parsed = urlparse(uri)

    if parsed.scheme == 'file':
        return unquote(parsed.path)

    if parsed.scheme == '':
        return uri

    return None

def generate_launch_description():
    # Get package directory
    robot_vision_dir = get_package_share_directory('robot_vision')
    launch_actions = []

    cyclonedds_uri = os.environ.get('CYCLONEDDS_URI')
    if cyclonedds_uri:
        cyclonedds_path = _cyclonedds_uri_to_path(cyclonedds_uri)
        if cyclonedds_path and not os.path.exists(cyclonedds_path):
            launch_actions.extend([
                LogInfo(
                    msg=(
                        f"Unsetting invalid CYCLONEDDS_URI '{cyclonedds_uri}' "
                        f"because '{cyclonedds_path}' does not exist."
                    )
                ),
                UnsetEnvironmentVariable(name='CYCLONEDDS_URI'),
            ])

    # USB Camera Node
    camera_node = Node(
        package='robot_vision',
        executable='usb_camera_node',
        name='usb_camera_node',
        parameters=[
            {'camera_device': '/dev/video0'},
            {'frame_width': 640},
            {'frame_height': 480},
            {'fps': 30}
        ],
        output='screen'
    )

    # YOLO Detector Node
    detector_node = Node(
        package='robot_vision',
        executable='bottle_detector_node',
        name='bottle_detector_node',
        parameters=[
            {'model_path': os.path.join(robot_vision_dir, 'models', 'best.pt')},
            {'input_topic': '/camera/image_raw'},
            {'output_topic': '/detection/image_boxes'},
            {'confidence': 0.5}
        ],
        output='screen'
    )

    return LaunchDescription(launch_actions + [
        camera_node,
        detector_node,
    ])
