from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    robot_vision_dir = get_package_share_directory('robot_vision')

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

    return LaunchDescription([
        camera_node,
        detector_node,
    ])
