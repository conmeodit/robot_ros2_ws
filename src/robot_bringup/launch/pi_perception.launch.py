from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
	config_file = PathJoinSubstitution(
		[FindPackageShare('robot_bringup'), 'config', 'usb_cam.yaml']
	)
	default_model = PathJoinSubstitution(
		[FindPackageShare('robot_vision'), 'models', 'best.pt']
	)

	enable_detector_arg = DeclareLaunchArgument(
		'enable_detector',
		default_value='true',
		description='Enable YOLO detector node',
	)
	model_path_arg = DeclareLaunchArgument(
		'model_path',
		default_value=default_model,
		description='Absolute path to YOLO model .pt file',
	)

	usb_camera = Node(
		package='robot_vision',
		executable='usb_camera_node',
		name='usb_camera_node',
		output='screen',
		parameters=[config_file],
	)

	detector = Node(
		package='robot_vision',
		executable='bottle_detector_node',
		name='bottle_detector_node',
		output='screen',
		parameters=[
			config_file,
			{
				'model_path': LaunchConfiguration('model_path'),
			},
		],
		condition=IfCondition(LaunchConfiguration('enable_detector')),
	)

	return LaunchDescription(
		[
			enable_detector_arg,
			model_path_arg,
			usb_camera,
			detector,
		]
	)

