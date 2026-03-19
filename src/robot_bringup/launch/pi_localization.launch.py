from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
	slam_params = PathJoinSubstitution(
		[FindPackageShare('robot_bringup'), 'config', 'slam.yaml']
	)

	use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')

	slam_toolbox = Node(
		package='slam_toolbox',
		executable='async_slam_toolbox_node',
		name='slam_toolbox',
		output='screen',
		parameters=[
			slam_params,
			{'use_sim_time': LaunchConfiguration('use_sim_time')},
		],
	)

	return LaunchDescription([
		use_sim_time_arg,
		slam_toolbox,
	])

