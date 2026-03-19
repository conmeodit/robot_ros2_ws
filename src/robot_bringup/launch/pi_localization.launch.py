from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
	slam_params = PathJoinSubstitution(
		[FindPackageShare('robot_bringup'), 'config', 'slam.yaml']
	)
	slam_launch = PathJoinSubstitution(
		[FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py']
	)

	use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')
	autostart_arg = DeclareLaunchArgument('autostart', default_value='true')

	slam_toolbox = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(slam_launch),
		launch_arguments={
			'slam_params_file': slam_params,
			'use_sim_time': LaunchConfiguration('use_sim_time'),
			'autostart': LaunchConfiguration('autostart'),
		}.items(),
	)

	return LaunchDescription([
		use_sim_time_arg,
		autostart_arg,
		slam_toolbox,
	])

