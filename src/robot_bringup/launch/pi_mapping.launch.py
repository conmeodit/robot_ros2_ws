from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sensors_launch = PathJoinSubstitution(
        [FindPackageShare('robot_bringup'), 'launch', 'pi_sensors.launch.py']
    )
    localization_launch = PathJoinSubstitution(
        [FindPackageShare('robot_bringup'), 'launch', 'pi_localization.launch.py']
    )

    laser_x_arg = DeclareLaunchArgument('laser_x', default_value='0.0')
    laser_y_arg = DeclareLaunchArgument('laser_y', default_value='0.0')
    laser_z_arg = DeclareLaunchArgument('laser_z', default_value='0.15')
    laser_roll_arg = DeclareLaunchArgument('laser_roll', default_value='0.0')
    laser_pitch_arg = DeclareLaunchArgument('laser_pitch', default_value='0.0')
    laser_yaw_arg = DeclareLaunchArgument('laser_yaw', default_value='0.0')

    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sensors_launch),
        launch_arguments={
            'laser_x': LaunchConfiguration('laser_x'),
            'laser_y': LaunchConfiguration('laser_y'),
            'laser_z': LaunchConfiguration('laser_z'),
            'laser_roll': LaunchConfiguration('laser_roll'),
            'laser_pitch': LaunchConfiguration('laser_pitch'),
            'laser_yaw': LaunchConfiguration('laser_yaw'),
        }.items(),
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localization_launch),
    )

    return LaunchDescription([
        laser_x_arg,
        laser_y_arg,
        laser_z_arg,
        laser_roll_arg,
        laser_pitch_arg,
        laser_yaw_arg,
        sensors,
        localization,
    ])
