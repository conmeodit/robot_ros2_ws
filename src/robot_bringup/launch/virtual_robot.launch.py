import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('robot_bringup')
    desc_dir = get_package_share_directory('robot_description')

    xacro_file = os.path.join(desc_dir, 'urdf', 'robot.urdf.xacro')
    robot_description_xml = xacro.process_file(xacro_file).toxml()
    robot_description = {'robot_description': robot_description_xml}

    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    with_rviz = LaunchConfiguration('with_rviz')

    declare_x = DeclareLaunchArgument('x', default_value='0.0')
    declare_y = DeclareLaunchArgument('y', default_value='0.0')
    declare_z = DeclareLaunchArgument('z', default_value='0.0')
    declare_yaw = DeclareLaunchArgument('yaw', default_value='0.0')
    declare_with_rviz = DeclareLaunchArgument('with_rviz', default_value='true')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    )

    odom_to_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[x, y, z, '0', '0', yaw, 'odom', 'base_link'],
    )

    rviz_config = os.path.join(bringup_dir, 'rviz', 'robot.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(with_rviz),
    )

    return LaunchDescription([
        declare_x,
        declare_y,
        declare_z,
        declare_yaw,
        declare_with_rviz,
        robot_state_publisher,
        map_to_odom,
        odom_to_base_link,
        rviz_node,
    ])
