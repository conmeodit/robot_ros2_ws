from setuptools import find_packages, setup
package_name = 'robot_mission'
setup(
    name=package_name, version='0.0.0', packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'], zip_safe=True, maintainer='Linh', maintainer_email='linh@todo.todo', description='Mission and autonomy nodes', license='Apache-2.0', tests_require=['pytest'],
    entry_points={'console_scripts': [
        'mission_manager_node = robot_mission.mission_manager_node:main',
        'autonomous_cleaning_node = robot_mission.autonomous_cleaning_node:main',
        'slam_session_manager_node = robot_mission.slam_session_manager_node:main',
    ]},
)
