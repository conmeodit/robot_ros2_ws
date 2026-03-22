from setuptools import find_packages, setup

package_name = 'robot_localization_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='linhpham',
    maintainer_email='hailinh13092006@gmail.com',
    description='ROS 2 package for robot localization',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Khai báo các node trong package này
            'imu_relay_node = robot_localization_pkg.imu_relay_node:main',
            'wheel_odom_node = robot_localization_pkg.wheel_odom_node:main',
        ],
    },
)