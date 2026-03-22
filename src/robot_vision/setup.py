from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Copy thư mục models chứa file best.pt vào install
        (os.path.join('share', package_name, 'models'), glob('models/*.pt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='linhpham',
    maintainer_email='hailinh13092006@gmail.com',
    description='ROS 2 package for robot vision and bottle detection',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Khai báo các node có thể chạy bằng ros2 run
            'usb_camera_node = robot_vision.usb_camera_node:main',
            'bottle_detector_node = robot_vision.bottle_detector_node:main',
        ],
    },
)