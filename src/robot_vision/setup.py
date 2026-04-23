from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'robot_vision'
setup(
    name=package_name, version='0.0.0', packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'models'), glob('models/*.pt')),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools', 'ultralytics>=8.0.0'], zip_safe=True, maintainer='Linh', maintainer_email='linh@todo.todo', description='TODO', license='TODO', tests_require=['pytest'],
    entry_points={'console_scripts': ['usb_camera_node = robot_vision.usb_camera_node:main', 'bottle_detector_node = robot_vision.bottle_detector_node:main']},
)
