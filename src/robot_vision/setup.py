from glob import glob

from setuptools import find_packages, setup


package_name = 'robot_vision'


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', glob('launch/*.launch.py')),
        (f'share/{package_name}/config', glob('config/*.yaml')),
        (f'share/{package_name}/models', glob('models/*')),
    ],
    install_requires=['setuptools', 'ultralytics>=8.0.0'],
    zip_safe=True,
    maintainer='cuong',
    maintainer_email='cuong@todo.todo',
    description='YOLO-based trash detection and ground projection for the vacuum robot.',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'robot_vision_node = robot_vision.vision_node:main',
        ],
    },
)
