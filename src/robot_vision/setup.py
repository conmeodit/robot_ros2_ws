from glob import glob
from setuptools import find_packages, setup

package_name = 'robot_vision'

setup(
	name=package_name,
	version='0.0.1',
	packages=find_packages(exclude=['test']),
	data_files=[
		('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
		(f'share/{package_name}', ['package.xml']),
		(f'share/{package_name}/models', glob('models/*')),
	],
	install_requires=['setuptools'],
	zip_safe=True,
	maintainer='linh-pham',
	maintainer_email='hailinh13092006@gmail.com',
	description='Vision nodes for USB camera and YOLO inference',
	license='Apache-2.0',
	tests_require=['pytest'],
	entry_points={
		'console_scripts': [
			'usb_camera_node = robot_vision.usb_camera_node:main',
			'bottle_detector_node = robot_vision.bottle_detector_node:main',
		],
	},
)

