from setuptools import find_packages, setup

package_name = 'robot_serial_bridge'

setup(
	name=package_name,
	version='0.0.1',
	packages=find_packages(exclude=['test']),
	data_files=[
		('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
		(f'share/{package_name}', ['package.xml']),
	],
	install_requires=['setuptools'],
	zip_safe=True,
	maintainer='linh-pham',
	maintainer_email='hailinh13092006@gmail.com',
	description='Serial bridge package for base controller',
	license='Apache-2.0',
	tests_require=['pytest'],
	entry_points={
		'console_scripts': [
			'mega_bridge_node = robot_serial_bridge.mega_bridge_node:main',
		],
	},
)

