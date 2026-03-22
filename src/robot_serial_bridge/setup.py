from setuptools import find_packages, setup
package_name = 'robot_serial_bridge'
setup(
    name=package_name, version='0.0.0', packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'], zip_safe=True, maintainer='Linh', maintainer_email='linh@todo.todo', description='TODO', license='TODO', tests_require=['pytest'],
    entry_points={'console_scripts': ['mega_bridge_node = robot_serial_bridge.mega_bridge_node:main']},
)
