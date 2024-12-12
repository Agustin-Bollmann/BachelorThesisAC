from setuptools import find_packages, setup

package_name = 'can_bridge'

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
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'bridge_control_to_can = can_bridge.bridge_control_to_can:main',
        'bridge_lidar_to_can = can_bridge.bridge_lidar_to_can:main',
        'bridge_gnss_to_can = can_bridge.bridge_gnss_to_can:main',
        'bridge_camera_to_can = can_bridge.bridge_camera_to_can:main',
        'bridge_can_to_control = can_bridge.bridge_can_to_control:main'     
        ],
    },
)
