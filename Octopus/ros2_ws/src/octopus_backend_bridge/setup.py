from setuptools import find_packages, setup

package_name = 'octopus_backend_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='PlastiX Octopus Team',
    maintainer_email='dominik.sandner@tum.de',
    description='ROS2 to backend bridge for Octopus map patches.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_patch_backend_bridge_node = octopus_backend_bridge.map_patch_backend_bridge_node:main',
            'camera_transform_status_backend_bridge_node = octopus_backend_bridge.camera_transform_status_backend_bridge_node:main',
            'camera_debug_backend_bridge_node = octopus_backend_bridge.camera_debug_backend_bridge_node:main',
            'local_camera_grid_backend_bridge_node = octopus_backend_bridge.local_camera_grid_backend_bridge_node:main',
            'eve_fake_gps_bridge_node = octopus_backend_bridge.eve_fake_gps_bridge_node:main',
        ],
    },
)
