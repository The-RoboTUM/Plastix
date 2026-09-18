from setuptools import find_packages, setup

package_name = 'octopus_camera_transform'

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
    description='Camera marker transform node for Octopus mapping.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_marker_transform_node = octopus_camera_transform.camera_marker_transform_node:main',
            'flight_camera_transform_node = octopus_camera_transform.flight_camera_transform_node:main',
            'world_posearray_to_json_bridge_node = octopus_camera_transform.world_posearray_to_json_bridge_node:main',
            'local_camera_grid_node = octopus_camera_transform.local_camera_grid_node:main',
            'trash_gps_goal_node = octopus_camera_transform.trash_gps_goal_node:main',
        ],
    },
)
