from setuptools import find_packages, setup

package_name = 'octopus_detector_bridge'

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
    description='Bridge from detector PoseArray messages to Octopus detection JSON.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detector_posearray_bridge_node = octopus_detector_bridge.detector_posearray_bridge_node:main',
        ],
    },
)
