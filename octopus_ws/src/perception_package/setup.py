from setuptools import find_packages, setup

package_name = 'perception_package'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Eve (Drone) Team',
    maintainer_email='eve@octopus.dev',
    description='Perception package for drone image processing',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_detection_node = perception_package.object_detection_node:main',
            'trash_classifier_node = perception_package.trash_classifier_node:main',
            'surface_detection_node = perception_package.surface_detection_node:main',
            'decision_node = perception_package.decision_node:main',
        ],
    },
)

