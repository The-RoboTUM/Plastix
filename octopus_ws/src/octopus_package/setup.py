from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'octopus_package'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Octopus Brain Team',
    maintainer_email='team@octopus.dev',
    description='Octopus Brain - Central coordination node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'octopus_main_node = octopus_package.octopus_main_node:main',
            'task_database_node = octopus_package.task_database_node:main',
            'location_database_node = octopus_package.location_database_node:main',
            'rtk_gps_node = octopus_package.rtk_gps_node:main',
        ],
    },
)

