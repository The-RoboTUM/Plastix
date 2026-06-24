import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'detection_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='victor-tipkemper',
    maintainer_email='victor.tipkemper@tum.de',
    description='Trash detection + localization node: subscribes to camera frames and publishes 2D positions.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'detector_node = detection_pkg.detector_node:main'
        ],
    },
)
