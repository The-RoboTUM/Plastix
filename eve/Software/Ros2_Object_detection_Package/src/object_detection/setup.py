from setuptools import find_packages, setup

package_name = 'object_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    package_data={  # <<< HIER
        'object_detection': ['_classes.csv', 'Config.yaml'],
    },
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jan',
    maintainer_email='jan.wech@gmx.de',
    description='ITQ-Project: Object Detection and Coordinate Transformation',
    license='Apache License 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'inference_node = object_detection.inference_node:main',
        ],
    },
)
