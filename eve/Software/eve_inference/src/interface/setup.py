from setuptools import find_packages, setup

package_name = 'interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resource', ['resource/test_image.jpg'])
    ],
    package_data={
        'interface': [''],
    },
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jan',
    maintainer_email='jan.wech@robotum.info',
    description='ITQ-Project: Node for receiving Images and sending Coordinates',
    license='Apache License 2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'interface_node = interface.interface_node:main',
            'test_node = interface.test_node:main'
        ],
    },
)