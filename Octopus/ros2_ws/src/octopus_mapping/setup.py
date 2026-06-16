from setuptools import find_packages, setup

package_name = 'octopus_mapping'

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
    description='Grid-map-style mapping prototype for Octopus.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'grid_map_builder_node = octopus_mapping.grid_map_builder_node:main',
        ],
    },
)
