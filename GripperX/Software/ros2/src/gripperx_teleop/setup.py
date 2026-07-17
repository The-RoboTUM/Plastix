from setuptools import find_packages, setup

package_name = 'gripperx_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/config', ['config/teleop_mux.yaml']),
        (f'share/{package_name}/launch', [
            'launch/laptop_teleop.launch.py',
            'launch/teleop_mux.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Theo Blässe',
    maintainer_email='theo.blaesse@itq.de',
    description='Teleop input multiplexer and keyboard teleop for GripperX.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'keyboard_teleop_node = gripperx_teleop.keyboard_teleop_node:main',
            'teleop_mux_node      = gripperx_teleop.teleop_mux_node:main',
        ],
    },
)
