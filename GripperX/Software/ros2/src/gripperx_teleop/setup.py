from setuptools import find_packages, setup

package_name = 'gripperx_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    # The browser UI's assets live inside the python package and are loaded
    # relative to web_server.py, so they must land next to it on install --
    # not under share/, which is not on the import path.
    package_data={package_name: ['web/*.html', 'web/*.css', 'web/*.js']},
    include_package_data=True,
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/config', ['config/teleop_mux.yaml', 'config/keyboard_teleop.yaml']),
        (f'share/{package_name}/launch', [
            'launch/laptop_teleop.launch.py',
            'launch/teleop_mux.launch.py',
            'launch/web_teleop.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    # colcon picks pytest over unittest only when the package declares it here;
    # without this line `colcon test` runs `python3 -m unittest`, which finds
    # nothing in test/ and exits GREEN on an empty suite.
    tests_require=["pytest"],
    zip_safe=True,
    maintainer='Theo Blässe',
    maintainer_email='theo.blaesse@itq.de',
    description='Teleop input multiplexer and keyboard teleop for GripperX.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'keyboard_teleop_node = gripperx_teleop.keyboard_teleop_node:main',
            'teleop_mux_node      = gripperx_teleop.teleop_mux_node:main',
            'web_teleop_node      = gripperx_teleop.web_teleop_node:main',
        ],
    },
)
