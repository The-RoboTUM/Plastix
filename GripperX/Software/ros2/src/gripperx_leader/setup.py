from setuptools import find_packages, setup

package_name = 'gripperx_leader'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/config', ['config/leader_arm.yaml', 'config/leader_calibration.example.yaml',
                                       'config/leader_calibration.yaml']),
        (f'share/{package_name}/udev', ['udev/99-gripperx-leader.rules']),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='Theo Blässe',
    maintainer_email='theo.blaesse@itq.de',
    description='Leader-arm reader and limit recorder for GripperX.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'leader_arm_node = gripperx_leader.leader_arm_node:main',
            'record_limits = gripperx_leader.record_limits:main',
            'leader_eeprom = gripperx_leader.leader_eeprom:main',
            'record_follow = gripperx_leader.record_follow:main',
            'analyze_follow = gripperx_leader.record_follow:analyze_main',
        ],
    },
)
