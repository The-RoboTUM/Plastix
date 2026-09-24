from setuptools import find_packages, setup

setup(
    name='gripperx_arm',
    version='0.1.0',
    packages=find_packages(),
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'arm_action_server = gripperx_arm.arm_action_server:main',
        ],
    },
)
