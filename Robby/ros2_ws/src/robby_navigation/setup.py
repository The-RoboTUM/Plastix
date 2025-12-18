from setuptools import find_packages, setup

package_name = 'robby_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ardac',
    maintainer_email='arda.cigizoglu@robotum.info',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "robby_motor_driver = drivers.robby_motor_driver:main",
            "robby_sensor_driver = drivers.robby_sensor_driver:main",
            "robby_navigator = robby_navigation.robby_navigator:main"
        ],
    },
)
