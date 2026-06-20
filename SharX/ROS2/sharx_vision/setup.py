from setuptools import find_packages, setup

package_name = 'sharx_vision'

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
    maintainer='vineeth',
    maintainer_email='vineeth2320@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
	   'waste_detector = sharx_vision.waste_detector:main',
	   'waste_follower = sharx_vision.waste_follower:main', 
        ],
    },
)
