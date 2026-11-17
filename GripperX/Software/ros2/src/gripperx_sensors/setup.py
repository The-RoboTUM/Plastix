from setuptools import find_packages, setup


package_name = "gripperx_sensors"


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(where="src", exclude=["test"]),
    package_dir={"": "src"},
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", ["config/sensors.yaml", "config/bno085.yaml", "config/camera_front.yaml"]),
        (f"share/{package_name}/launch", ["launch/sensors.launch.py", "launch/bno085.launch.py", "launch/camera.launch.py"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="celeste",
    maintainer_email="mariaceleste.fernandez@robotum.info",
    description="LiDAR, IMU, and GPS drivers for the ITQ outdoor bot.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "sensor_mocks = gripperx_sensors.sensor_mocks:main",
            "scan_range_filter = gripperx_sensors.scan_range_filter:main",
            "bno085_imu = gripperx_sensors.bno085_node:main",
            "imu_axis_check = gripperx_sensors.imu_axis_check:main",
            "bno085_query = gripperx_sensors.bno085_query:main",
            "jpeg_camera = gripperx_sensors.jpeg_camera_node:main",
        ],
    },
)
