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
        (f"share/{package_name}/config", ["config/sensors.yaml"]),
        (f"share/{package_name}/launch", ["launch/sensors.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="celeste",
    maintainer_email="mariaceleste.fernandez@robotum.info",
    description="LiDAR, IMU, and GPS drivers for the ITQ outdoor bot.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "sensor_mocks = gripperx_sensors.sensor_mocks:main",
        ],
    },
)
