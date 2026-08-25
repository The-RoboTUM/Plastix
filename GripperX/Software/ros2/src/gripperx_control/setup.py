from setuptools import find_packages, setup


package_name = "gripperx_control"


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(where="src", exclude=["test"]),
    package_dir={"": "src"},
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (
            f"share/{package_name}/config",
            [
                "config/swerve_cmd.yaml",
                "config/teleop_joint_commands.yaml",
                "config/steer_servo.yaml",
                "config/ros2_controllers.yaml",
                # NFR-10: sim-only VALUE overlay for swerve_controller, layered
                # on top of ros2_controllers.yaml by the sim spawner (§3.1.6).
                "config/swerve_controller.sim.yaml",
                "config/lidar_power.yaml",
            ],
        ),
        (f"share/{package_name}/launch", ["launch/teleop.launch.py"]),
    ],
    install_requires=["setuptools"],
    # colcon picks pytest over unittest only when the package declares it here;
    # without this line `colcon test` runs `python3 -m unittest`, which finds
    # nothing in test/ and exits GREEN on an empty suite.
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="celeste",
    maintainer_email="mariaceleste.fernandez@robotum.info",
    description="Swerve control and ros2_control for the ITQ bot platform.",
    license="TODO: License declaration",
    entry_points={
        "console_scripts": [
            "hw_firmware_mock = gripperx_control.hw_firmware_mock:main",
            "teleop_joint_commands_node = gripperx_control.teleop_joint_commands_node:main",
            "steer_servo_node = gripperx_control.steer_servo_node:main",
            "steer_servo_calibrate = gripperx_control.steer_servo_calibrate:main",
            "lidar_power_node = gripperx_control.lidar_power_node:main",
        ],
    },
)
