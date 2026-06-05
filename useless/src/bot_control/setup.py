from setuptools import find_packages, setup


package_name = "bot_control"


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
                "config/joint_command_bridge.yaml",
                "config/swerve_cmd.yaml",
                "config/ros2_controllers.yaml",
            ],
        ),
        (f"share/{package_name}/launch", ["launch/control.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="celeste",
    maintainer_email="mariaceleste.fernandez@robotum.info",
    description="Swerve control and ros2_control for the ITQ bot platform.",
    license="TODO: License declaration",
    entry_points={
        "console_scripts": [
            "joint_command_bridge = bot_control.joint_command_bridge:main",
            "swerve_cmd_node = bot_control.swerve_cmd_node:main",
        ],
    },
)
