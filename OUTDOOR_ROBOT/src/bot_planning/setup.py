from setuptools import setup


package_name = "bot_planning"


setup(
    name=package_name,
    version="0.0.0",
    packages=[],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (
            f"share/{package_name}/config",
            [
                "config/nav2.yaml",
                "config/navigate_to_pose_w_replanning_and_recovery.xml",
            ],
        ),
        (f"share/{package_name}/launch", ["launch/navigation.launch.py"]),
        (
            f"share/{package_name}/maps",
            ["../../maps/terrain_cost.yaml", "../../maps/terrain_cost.pgm"],
        ),
        (f"share/{package_name}", ["INTERFACE.md"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="aditya",
    maintainer_email="kotteaditya919@gmail.com",
    description="Motion planning and Nav2 navigation for the ITQ bot platform.",
    license="TODO: License declaration",
)
