import os
from glob import glob

from setuptools import find_packages, setup


package_name = "gripperx_external"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(where="src", exclude=["test"]),
    package_dir={"": "src"},
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    # colcon picks pytest over unittest only when the package declares it here;
    # without this line `colcon test` runs `python3 -m unittest`, which finds
    # nothing in test/ and exits GREEN on an empty suite.
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="Theo Blässe",
    maintainer_email="theo.blaesse@itq.de",
    description=(
        "External-goal link: Octopus litter positions over rosbridge, validated, "
        "resolved into robot standing poses, dispatched behind an arming gate."
    ),
    license="MIT",
    entry_points={
        # The pure modules (octopus_protocol, geodesy, grasp, validation,
        # arming) stay rclpy-free and are exercised with plain python3 and
        # nothing running - see test/check_*.py. Only these two wrappers touch
        # rclpy.
        #
        # There is deliberately no third executable. The Nav2 action client,
        # the PickPlastic client and the trash_goal_done acknowledgement (stage
        # 3) live in `goal_gateway_node` behind the arming gate, not in a
        # separate process that could be started on its own.
        "console_scripts": [
            "octopus_link_node = gripperx_external.octopus_link_node:main",
            "goal_gateway_node = gripperx_external.goal_gateway_node:main",
        ],
    },
)
