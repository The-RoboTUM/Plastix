import glob

from setuptools import find_packages, setup


package_name = "gripperx_localization"


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(where="src", exclude=["test"]),
    package_dir={"": "src"},
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", ["config/localization.yaml"]),
        (
            f"share/{package_name}/launch",
            ["launch/localization.launch.py"],
        ),
        (f"share/{package_name}/rviz", ["rviz/localization.rviz"]),
        (
            # DT-4: paketeigener Karten-Ordner statt Verweis auf die externe
            # maps/ (Workspace-Root, reale Roboter-Karten, Stack A). glob statt
            # fester Dateiliste, damit der Build nicht bricht, solange der
            # Ordner leer ist (Karten-Pfad-Luecke, DIGITAL_TWIN_PLAN.md §3.2.4),
            # und neue Sim-Karten (z.B. testworld_v1_map.*) automatisch mitgehen.
            f"share/{package_name}/maps",
            glob.glob("maps/*.yaml") + glob.glob("maps/*.pgm"),
        ),
    ],
    install_requires=["setuptools"],
    # colcon picks pytest over unittest only when the package declares it here;
    # without this line `colcon test` runs `python3 -m unittest`, which finds
    # nothing in test/ and exits GREEN on an empty suite.
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="aditya",
    maintainer_email="kotteaditya919@gmail.com",
    description="State estimation, mapping, and map-based pose for the ITQ bot platform.",
    license="TODO: License declaration",
    entry_points={
        "console_scripts": [
            "localization_input_node = gripperx_localization.localization_input_node:main",
            "ground_truth_odom_bridge = gripperx_localization.ground_truth_odom_bridge:main",
            "odom_divergence_monitor = gripperx_localization.odom_divergence_monitor:main",
        ],
    },
)
