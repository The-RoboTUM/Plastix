from setuptools import find_packages, setup


package_name = "gripperx_debug"


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(where="src", exclude=["test"]),
    package_dir={"": "src"},
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/debug_tools.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="aditya",
    maintainer_email="kotteaditya919@gmail.com",
    description="Debugging, logging, and plotting tools for the ITQ robot workspace.",
    license="TODO: License declaration",
    entry_points={
        "console_scripts": [
            "debug_log_node = gripperx_debug.debug_log_node:main",
            "plot_debug_csv = gripperx_debug.plot_debug_csv:main",
        ],
    },
)
