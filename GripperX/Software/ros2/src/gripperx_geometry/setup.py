from setuptools import find_packages, setup


package_name = "gripperx_geometry"


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(where="src", exclude=["test"]),
    package_dir={"": "src"},
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", ["config/geometry.yaml"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="Theo Blaesse",
    maintainer_email="theoblaesse@gmail.com",
    description="Single source of truth for GripperX robot geometry.",
    license="TODO: License declaration",
    entry_points={
        "console_scripts": [
            "gripperx_geometry_generate = gripperx_geometry.generate:main",
        ],
    },
)
