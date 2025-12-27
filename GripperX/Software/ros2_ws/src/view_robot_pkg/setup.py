from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'view_robot_pkg'

def install_files_recursively(source_dir, install_base):
    """
    Creates (destination_dir, [files...]) entries for setuptools data_files.
    Keeps the subfolder structure below source_dir.
    """
    entries = []
    for root, _, files in os.walk(source_dir):
        if not files:
            continue
        rel_dir = os.path.relpath(root, source_dir)
        dest_dir = os.path.join(install_base, source_dir, rel_dir) if rel_dir != '.' else os.path.join(install_base, source_dir)
        file_paths = [os.path.join(root, f) for f in files]
        entries.append((dest_dir, file_paths))
    return entries

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        # meshes recursively with correct destination paths:
        *install_files_recursively('meshes', os.path.join('share', package_name)),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='apostolos-ubuntu-pc',
    maintainer_email='apostolos-ubuntu-pc@todo.todo',
    description='URDF visualization package for RViz2',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={'console_scripts': []},
)
