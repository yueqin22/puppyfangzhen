from setuptools import setup
import os
from glob import glob

package_name = 'puppy_minicpm_robot'

setup(
    name=package_name,
    version='0.1.0',
    # Install ONLY the real package. Do not use a bare find_packages() here:
    # the sibling scripts/ directory carries an __init__.py, and installing it
    # as a top-level "scripts" package shadows gazebo_ros's own "scripts"
    # module (GazeboRosPaths), which breaks `ros2 launch` for Gazebo worlds.
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name] if os.path.exists('resource/' + package_name) else []),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'pyyaml', 'numpy'],
    zip_safe=True,
    maintainer='Puppy Developer',
    maintainer_email='developer@example.com',
    description='MiniCPM-RobotTrack visual tracking and navigation integration for Puppy quadruped robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_bridge_node = puppy_minicpm_robot.vision_bridge_node:main',
            'minicpm_track_node = puppy_minicpm_robot.minicpm_track_node:main',
            'track_cmd_adapter_node = puppy_minicpm_robot.track_cmd_adapter_node:main',
            'mission_grounder_node = puppy_minicpm_robot.mission_grounder_node:main',
        ],
    },
)
