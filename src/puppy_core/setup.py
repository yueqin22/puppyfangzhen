from setuptools import setup

package_name = 'puppy_core'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', [
            'config/modes.yaml',
            'config/safety.yaml',
            'config/features.yaml',
            'config/navigation_targets.yaml',
            'config/patrol_routes.yaml',
        ]),
        ('share/' + package_name + '/launch', ['launch/core_bringup.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='puppy_team',
    maintainer_email='puppy@example.com',
    description='Platform-agnostic robot capability layer',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mode_manager = puppy_core.mode_manager:main',
            'safety_manager = puppy_core.safety_manager:main',
            'mission_manager = puppy_core.mission_manager:main',
            'state_aggregator = puppy_core.robot_state_aggregator:main',
            'goal_dispatcher = puppy_core.goal_dispatcher:main',
            'battery_status_adapter = puppy_core.battery_status_adapter:main',
            'fall_event_adapter = puppy_core.fall_event_adapter:main',
            'capability_registry = puppy_core.capability_registry_node:main',
        ],
    },
)
