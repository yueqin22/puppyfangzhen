from setuptools import setup

package_name = 'puppypi_mock'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/mock_bringup.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='puppy_team',
    maintainer_email='puppy@example.com',
    description='Mock PuppyPi platform',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fake_motion = puppypi_mock.fake_motion_server:main',
            'fake_status = puppypi_mock.fake_status_server:main',
            'fake_sensors = puppypi_mock.fake_sensor_publishers:main',
        ],
    },
)
