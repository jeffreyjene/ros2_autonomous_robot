from setuptools import setup
from glob import glob
import os

package_name = 'robot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # Required ROS index entry
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        # Package.xml
        ('share/' + package_name, ['package.xml']),

        # Install config files (YAML)
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),

        # Install launch files (optional but recommended)
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jeffreyjene',
    maintainer_email='your@email.com',
    description='Robot motor control and Arduino bridge node',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arduino_bridge = robot_control.arduino_bridge:main',
            'cmd_vel_adapter = robot_control.cmd_vel_adapter:main',
        ],
    },
)