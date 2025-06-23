from setuptools import setup
import os
from glob import glob

package_name = 'custom_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools', 'rclpy', 'geometry_msgs'],  # added dependencies
    zip_safe=True,
    maintainer='adithya',
    maintainer_email='adithyapothula123@gmail.com',
    description='Teleoperation for controlling both arm and wheels',
    license='Apache-2.0',  # updated license
    tests_require=['pytest'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['package.xml']),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    entry_points={
        'console_scripts': [
            'teleop_arm_node = custom_teleop.teleop_arm_node:main',
        ],
    },
)
