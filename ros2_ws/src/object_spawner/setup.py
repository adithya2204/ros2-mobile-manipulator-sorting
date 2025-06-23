from setuptools import setup
import os
from glob import glob

package_name = 'object_spawner'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],  # the Python module folder (with __init__.py)
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        # add other resources if needed
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adithya',
    maintainer_email='adithyapothula123@gmail.com',
    description='Ramp Gazebo model with launch file',
    license='MIT',
    entry_points={
        'console_scripts': [
            'path_visualizer = object_spawner.path_visualizer:main',
        ],
    },
)
