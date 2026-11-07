import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'trajectory_planner_g6'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aziz',
    maintainer_email='aziz@todo.todo',
    description='Visualisation RViz2 pour Kinova Gen3 6-DOF sous simulation MATLAB/ROS 2',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [],
    },
)
