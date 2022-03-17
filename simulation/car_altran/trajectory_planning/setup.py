from setuptools import setup

import os
from glob import glob

package_name = 'trajectory_planning'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, 'frenet_optimal_trajectory', 'cubic_spline', 'quintic_polynomials_planner'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dhruv Chad',
    maintainer_email='dhruv.chad@altran.com',
    description='Nodes related to trajectory planning',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'trajectory_planner = trajectory_planning.trajectory_planner_ros:main',
                'dummy_data_generator = trajectory_planning.dummy_data_generator:main',
        ],
    },
)
