"""
Setup for carla_ad_demo
"""
import os
from glob import glob
ROS_VERSION = int(os.environ['ROS_VERSION'])

if ROS_VERSION == 1:
    from distutils.core import setup
    from catkin_pkg.python_setup import generate_distutils_setup

    d = generate_distutils_setup(
        packages=['carla_aorta_test'],
    )

    setup(**d)

elif ROS_VERSION == 2:
    from setuptools import setup

    package_name = 'carla_aorta_test'
    setup(
        name=package_name,
        version='0.0.0',
        data_files=[
            ('share/ament_index/resource_index/packages',
             ['resource/' + package_name]),
            ('share/' + package_name, ['package.xml']),
            ('share/' + package_name + '/config', glob('config/*.xosc')),
            ('share/' + package_name + '/config', glob('config/*.rviz')),
            ('share/' + package_name + '/config', glob('config/*.json')),
            (os.path.join('share', package_name), glob('launch/*.launch.py'))
        ],
        install_requires=['setuptools'],
        zip_safe=True,
        maintainer='CARLA Simulator Team',
        maintainer_email='carla.simulator@gmail.com',
        description='CARLA ad demo for ROS2 bridge',
        license='MIT',
        tests_require=['pytest'],
    )
