from setuptools import setup
import os
from glob import glob

package_name = 'trajectory_following'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, 'Components'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='junyu zhou',
    maintainer_email='junyu.zhou@altran.com',
    description='Trajectory Following',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'trajectory_following = trajectory_following.trajectory_following_ros:main',
        'dummy_vehicle = trajectory_following.dummy_vehicle_ros:main',
        ],
    },
)
