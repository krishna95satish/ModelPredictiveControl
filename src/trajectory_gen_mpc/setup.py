from setuptools import setup

package_name = 'trajectory_gen_mpc'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='krishna',
    maintainer_email='krishna-satish.deshkulkarni-shankaranarayana@altran.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "trajectory_gen_mpc = trajectory_gen_mpc.main:main",
            "TrajectoryServer = trajectory_gen_mpc.TrajectoryGenServer:main",
            "CarlaClient = trajectory_gen_mpc.CarlaClient:main",
            "CarlaNode =trajectory_gen_mpc.CarlaNode:main",
            "TrajectorNode = trajectory_gen_mpc.TrajectorGenMpcNode:main",
            "TestVizNode = trajectory_gen_mpc.test_rviz:main",
            "OdomertyListener = trajectory_gen_mpc.OdometryListener:main"
        ],
    },
)
