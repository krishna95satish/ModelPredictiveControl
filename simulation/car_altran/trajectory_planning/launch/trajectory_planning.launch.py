from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
    get_package_share_directory('trajectory_planning'),
    'config',
    'default.yaml')

    return LaunchDescription([
        Node(
            package='trajectory_planning',
            executable='trajectory_planner',
            name='trajectory_planning_node',
            output='screen',
            parameters=[config]
        )
    ])
