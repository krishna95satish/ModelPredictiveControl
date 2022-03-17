from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='trajectory_following',
            executable='trajectory_following',
            name='trajectory_following_node',
            output='screen'
        )
    ])
