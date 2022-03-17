from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os 
   
    
def generate_launch_description():
    config = os.path.join(
    get_package_share_directory('trajectory_following_parameter'),
    'config',
    'trajectory_following_parameter.yaml'
    )
    return LaunchDescription([
        Node(
            package='trajectory_following_parameter',
            executable='trajectory_following_parameter',
            name='trajectory_following_parameter_node',
            output='screen',
            parameters = [config]
        )
    ])