from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os 
   
    
def generate_launch_description():
    config = os.path.join(
    get_package_share_directory('default_filter'),
    'config',
    'default.yaml'
    )
    return LaunchDescription([
        Node(
            package='default_filter',
            executable='default_filter',
            name='default_filter_node',
            output='screen',
            parameters = [config]
        )
    ])