from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os 
   
    
def generate_launch_description():
    config = os.path.join(
    get_package_share_directory('akka_filter'),
    'config',
    'default.yaml'
    )
    return LaunchDescription([
        Node(
            package='akka_filter',
            executable='akka_filter',
            name='akka_filter_node',
            output='screen',
            parameters = [config]
        )
    ])