from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os 
   
    
def generate_launch_description():
    config = os.path.join(
    get_package_share_directory('altran_filter'),
    'config',
    'default.yaml'
    )
    return LaunchDescription([
        Node(
            package='altran_filter',
            executable='altran_filter',
            name='altran_filter_node',
            output='screen',
            parameters = [config]
        )
    ])