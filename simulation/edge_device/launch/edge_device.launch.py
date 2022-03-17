from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os 
   
    
def generate_launch_description():
    config = os.path.join(
    get_package_share_directory('edge_device'),
    'config',
    'default.yaml'
    )
    return LaunchDescription([
        Node(
            package='edge_device',
            executable='edge_device',
            name='edge_device_node',
            output='screen',
            parameters = [config]
        )
    ])