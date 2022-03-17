from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os 
   
    
def generate_launch_description():
    config = os.path.join(
    get_package_share_directory('aorta_server'),
    'config',
    'default.yaml'
    )
    return LaunchDescription([
        Node(
            package='aorta_server',
            executable='aorta_server',
            name='aorta_server_node',
            output='screen',
            parameters = [config]
        )
    ])