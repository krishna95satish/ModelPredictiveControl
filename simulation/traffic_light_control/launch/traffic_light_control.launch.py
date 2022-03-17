from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os 
   
    
def generate_launch_description():
    config = os.path.join(
    get_package_share_directory('traffic_light_control'),
    'config',
    'default.yaml'
    )
    return LaunchDescription([
        Node(
            package='traffic_light_control',
            executable='traffic_light_control',
            name='traffic_light_control_node',
            output='screen',
            parameters = [config]
        )
    ])