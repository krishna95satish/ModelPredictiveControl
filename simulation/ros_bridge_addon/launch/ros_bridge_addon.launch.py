from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os 
   
    
def generate_launch_description():
    config = os.path.join(
    get_package_share_directory('ros_bridge_addon'),
    'config',
    'default.yaml'
    )
    return LaunchDescription([
        Node(
            package='ros_bridge_addon',
            executable='ros_bridge_addon',
            name='ros_bridge_addon_node',
            output='screen',
            parameters = [config]
        )
    ])