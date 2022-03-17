from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os 
   
    
def generate_launch_description():
    config = os.path.join(
    get_package_share_directory('dummy_manual_control'),
    'config',
    'default.yaml'
    )
    return LaunchDescription([
        Node(
            package='dummy_manual_control',
            executable='dummy_manual_control',
            name='dummy_manual_control_node',
            output='screen',
            parameters = [config]
        )
    ])